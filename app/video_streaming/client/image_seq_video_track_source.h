#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <algorithm>

#include "api/video/i420_buffer.h"
#include "api/video/video_frame.h"
#include "media/base/video_broadcaster.h"
#include "pc/video_track_source.h"
#include "rtc_base/logging.h"
#include "third_party/libyuv/include/libyuv/convert.h"

#define STB_IMAGE_IMPLEMENTATION
#include "app/video_streaming/client/stb_image.h"

class ImageSequenceVideoTrackSource : public webrtc::VideoTrackSource {
 public:
  struct Options {
    std::string pattern;
    int start_index = 1;
    int end_index   = 0;        // [start,end] 环回；0 表示无限序列
    double fps      = 30.0;
    int fixed_width  = 0;
    int fixed_height = 0;
    size_t queue_capacity = 16; // 就绪帧最大在制上限（保护内存与背压）
    size_t warmup_frames  = 8;  // 预热目标
    int threads = 2;            // 解码线程数（建议 1~2）
    bool loop_missing = true;   // 缺帧是否忽略
  };

  static rtc::scoped_refptr<ImageSequenceVideoTrackSource> Create(Options opt) {
    auto src = rtc::make_ref_counted<ImageSequenceVideoTrackSource>(std::move(opt));
    src->Start();
    return src;
  }

  ~ImageSequenceVideoTrackSource() override { Stop(); }

 protected:
  explicit ImageSequenceVideoTrackSource(Options opt)
      : webrtc::VideoTrackSource(false),
        opt_(Sanitize(std::move(opt))),
        frame_interval_us_(static_cast<int64_t>(1e6 / std::max(1e-9, opt_.fps))) {}

  rtc::VideoSourceInterface<webrtc::VideoFrame>* source() override {
    return &broadcaster_;
  }

 private:
  struct Decoded {
    int64_t seq = -1;
    rtc::scoped_refptr<webrtc::I420Buffer> i420;
    int width = 0;
    int height = 0;
    bool valid() const { return i420 != nullptr; }
  };

  static Options Sanitize(Options in) {
    if (in.threads <= 0) in.threads = 2;
    in.threads = std::min(in.threads, 2); // 限制并发上限，稳定优先
    if (in.queue_capacity < 16) in.queue_capacity = 16;
    if (in.warmup_frames == 0) in.warmup_frames = std::min<size_t>(16, in.queue_capacity);
    if (in.warmup_frames > in.queue_capacity) in.warmup_frames = in.queue_capacity;
    return in;
  }

  void Start() {
    if (running_) return;
    running_ = true;

    RTC_LOG(LS_INFO) << "[ImageSequence] Starting video track source with "
                     << opt_.threads << " workers, fps=" << opt_.fps
                     << ", queue_capacity=" << opt_.queue_capacity;

    next_seq_to_decode_.store(0, std::memory_order_relaxed);

    // 启动生产者线程
    for (int i = 0; i < opt_.threads; ++i) {
      workers_.emplace_back([this, i]{
        RTC_LOG(LS_INFO) << "[Worker-" << i << "] started.";
        WorkerLoop(i);
        RTC_LOG(LS_INFO) << "[Worker-" << i << "] stopped.";
      });
    }

    // 启动消费者线程
    consumer_ = std::thread([this]{
      RTC_LOG(LS_INFO) << "[Consumer] started.";
      ConsumerLoop();
      RTC_LOG(LS_INFO) << "[Consumer] stopped.";
    });
  }
  void Stop() {
    if (!running_) return;
    running_ = false;

    // 唤醒全部等待者
    producer_cv_.notify_all();
    ready_cv_.notify_all();

    for (auto& t : workers_) if (t.joinable()) t.join();
    if (consumer_.joinable()) consumer_.join();

    // 清理
    {
      std::lock_guard<std::mutex> lk(ready_mu_);
      ready_map_.clear();
    }
  }

  void WorkerLoop(int id) {
    while (running_) {
      // 背压：控制就绪帧上限，避免乱序积压过多
      {
        std::unique_lock<std::mutex> lk(ready_mu_);

        producer_cv_.wait(lk, [&]{
          return !running_ || ready_map_.size() < opt_.queue_capacity;
        });

        if (!running_) break;
      }

      const int64_t seq = next_seq_to_decode_.fetch_add(1, std::memory_order_relaxed);

      //RTC_LOG(LS_INFO) << "[Worker-" << id << "] fetch frame seq=" << seq;

      // 映射到文件编号与路径
      int file_index = IndexFromSeq(seq);
      char path[1024];
      std::snprintf(path, sizeof(path), opt_.pattern.c_str(), file_index);

      // 同步读盘 + 解码
      int w=0, h=0, comp=0;
      stbi_uc* rgb = stbi_load(path, &w, &h, &comp, 3);
      if (!rgb) {
        RTC_LOG(LS_INFO) << "[Worker-" << id << "] failed decoding seq=" << seq << " file=" << path;
        if (!opt_.loop_missing) {
          StoreReady({seq, nullptr, 0, 0});
        }
        continue;
      }

      //RTC_LOG(LS_INFO) << "[Worker-" << id << "] decoded seq=" << seq << " file=" << path;
      // RGB24 -> I420
      auto i420 = webrtc::I420Buffer::Create(w, h);
      libyuv::RAWToI420(rgb, w * 3,
                          i420->MutableDataY(), i420->StrideY(),
                          i420->MutableDataU(), i420->StrideU(),
                          i420->MutableDataV(), i420->StrideV(),
                          w, h);

      // 可选缩放
      if (opt_.fixed_width > 0 && opt_.fixed_height > 0 &&
          (w != opt_.fixed_width || h != opt_.fixed_height)) {
        auto scaled = webrtc::I420Buffer::Create(opt_.fixed_width, opt_.fixed_height);
        libyuv::I420Scale(i420->DataY(), i420->StrideY(),
                          i420->DataU(), i420->StrideU(),
                          i420->DataV(), i420->StrideV(),
                          w, h,
                          scaled->MutableDataY(), scaled->StrideY(),
                          scaled->MutableDataU(), scaled->StrideU(),
                          scaled->MutableDataV(), scaled->StrideV(),
                          opt_.fixed_width, opt_.fixed_height,
                          libyuv::kFilterLinear);
        i420 = scaled;
        w = opt_.fixed_width;
        h = opt_.fixed_height;
      }
      stbi_image_free(rgb);

      StoreReady({seq, i420, w, h});
    }
  }

  // 生产者写入就绪表（受背压控制）
  void StoreReady(Decoded d) {
    {
      std::lock_guard<std::mutex> lk(ready_mu_);
      // 预留 buckets 降低 rehash（只做一次，不必每次 reserve）
      if (!ready_reserved_) {
        ready_map_.reserve(opt_.queue_capacity * 2);
        ready_reserved_ = true;
      }
      ready_map_.emplace(d.seq, std::move(d));
    }
    // 唤醒 consumer
    ready_cv_.notify_one();
  }

  void ConsumerLoop() {
    WaitWarmup();

    rtc::scoped_refptr<webrtc::I420Buffer> last;
    int last_w = 0, last_h = 0;

    auto last_broadcast_time = std::chrono::steady_clock::now();

    const auto t0 = std::chrono::steady_clock::now();
    int64_t next_seq = 0;

    while (running_) {
      // 精确到帧时钟
      auto deadline = t0 + std::chrono::microseconds(next_seq * frame_interval_us_);
      std::this_thread::sleep_until(deadline);
      if (!running_) break;

      rtc::scoped_refptr<webrtc::I420Buffer> buf;
      int w = 0, h = 0;

      {
        std::unique_lock<std::mutex> lk(ready_mu_);

        ready_cv_.wait(lk, [&] {
          return !running_ || ready_map_.find(next_seq) != ready_map_.end();
        });
        if (!running_) break;

        auto it = ready_map_.find(next_seq);
        if (it != ready_map_.end()) {
          if (it->second.valid()) {
            buf = it->second.i420;
            w = it->second.width;
            h = it->second.height;
            last = buf;
            last_w = w;
            last_h = h;
          } else {
            buf = last;
            w = last_w; h = last_h;
          }
          ready_map_.erase(it);
        } else {
          RTC_LOG(LS_INFO) << "[Consumer] ready_map is empty! Frame not delivered in time : seq=" << next_seq;
          buf = last;
          w = last_w; h = last_h;
        }
      }

      {
        std::lock_guard<std::mutex> lk(ready_mu_);
        if (ready_map_.size() < opt_.queue_capacity / 2) {
          // RTC_LOG(LS_VERBOSE) << "[Consumer] notified producers, queue size =" << ready_map_.size();
          producer_cv_.notify_all();
        }
      }

      // 可能启动阶段 last 还为空
      if (!buf) {
        ++next_seq;
        continue;
      }

      const int64_t ts_us = next_seq * frame_interval_us_;
      webrtc::VideoFrame vf = webrtc::VideoFrame::Builder()
                                  .set_video_frame_buffer(buf)
                                  .set_timestamp_us(ts_us)
                                  .set_rotation(webrtc::kVideoRotation_0)
                                  .build();

      // 同步广播到 sinks（编码/发送在内部异步）
      auto now = std::chrono::steady_clock::now();
      double interval_ms = std::chrono::duration<double, std::milli>(now - last_broadcast_time).count();
      RTC_LOG(LS_INFO) << "[Consumer] broadcast frame seq=" << next_seq << " interval=" << interval_ms << "ms";
      last_broadcast_time = now; 
      broadcaster_.OnFrame(vf);
      ++next_seq;
    }
  }

  void WaitWarmup() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (running_) {
      size_t sz = 0;
      {
        std::lock_guard<std::mutex> lk(ready_mu_);
        sz = ready_map_.size();
      }
      if (sz >= opt_.warmup_frames) return;
      if (std::chrono::steady_clock::now() > deadline) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // 序列号 → 文件索引
  int IndexFromSeq(int64_t seq) const {
    if (opt_.end_index > 0 && opt_.end_index >= opt_.start_index) {
      int span = opt_.end_index - opt_.start_index + 1;
      int off = static_cast<int>(seq % span);
      return opt_.start_index + off;
    }
    return opt_.start_index + static_cast<int>(seq);
  }

 private:
  Options opt_;
  std::atomic<bool> running_{false};
  const int64_t frame_interval_us_;
  rtc::VideoBroadcaster broadcaster_;

  // 两层：解码 workers + consumer
  std::vector<std::thread> workers_;
  std::thread consumer_;

  // 任务分配器：全局递增的解码序号
  std::atomic<int64_t> next_seq_to_decode_{0};

  // 乱序产物缓冲（受背压限制）
  std::mutex ready_mu_;
  std::condition_variable ready_cv_;    // 唤醒 consumer
  std::condition_variable producer_cv_; // 背压：唤醒生产者
  std::unordered_map<int64_t, Decoded> ready_map_;
  bool ready_reserved_ = false;
};