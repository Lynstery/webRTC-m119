#include "app/video_streaming/client/image_seq_video_track_source.h"

#include <cstdio>
#include <algorithm>
#include <chrono>

#include "rtc_base/logging.h"
#include "third_party/libyuv/include/libyuv/convert.h"

#define STB_IMAGE_IMPLEMENTATION
#include "app/video_streaming/client/stb_image.h"

rtc::scoped_refptr<ImageSequenceVideoTrackSource>
ImageSequenceVideoTrackSource::Create(Options opt) {
  auto src = rtc::make_ref_counted<ImageSequenceVideoTrackSource>(std::move(opt));
  src->Start();
  return src;
}

ImageSequenceVideoTrackSource::~ImageSequenceVideoTrackSource() {
  Stop();
}

ImageSequenceVideoTrackSource::ImageSequenceVideoTrackSource(Options opt)
    : webrtc::VideoTrackSource(false),
      opt_(Sanitize(std::move(opt))),
      frame_interval_us_(static_cast<int64_t>(1e6 / std::max(1e-9, opt_.fps))) {}

rtc::VideoSourceInterface<webrtc::VideoFrame>* ImageSequenceVideoTrackSource::source() {
  return &broadcaster_;
}

ImageSequenceVideoTrackSource::Options
ImageSequenceVideoTrackSource::Sanitize(Options in) {
  if (in.threads <= 0) in.threads = 2;
  in.threads = std::min(in.threads, 2);
  if (in.queue_capacity < 16) in.queue_capacity = 16;
  if (in.warmup_frames == 0) in.warmup_frames = std::min<size_t>(16, in.queue_capacity);
  if (in.warmup_frames > in.queue_capacity) in.warmup_frames = in.queue_capacity;
  return in;
}

void ImageSequenceVideoTrackSource::Start() {
  if (running_) return;
  running_ = true;

  RTC_LOG(LS_INFO) << "[ImageSequence] Starting with " << opt_.threads
                   << " threads, fps=" << opt_.fps;

  next_seq_to_decode_.store(0, std::memory_order_relaxed);

  for (int i = 0; i < opt_.threads; ++i) {
    workers_.emplace_back([this, i] {
      RTC_LOG(LS_INFO) << "[Worker-" << i << "] started.";
      WorkerLoop(i);
      RTC_LOG(LS_INFO) << "[Worker-" << i << "] stopped.";
    });
  }

  consumer_ = std::thread([this] {
    RTC_LOG(LS_INFO) << "[Consumer] started.";
    ConsumerLoop();
    RTC_LOG(LS_INFO) << "[Consumer] stopped.";
  });
}

void ImageSequenceVideoTrackSource::Stop() {
  if (!running_) return;
  running_ = false;

  producer_cv_.notify_all();
  ready_cv_.notify_all();

  for (auto& t : workers_)
    if (t.joinable()) t.join();
  if (consumer_.joinable()) consumer_.join();

  {
    std::lock_guard<std::mutex> lk(ready_mu_);
    ready_map_.clear();
  }
}

void ImageSequenceVideoTrackSource::WorkerLoop(int id) {
  while (running_) {
    {
      std::unique_lock<std::mutex> lk(ready_mu_);
      producer_cv_.wait(lk, [&] {
        return !running_ || ready_map_.size() < opt_.queue_capacity;
      });
      if (!running_) break;
    }

    const int64_t seq = next_seq_to_decode_.fetch_add(1, std::memory_order_relaxed);
    int file_index = IndexFromSeq(seq);
    char path[1024];
    std::snprintf(path, sizeof(path), opt_.pattern.c_str(), file_index);

    int w = 0, h = 0, comp = 0;
    stbi_uc* rgb = stbi_load(path, &w, &h, &comp, 3);
    if (!rgb) {
      RTC_LOG(LS_INFO) << "[Worker-" << id << "] failed seq=" << seq << " file=" << path;
      if (!opt_.loop_missing) StoreReady({seq, nullptr, 0, 0});
      continue;
    }

    auto i420 = webrtc::I420Buffer::Create(w, h);
    libyuv::RAWToI420(rgb, w * 3,
                      i420->MutableDataY(), i420->StrideY(),
                      i420->MutableDataU(), i420->StrideU(),
                      i420->MutableDataV(), i420->StrideV(),
                      w, h);

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

void ImageSequenceVideoTrackSource::StoreReady(Decoded d) {
  {
    std::lock_guard<std::mutex> lk(ready_mu_);
    if (!ready_reserved_) {
      ready_map_.reserve(opt_.queue_capacity * 2);
      ready_reserved_ = true;
    }
    ready_map_.emplace(d.seq, std::move(d));
  }
  ready_cv_.notify_one();
}

void ImageSequenceVideoTrackSource::ConsumerLoop() {
  WaitWarmup();

  rtc::scoped_refptr<webrtc::I420Buffer> last;
  int last_w = 0, last_h = 0;
  auto last_broadcast_time = std::chrono::steady_clock::now();

  const auto t0 = std::chrono::steady_clock::now();
  int64_t next_seq = 0;

  while (running_) {
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
          w = last_w;
          h = last_h;
        }
        ready_map_.erase(it);
      } else {
        buf = last;
        w = last_w;
        h = last_h;
      }
    }

    {
      std::lock_guard<std::mutex> lk(ready_mu_);
      if (ready_map_.size() < opt_.queue_capacity / 2)
        producer_cv_.notify_all();
    }

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

    auto now = std::chrono::steady_clock::now();
    double interval_ms =
        std::chrono::duration<double, std::milli>(now - last_broadcast_time).count();
    RTC_LOG(LS_INFO) << "[Consumer] frame seq=" << next_seq
                     << " interval=" << interval_ms << "ms";
    last_broadcast_time = now;

    broadcaster_.OnFrame(vf);
    ++next_seq;
  }
}

void ImageSequenceVideoTrackSource::WaitWarmup() {
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

int ImageSequenceVideoTrackSource::IndexFromSeq(int64_t seq) const {
  if (opt_.end_index > 0 && opt_.end_index >= opt_.start_index) {
    int span = opt_.end_index - opt_.start_index + 1;
    int off = static_cast<int>(seq % span);
    return opt_.start_index + off;
  }
  return opt_.start_index + static_cast<int>(seq);
}