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
  if (in.queue_capacity < 32) in.queue_capacity = 32;
  return in;
}

void ImageSequenceVideoTrackSource::Start() {
  if (running_) return;
  running_ = true;

  RTC_LOG(LS_INFO) << "[ImageSequence] Starting with " << opt_.threads << " threads, fps=" << opt_.fps;

  queues_.clear();
  for (int i = 0; i < opt_.threads; ++i)
    queues_.emplace_back(std::make_unique<SPSCQueue<Decoded>>(opt_.queue_capacity));

  for (int i = 0; i < opt_.threads; ++i) {
    auto t = rtc::Thread::Create();
    t->SetName("ImgWorker-" + std::to_string(i), nullptr);
    t->Start();
    t->PostTask([this, i]() { WorkerLoop(i); });
    workers_.push_back(std::move(t));
  }

  consumer_ = rtc::Thread::Create();
  consumer_->SetName("ImgConsumer", nullptr);
  consumer_->Start();
  consumer_->PostTask([this]() { ConsumerLoop(); });
}

void ImageSequenceVideoTrackSource::Stop() {
  if (!running_) return;
  running_ = false;

  // 停止所有 worker
  for (auto& t : workers_) {
    if (t) t->Quit();
  }
  workers_.clear();

  // 停止 consumer
  if (consumer_) {
    consumer_->Quit();
    consumer_.reset();
  }

  RTC_LOG(LS_INFO) << "[ImageSequence] All threads stopped.";
}


void ImageSequenceVideoTrackSource::WorkerLoop(int id) {
  auto& q = *queues_[id];
  const int step = opt_.threads;
  int64_t seq = id + 1;
  const double target_interval_ms = frame_interval_us_ / 1000.0;

  while (running_) {
    if (q.size_approx() > opt_.queue_capacity) {
      int sleep_ms = static_cast<int>(target_interval_ms * opt_.queue_capacity / 2);
      // RTC_LOG(LS_INFO) << "Queue full, sleeping " << sleep_ms << " ms";
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      continue;
    }

    int file_index = IndexFromSeq(seq);
    char path[1024];
    std::snprintf(path, sizeof(path), opt_.pattern.c_str(), file_index);

    int w = 0, h = 0, comp = 0;

    stbi_uc* rgb = stbi_load(path, &w, &h, &comp, 3);
    if (!rgb) {
      RTC_LOG(LS_WARNING) << "Missing file: " << path;
      if (!opt_.loop_missing) q.try_enqueue({seq, nullptr, 0, 0});
      seq += step;
      continue;
    }
    // RTC_LOG(LS_INFO) << "Loaded seq=" << seq << " file=" << path;

    auto i420 = webrtc::I420Buffer::Create(w, h);
    libyuv::RAWToI420(rgb, w * 3,
                      i420->MutableDataY(), i420->StrideY(),
                      i420->MutableDataU(), i420->StrideU(),
                      i420->MutableDataV(), i420->StrideV(),
                      w, h);
    stbi_image_free(rgb);

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

    q.try_enqueue({seq, i420, w, h});
    seq += step;
  }
}

void ImageSequenceVideoTrackSource::ConsumerLoop() {
  WaitWarmup();

  const int n = opt_.threads;
  const double target_interval_ms = frame_interval_us_ / 1000.0;
  auto last_time = std::chrono::steady_clock::now();

  int64_t seq = 1;
  Decoded d_last;

  while (running_) {
    auto deadline = last_time + std::chrono::microseconds(frame_interval_us_);
    std::this_thread::sleep_until(deadline);
    if (!running_) break;

    int worker_id = (seq - 1) % n;
    Decoded d;
    if (!queues_[worker_id]->try_dequeue(d)) {
      RTC_LOG(LS_WARNING) << "Queue empty when seq=" << seq;
      d = d_last;
    }

    if (!d.i420) {
      RTC_LOG(LS_WARNING) << "Decoded frame is null at seq=" << seq;
      d = d_last;
    }

    const int64_t ts_us = seq * frame_interval_us_;
    webrtc::VideoFrame vf = webrtc::VideoFrame::Builder()
        .set_video_frame_buffer(d.i420)
        .set_timestamp_us(ts_us)
        .set_rotation(webrtc::kVideoRotation_0)
        .build();

    broadcaster_.OnFrame(vf);

    d_last = d;    

    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double, std::milli>(now - last_time).count();
    if (std::abs(dt - target_interval_ms) > 1.0) {
      RTC_LOG(LS_WARNING) << "seq=" << seq << " frame interval=" << dt << "ms";
    }
    last_time = now;
    ++seq;
  }
}

void ImageSequenceVideoTrackSource::WaitWarmup() {
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

int ImageSequenceVideoTrackSource::IndexFromSeq(int64_t seq) const {
  if (opt_.end_index > 0 && opt_.end_index >= opt_.start_index) {
    int span = opt_.end_index - opt_.start_index + 1;
    int off = static_cast<int>(seq % span);
    return opt_.start_index + off;
  }
  return opt_.start_index + static_cast<int>(seq);
}