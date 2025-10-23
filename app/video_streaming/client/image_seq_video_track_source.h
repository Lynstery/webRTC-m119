#ifndef APP_VIDEO_STREAMING_CLIENT_IMAGE_SEQ_VIDEO_TRACK_SOURCE_H_
#define APP_VIDEO_STREAMING_CLIENT_IMAGE_SEQ_VIDEO_TRACK_SOURCE_H_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "api/video/i420_buffer.h"
#include "api/video/video_frame.h"
#include "media/base/video_broadcaster.h"
#include "pc/video_track_source.h"
#include "rtc_base/ref_counted_object.h"
#include "rtc_base/thread.h"
#include "app/video_streaming/client/readerwriterqueue.h"

template<typename T>
class SPSCQueue {
 public:
  explicit SPSCQueue(size_t capacity) : q_(capacity) {}
  bool try_enqueue(const T& v) { return q_.try_enqueue(v); }
  bool try_dequeue(T& v) { return q_.try_dequeue(v); }
  size_t size_approx() const { return q_.size_approx(); }
 private:
  moodycamel::ReaderWriterQueue<T> q_;
};


class ImageSequenceVideoTrackSource : public webrtc::VideoTrackSource {
 public:
  struct Options {
    std::string pattern;
    int start_index = 1;
    int end_index = 0;         // [start, end] 环回；0 表示无限序列
    double fps = 30.0;
    int fixed_width = 0;
    int fixed_height = 0;
    size_t queue_capacity = 32;
    int threads = 2;
    bool loop_missing = true;
  };

  static rtc::scoped_refptr<ImageSequenceVideoTrackSource> Create(Options opt);
  ~ImageSequenceVideoTrackSource() override;

 protected:
  explicit ImageSequenceVideoTrackSource(Options opt);
  rtc::VideoSourceInterface<webrtc::VideoFrame>* source() override;

 private:
  struct Decoded {
    int64_t seq = -1;
    rtc::scoped_refptr<webrtc::I420Buffer> i420;
    int width = 0;
    int height = 0;
    bool valid() const { return i420 != nullptr; }
  };

  static Options Sanitize(Options in);
  void Start();
  void Stop();

  void WorkerLoop(int id);
  void ConsumerLoop();
  void WaitWarmup();
  int IndexFromSeq(int64_t seq) const;

 private:
  Options opt_;
  std::atomic<bool> running_{false};
  const int64_t frame_interval_us_;
  rtc::VideoBroadcaster broadcaster_;

  std::vector<std::unique_ptr<rtc::Thread>> workers_;
  std::unique_ptr<rtc::Thread> consumer_;
  std::vector<std::unique_ptr<SPSCQueue<Decoded>>> queues_;
};

#endif  // APP_VIDEO_STREAMING_CLIENT_IMAGE_SEQ_VIDEO_TRACK_SOURCE_H_
