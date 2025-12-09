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
#include <cstdint>
#include <string>

class FileNameMapper {
 public:
  FileNameMapper(std::string pattern,
                 int start_index,
                 int end_index = 0)
      : pattern_(std::move(pattern)),
        start_(start_index),
        end_(end_index) {}

  int IndexFromSeq(int64_t seq) const {
    if (end_ <= 0) {
      return start_ + static_cast<int>(seq - 1);
    }
    int n = end_ - start_ + 1;
    int offset = static_cast<int>((seq - 1) % n);
    return start_ + offset;
  }

  std::string GetFilePath(int64_t seq) const {
    int idx = IndexFromSeq(seq);

    char buf[1024];
    std::snprintf(buf, sizeof(buf), pattern_.c_str(), idx);
    return std::string(buf);
  }

  int GetIndex(int64_t seq) const {
    return IndexFromSeq(seq);
  }

 private:
  std::string pattern_;
  int start_ = 1;
  int end_ = 0;  // 0=无限序列
};

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
    size_t queue_capacity = 600;
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
    std::string file_path;
    bool valid() const { return i420 != nullptr; }
  };

  static Options Sanitize(Options in);
  void Start();
  void Stop();

  void WorkerLoop(int id);
  void ConsumerLoop();
  void WaitWarmup();

 private:
  Options opt_;
  std::unique_ptr<FileNameMapper> filename_mapper_;
  std::atomic<bool> running_{false};
  const int64_t frame_interval_us_;
  rtc::VideoBroadcaster broadcaster_;

  std::vector<std::unique_ptr<rtc::Thread>> workers_;
  std::unique_ptr<rtc::Thread> consumer_;
  std::vector<std::unique_ptr<SPSCQueue<Decoded>>> queues_;
};



#endif  // APP_VIDEO_STREAMING_CLIENT_IMAGE_SEQ_VIDEO_TRACK_SOURCE_H_
