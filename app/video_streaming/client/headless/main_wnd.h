#ifndef EXAMPLES_PEERCONNECTION_CLIENT_HEADLESS_MAIN_WND_H_
#define EXAMPLES_PEERCONNECTION_CLIENT_HEADLESS_MAIN_WND_H_

#include <stdint.h>

#include <cstdint>
#include <memory>
#include <string>

#include "api/media_stream_interface.h"
#include "api/scoped_refptr.h"
#include "api/video/video_frame.h"
#include "api/video/video_sink_interface.h"
#include "app/video_streaming/client/main_wnd.h"
#include "app/video_streaming/client/peer_connection_client.h"
#include "rtc_base/thread.h"
#include "rtc_base/task_queue.h"
#include "api/task_queue/task_queue_factory.h"
#include "app/video_streaming/client/image_seq_video_track_source.h"

class HeadlessMainWnd : public MainWindow {
 public:
  HeadlessMainWnd(const char* server, int port, bool autoconnect, bool autocall);
  ~HeadlessMainWnd();

  void RegisterObserver(MainWndCallback* callback) override;
  bool IsWindow() override;
  void SwitchToConnectUI() override;
  void SwitchToPeerList(const Peers& peers) override;
  void SwitchToStreamingUI() override;
  void MessageBox(const char* caption, const char* text, bool is_error) override;
  MainWindow::UI current_ui() override;
  void StartLocalRenderer(webrtc::VideoTrackInterface* local_video) override;
  void StopLocalRenderer() override;
  void StartRemoteRenderer(webrtc::VideoTrackInterface* remote_video) override;
  void StopRemoteRenderer() override;
  void QueueUIThreadCallback(int msg_id, void* data) override;

  void SetUIThread(rtc::Thread* thread);

  bool Create();

  bool Destroy();

 protected:
  class VideoRenderer : public rtc::VideoSinkInterface<webrtc::VideoFrame> {
   public:
    VideoRenderer(std::string save_path, webrtc::VideoTrackInterface* track_to_render);
    virtual ~VideoRenderer();

    // VideoSinkInterface implementation
    void OnFrame(const webrtc::VideoFrame& frame) override;

   protected:
    std::unique_ptr<FileNameMapper> filename_mapper_;
    std::string save_path_;
    rtc::scoped_refptr<webrtc::VideoTrackInterface> rendered_track_;
    std::unique_ptr<webrtc::TaskQueueFactory> task_queue_factory_;
    std::unique_ptr<rtc::TaskQueue> io_queue_;
    int last_rendered_frame_id_ = 0;
  };

 protected:
  bool is_window_;
  MainWindow::UI current_ui_;
  rtc::Thread* main_thread_;
  MainWndCallback* callback_;
  std::string server_;
  std::string port_;
  int port_int_;
  bool autoconnect_;
  bool autocall_;
  std::unique_ptr<VideoRenderer> video_renderer_;
};

#endif  // EXAMPLES_PEERCONNECTION_CLIENT_HEADLESS_MAIN_WND_H_
