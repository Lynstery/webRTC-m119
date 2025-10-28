#ifndef EXAMPLES_PEERCONNECTION_CLIENT_HEADLESS_MAIN_WND_H_
#define EXAMPLES_PEERCONNECTION_CLIENT_HEADLESS_MAIN_WND_H_

#include <stdint.h>

#include <memory>
#include <string>

#include "api/media_stream_interface.h"
#include "api/scoped_refptr.h"
#include "api/video/video_frame.h"
#include "api/video/video_sink_interface.h"
#include "app/video_streaming/client/main_wnd.h"
#include "app/video_streaming/client/peer_connection_client.h"
#include "rtc_base/thread.h"

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
  bool is_window_;
  MainWindow::UI current_ui_;
  rtc::Thread* main_thread_;
  MainWndCallback* callback_;
  std::string server_;
  std::string port_;
  int port_int_;
  bool autoconnect_;
  bool autocall_;
};

#endif  // EXAMPLES_PEERCONNECTION_CLIENT_HEADLESS_MAIN_WND_H_
