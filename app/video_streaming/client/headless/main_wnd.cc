    
#include "app/video_streaming/client/headless/main_wnd.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cstdint>
#include <map>
#include <utility>

#include "api/video/i420_buffer.h"
#include "api/video/video_frame_buffer.h"
#include "api/video/video_rotation.h"
#include "api/video/video_source_interface.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "third_party/libyuv/include/libyuv/convert.h"
#include "third_party/libyuv/include/libyuv/convert_from.h"

HeadlessMainWnd::HeadlessMainWnd(const char* server,
                       int port,
                       bool autoconnect,
                       bool autocall)
    : is_window_(false),
      callback_(NULL),
      server_(server),
      autoconnect_(autoconnect),
      autocall_(autocall){
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%i", port);
  port_ = buffer;
  port_int_ = port;
  main_thread_ = rtc::Thread::Current();
}

HeadlessMainWnd::~HeadlessMainWnd() {
}

void HeadlessMainWnd::RegisterObserver(MainWndCallback* callback) {
  callback_ = callback;
}

bool HeadlessMainWnd::IsWindow() {
  return is_window_;
}

void HeadlessMainWnd::MessageBox(const char* caption,
                                  const char* text,
                                  bool is_error) {
  RTC_LOG(LS_INFO) << "UI Message:" << caption << ": " << text;
}

MainWindow::UI HeadlessMainWnd::current_ui() {
  return current_ui_; 
}

void HeadlessMainWnd::StartLocalRenderer(webrtc::VideoTrackInterface* local_video) {}
void HeadlessMainWnd::StopLocalRenderer() {}
void HeadlessMainWnd::StartRemoteRenderer(webrtc::VideoTrackInterface* remote_video) {}
void HeadlessMainWnd::StopRemoteRenderer() {}

void HeadlessMainWnd::QueueUIThreadCallback(int msg_id, void* data) {
  main_thread_->PostTask([cb = callback_, msg_id, data]() {
      cb->UIThreadCallback(msg_id, data);
  });
}

void HeadlessMainWnd::SetUIThread(rtc::Thread* thread) {
  main_thread_ = thread;
}

bool HeadlessMainWnd::Create() {
  is_window_ = true;
  SwitchToConnectUI();
  return true;
}

bool HeadlessMainWnd::Destroy() {
  is_window_ = false;
  return true;
}

void HeadlessMainWnd::SwitchToConnectUI() {
  current_ui_ = CONNECT_TO_SERVER;
  if (autoconnect_) callback_->StartLogin(server_, port_int_);
}

void HeadlessMainWnd::SwitchToPeerList(const Peers& peers) {
  current_ui_ = LIST_PEERS;
  if (autocall_ && !peers.empty()) {
    int peer_id = peers.begin()->first;
    callback_->ConnectToPeer(peer_id);
  }
}

void HeadlessMainWnd::SwitchToStreamingUI() {
  current_ui_ = STREAMING;
}


