/*
 *  Copyright 2012 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <glib.h>
#include <gtk/gtk.h>
#include <stdio.h>

#include "absl/flags/parse.h"
#include "api/scoped_refptr.h"
#include "app/video_streaming/client/conductor.h"
#include "app/video_streaming/client/flag_defs.h"
#include "app/video_streaming/client/linux/main_wnd.h"
#include "app/video_streaming/client/peer_connection_client.h"
#include "rtc_base/physical_socket_server.h"
#include "rtc_base/ssl_adapter.h"
#include "rtc_base/thread.h"
#include "system_wrappers/include/field_trial.h"
#include "test/field_trial.h"
#include "rtc_base/logging.h"
#include "absl/strings/match.h"
#include "absl/strings/ascii.h"
#include <string>
#include <vector>
#include <mutex>
#include <chrono>
#include <ctime>
#include <mutex>
#include <vector>
#include <string>
#include <cstdio>
#include <utility>
#include "rtc_base/logging.h"
#include "absl/strings/ascii.h"
#include "absl/strings/match.h"

class FilteringLogSink : public rtc::LogSink {
 public:
  explicit FilteringLogSink(std::vector<std::string> blocked_keywords)
      : blocked_keywords_(std::move(blocked_keywords)) {
    for (auto& kw : blocked_keywords_) {
      absl::AsciiStrToLower(&kw);
    }
  }

  void OnLogMessage(const std::string& message) override {
    if (ShouldBlock(message)) return;

    std::lock_guard<std::mutex> lock(mu_);
    PrintWithTimestamp(message);
  }

  void OnLogMessage(absl::string_view message) override {
    std::string msg(message);
    if (ShouldBlock(msg)) return;

    std::lock_guard<std::mutex> lock(mu_);
    PrintWithTimestamp(msg);
  }

 private:
  bool ShouldBlock(const std::string& msg) {
    std::string lower = absl::AsciiStrToLower(msg);
    for (const auto& kw : blocked_keywords_) {
      if (absl::StrContains(lower, kw)) return true;
    }
    return false;
  }

  static std::string CurrentThreadLabel() {
    // 线程名优先；没有就用线程ID
    if (auto* t = rtc::Thread::Current()) {
      std::string name = t->name();   // 拷贝，避免临时生命周期问题
      if (!name.empty()) return name;
    }
    // 未注册为 rtc::Thread 或没有名字：退化到线程ID
    std::ostringstream os;
    os << rtc::CurrentThreadId();
    return os.str();
  }

  void PrintWithTimestamp(const std::string& msg) {
    // 获取当前系统时间
    using namespace std::chrono;
    auto now = system_clock::now();
    auto t = system_clock::to_time_t(now);
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::tm tm_buf;
    localtime_r(&t, &tm_buf);

    char ts[64];
    std::snprintf(ts, sizeof(ts),
                  "%04d-%02d-%02d %02d:%02d:%02d.%03d",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
                  static_cast<int>(ms.count()));
    const std::string thr = CurrentThreadLabel();
    // 输出格式: [时间戳] 原始日志内容
    fprintf(stderr, "[%s] [%s] %s\n", ts, thr.c_str(), msg.c_str());
  }

  std::vector<std::string> blocked_keywords_;
  std::mutex mu_;
};


class CustomSocketServer : public rtc::PhysicalSocketServer {
 public:
  explicit CustomSocketServer(GtkMainWnd* wnd)
      : wnd_(wnd), conductor_(NULL), client_(NULL) {}
  virtual ~CustomSocketServer() {}

  void SetMessageQueue(rtc::Thread* queue) override { message_queue_ = queue; }

  void set_client(PeerConnectionClient* client) { client_ = client; }
  void set_conductor(Conductor* conductor) { conductor_ = conductor; }

  // Override so that we can also pump the GTK message loop.
  // This function never waits.
  bool Wait(webrtc::TimeDelta max_wait_duration, bool process_io) override {
    // Pump GTK events.
    // TODO(henrike): We really should move either the socket server or UI to a
    // different thread.  Alternatively we could look at merging the two loops
    // by implementing a dispatcher for the socket server and/or use
    // g_main_context_set_poll_func.
    while (gtk_events_pending())
      gtk_main_iteration();

    if (!wnd_->IsWindow() && !conductor_->connection_active() &&
        client_ != NULL && !client_->is_connected()) {
      message_queue_->Quit();
    }
    return rtc::PhysicalSocketServer::Wait(webrtc::TimeDelta::Zero(),
                                           process_io);
  }

 protected:
  rtc::Thread* message_queue_;
  GtkMainWnd* wnd_;
  Conductor* conductor_;
  PeerConnectionClient* client_;
};

int main(int argc, char* argv[]) {
  gtk_init(&argc, &argv);
// g_type_init API is deprecated (and does nothing) since glib 2.35.0, see:
// https://mail.gnome.org/archives/commits-list/2012-November/msg07809.html
#if !GLIB_CHECK_VERSION(2, 35, 0)
  g_type_init();
#endif
// g_thread_init API is deprecated since glib 2.31.0, see release note:
// http://mail.gnome.org/archives/gnome-announce-list/2011-October/msg00041.html
#if !GLIB_CHECK_VERSION(2, 31, 0)
  g_thread_init(NULL);
#endif

  absl::ParseCommandLine(argc, argv);

  // InitFieldTrialsFromString stores the char*, so the char array must outlive
  // the application.
  const std::string forced_field_trials =
      absl::GetFlag(FLAGS_force_fieldtrials);
  webrtc::field_trial::InitFieldTrialsFromString(forced_field_trials.c_str());

  // Abort if the user specifies a port that is outside the allowed
  // range [1, 65535].
  if ((absl::GetFlag(FLAGS_port) < 1) || (absl::GetFlag(FLAGS_port) > 65535)) {
    printf("Error: %i is not a valid port.\n", absl::GetFlag(FLAGS_port));
    return -1;
  }

  static FilteringLogSink sink({
      "stun", "ice", "p2p_transport_channel", 
       "port", "candidate", "interface"
  });
  rtc::LogMessage::LogToDebug(rtc::LS_NONE);
  rtc::LogMessage::AddLogToStream(&sink, rtc::LS_INFO); 

  const std::string server = absl::GetFlag(FLAGS_server);
  GtkMainWnd wnd(server.c_str(), absl::GetFlag(FLAGS_port),
                 absl::GetFlag(FLAGS_autoconnect),
                 absl::GetFlag(FLAGS_autocall));
  wnd.Create();

  CustomSocketServer socket_server(&wnd);
  rtc::AutoSocketServerThread thread(&socket_server);
  thread.SetName("MainThread", nullptr);


  rtc::InitializeSSL();
  // Must be constructed after we set the socketserver.
  PeerConnectionClient client;
  auto conductor = rtc::make_ref_counted<Conductor>(&client, &wnd);
  socket_server.set_client(&client);
  socket_server.set_conductor(conductor.get());

  thread.Run();

  // gtk_main();
  wnd.Destroy();

  // TODO(henrike): Run the Gtk main loop to tear down the connection.
  /*
  while (gtk_events_pending()) {
    gtk_main_iteration();
  }
  */
  rtc::CleanupSSL();
  return 0;
}
