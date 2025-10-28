/*
 *  Copyright 2012 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <stdio.h>
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

#include "app/video_streaming/client/headless/main_wnd.h"
#include "absl/flags/parse.h"
#include "api/scoped_refptr.h"
#include "app/video_streaming/client/conductor.h"
#include "app/video_streaming/client/flag_defs.h"
#include "app/video_streaming/client/peer_connection_client.h"
#include "rtc_base/physical_socket_server.h"
#include "rtc_base/ssl_adapter.h"
#include "rtc_base/thread.h"
#include "system_wrappers/include/field_trial.h"
#include "test/field_trial.h"
#include "rtc_base/logging.h"
#include "app/video_streaming/client/log.h"


class CustomSocketServer : public rtc::PhysicalSocketServer {
 public:
  explicit CustomSocketServer(HeadlessMainWnd* wnd)
      : wnd_(NULL), conductor_(NULL), client_(NULL) {}
  virtual ~CustomSocketServer() {}

  void SetMessageQueue(rtc::Thread* queue) override { message_queue_ = queue; }
  void set_wnd(HeadlessMainWnd* wnd) { wnd_ = wnd; }
  void set_client(PeerConnectionClient* client) { client_ = client; }
  void set_conductor(Conductor* conductor) { conductor_ = conductor; }

  bool Wait(webrtc::TimeDelta max_wait_duration, bool process_io) override {
    if (!wnd_->IsWindow() && !conductor_->connection_active() &&
        client_ != NULL && !client_->is_connected()) {
      message_queue_->Quit();
    }
    return rtc::PhysicalSocketServer::Wait(webrtc::TimeDelta::Zero(),
                                           process_io);
  }

 protected:
  rtc::Thread* message_queue_;
  HeadlessMainWnd* wnd_;
  Conductor* conductor_;
  PeerConnectionClient* client_;
};

int main(int argc, char* argv[]) {

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

  static CustomLogSink sink({
      "stun", "ice", "p2p_transport_channel", 
       "port", "candidate", "interface"
  });
  rtc::LogMessage::LogToDebug(rtc::LS_NONE);
  rtc::LogMessage::AddLogToStream(&sink, rtc::LS_INFO); 

  CustomSocketServer socket_server(nullptr);

  rtc::AutoSocketServerThread thread(&socket_server);
  thread.SetName("MainThread", nullptr);

  const std::string server = absl::GetFlag(FLAGS_server);
  HeadlessMainWnd wnd(server.c_str(), absl::GetFlag(FLAGS_port),
                 true /* autoconnect */,
                 absl::GetFlag(FLAGS_autocall));

  socket_server.set_wnd(&wnd);
  wnd.SetUIThread(rtc::Thread::Current());


  rtc::InitializeSSL();
  // Must be constructed after we set the socketserver.
  PeerConnectionClient client;
  auto conductor = rtc::make_ref_counted<Conductor>(&client, &wnd);
  socket_server.set_client(&client);
  socket_server.set_conductor(conductor.get());
  
  wnd.Create();

  thread.Run();

  rtc::CleanupSSL();
  return 0;
}


