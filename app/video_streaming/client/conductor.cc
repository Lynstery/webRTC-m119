/*
 *  Copyright 2012 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "app/video_streaming/client/conductor.h"

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "api/audio/audio_mixer.h"
#include "api/audio_codecs/audio_decoder_factory.h"
#include "api/audio_codecs/audio_encoder_factory.h"
#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "api/audio_codecs/builtin_audio_encoder_factory.h"
#include "api/audio_options.h"
#include "api/create_peerconnection_factory.h"
#include "api/rtp_sender_interface.h"
#include "api/video_codecs/video_decoder_factory.h"
#include "api/video_codecs/video_decoder_factory_template.h"
#include "api/video_codecs/video_decoder_factory_template_dav1d_adapter.h"
#include "api/video_codecs/video_decoder_factory_template_libvpx_vp8_adapter.h"
#include "api/video_codecs/video_decoder_factory_template_libvpx_vp9_adapter.h"
#include "api/video_codecs/video_decoder_factory_template_open_h264_adapter.h"
#include "api/video_codecs/video_encoder_factory.h"
#include "api/video_codecs/video_encoder_factory_template.h"
#include "api/video_codecs/video_encoder_factory_template_libaom_av1_adapter.h"
#include "api/video_codecs/video_encoder_factory_template_libvpx_vp8_adapter.h"
#include "api/video_codecs/video_encoder_factory_template_libvpx_vp9_adapter.h"
#include "api/video_codecs/video_encoder_factory_template_open_h264_adapter.h"
#include "app/video_streaming/client/defaults.h"
#include "modules/audio_device/include/audio_device.h"
#include "modules/audio_processing/include/audio_processing.h"
#include "modules/video_capture/video_capture.h"
#include "modules/video_capture/video_capture_factory.h"
#include "p2p/base/port_allocator.h"
#include "pc/video_track_source.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/rtc_certificate_generator.h"
#include "rtc_base/strings/json.h"
#include "test/vcm_capturer.h"

#include "api/video/video_frame.h"
#include "api/video/i420_buffer.h"
#include "rtc_base/ref_counted_object.h"
#include "rtc_base/time_utils.h"
#include "rtc_base/thread.h"
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <cstring>
#include <string>

#include "app/video_streaming/client/image_seq_video_track_source.h"

#include "api/stats/rtcstats_objects.h"
#include "api/stats/rtc_stats_collector_callback.h"

namespace {

class PeriodicStatsCallback : public webrtc::RTCStatsCollectorCallback {
  
  static void StatsReport(const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) {
    uint64_t bytes_sent = 0, bytes_recv = 0;
    double rtt_ms = 0.0;

    double fps_send = 0, fps_recv = 0;
    uint32_t width_send = 0, height_send = 0;
    uint32_t width_recv = 0, height_recv = 0;

    double encode_time = 0.0, target_bitrate = 0.0;
    uint32_t frames_encoded = 0, frames_decoded = 0, frames_dropped = 0;
    uint64_t qp_sum = 0;
    std::string quality_reason;
    double avg_qp = 0.0;

    double jitter_ms = 0.0;
    int32_t packets_lost = 0;
    uint32_t pli_count = 0, fir_count = 0, nack_count = 0;

    for (const auto& s : *report) {
      if (s.type() == webrtc::RTCOutboundRtpStreamStats::kType) {
        const auto& outbound = s.cast_to<webrtc::RTCOutboundRtpStreamStats>();

        if (outbound.bytes_sent.is_defined())
          bytes_sent += *outbound.bytes_sent;
        if (outbound.frames_per_second.is_defined())
          fps_send = *outbound.frames_per_second;
        if (outbound.frame_width.is_defined())
          width_send = *outbound.frame_width;
        if (outbound.frame_height.is_defined())
          height_send = *outbound.frame_height;
        if (outbound.frames_encoded.is_defined())
          frames_encoded = *outbound.frames_encoded;
        if (outbound.total_encode_time.is_defined())
          encode_time = *outbound.total_encode_time;
        if (outbound.target_bitrate.is_defined())
          target_bitrate = *outbound.target_bitrate;
        if (outbound.qp_sum.is_defined())
          qp_sum = *outbound.qp_sum;
        if (outbound.quality_limitation_reason.is_defined())
          quality_reason = *outbound.quality_limitation_reason;
        if (outbound.quality_limitation_resolution_changes.is_defined()) {
          RTC_LOG(LS_VERBOSE) << "[Stats] Resolution changes: "
                              << *outbound.quality_limitation_resolution_changes;
        }

        if (qp_sum > 0 && frames_encoded > 0)
          avg_qp = static_cast<double>(qp_sum) / frames_encoded;
      }

      if (s.type() == webrtc::RTCInboundRtpStreamStats::kType) {
        const auto& inbound = s.cast_to<webrtc::RTCInboundRtpStreamStats>();

        if (inbound.bytes_received.is_defined())
          bytes_recv += *inbound.bytes_received;
        if (inbound.frames_per_second.is_defined())
          fps_recv = *inbound.frames_per_second;
        if (inbound.frame_width.is_defined())
          width_recv = *inbound.frame_width;
        if (inbound.frame_height.is_defined())
          height_recv = *inbound.frame_height;
        if (inbound.frames_decoded.is_defined())
          frames_decoded = *inbound.frames_decoded;
        if (inbound.frames_dropped.is_defined())
          frames_dropped = *inbound.frames_dropped;
        if (inbound.jitter.is_defined())
          jitter_ms = *inbound.jitter * 1000.0;
        if (inbound.packets_lost.is_defined())
          packets_lost = *inbound.packets_lost;
        if (inbound.pli_count.is_defined())
          pli_count = *inbound.pli_count;
        if (inbound.fir_count.is_defined())
          fir_count = *inbound.fir_count;
        if (inbound.nack_count.is_defined())
          nack_count = *inbound.nack_count;
      }

      if (s.type() == webrtc::RTCRemoteInboundRtpStreamStats::kType) {
        const auto& remote_in = s.cast_to<webrtc::RTCRemoteInboundRtpStreamStats>();
        if (remote_in.round_trip_time.is_defined())
          rtt_ms = *remote_in.round_trip_time * 1000.0;
      }
    }

    RTC_LOG(LS_INFO)
        << "[VideoStats]"
        << " Sent=" << bytes_sent << "B"
        << " Recv=" << bytes_recv << "B"
        << " | FPS(S/R)=" << fps_send << "/" << fps_recv
        << " | Size(S)=" << width_send << "x" << height_send
        << " R=" << width_recv << "x" << height_recv
        << " | Frames Enc/Dec/Drop=" << frames_encoded << "/" << frames_decoded << "/" << frames_dropped
        << " | EncodeTime=" << encode_time << "s"
        << " | AvgQP=" << avg_qp
        << " | TargetBitrate=" << target_bitrate << "bps"
        << " | Jitter=" << jitter_ms << "ms"
        << " | RTT=" << rtt_ms << "ms"
        << " | Lost=" << packets_lost
        << " | PLI/FIR/NACK=" << pli_count << "/" << fir_count << "/" << nack_count
        << " | Quality=" << quality_reason;
  }
 public:
  explicit PeriodicStatsCallback(rtc::Thread* thread)
      : thread_(thread) {}

  void OnStatsDelivered(const rtc::scoped_refptr<const webrtc::RTCStatsReport>& report) override {
    thread_->PostTask([report]() {
      StatsReport(report);
    });
  }

 private:
  rtc::Thread* thread_;
};

}  // namespace


namespace {
// Names used for a IceCandidate JSON object.
const char kCandidateSdpMidName[] = "sdpMid";
const char kCandidateSdpMlineIndexName[] = "sdpMLineIndex";
const char kCandidateSdpName[] = "candidate";

// Names used for a SessionDescription JSON object.
const char kSessionDescriptionTypeName[] = "type";
const char kSessionDescriptionSdpName[] = "sdp";

class DummySetSessionDescriptionObserver
    : public webrtc::SetSessionDescriptionObserver {
 public:
  static rtc::scoped_refptr<DummySetSessionDescriptionObserver> Create() {
    return rtc::make_ref_counted<DummySetSessionDescriptionObserver>();
  }
  virtual void OnSuccess() { RTC_LOG(LS_INFO) << __FUNCTION__; }
  virtual void OnFailure(webrtc::RTCError error) {
    RTC_LOG(LS_INFO) << __FUNCTION__ << " " << ToString(error.type()) << ": "
                     << error.message();
  }
};

}  // namespace

Conductor::Conductor(PeerConnectionClient* client, MainWindow* main_wnd)
    : peer_id_(-1), is_sender_(false), loopback_(false), client_(client), main_wnd_(main_wnd) {
  client_->RegisterObserver(this);
  main_wnd->RegisterObserver(this);
}

Conductor::~Conductor() {
  RTC_DCHECK(!peer_connection_);
  StopPeriodicStats();
}

bool Conductor::connection_active() const {
  return peer_connection_ != nullptr;
}

void Conductor::Close() {
  client_->SignOut();
  DeletePeerConnection();
}

void Conductor::StartPeriodicStats() {
  if (stats_task_.Running()) {
    RTC_LOG(LS_INFO) << "Periodic stats already running";
    return;
  }

  if (!stats_thread_) {
    stats_thread_ = rtc::Thread::Create();
    stats_thread_->SetName("StatsThread", nullptr);
    stats_thread_->Start();
  }

  auto weak_this = weak_factory_.GetWeakPtr();

  stats_task_ = webrtc::RepeatingTaskHandle::DelayedStart(
      stats_thread_.get(),                          
      webrtc::TimeDelta::Seconds(2),
      [weak_this]() -> webrtc::TimeDelta {
        if (!weak_this) {
          return webrtc::TimeDelta::Zero();
        }
        weak_this->PrintStats();
        return webrtc::TimeDelta::Seconds(2);
      });

  RTC_LOG(LS_INFO) << "Started periodic stats task on StatsThread.";
}

void Conductor::StopPeriodicStats() {
  if (!stats_task_.Running())
    return;

  auto weak_this = weak_factory_.GetWeakPtr();
  stats_thread_->PostTask([weak_this]() {
    if (weak_this && weak_this->stats_task_.Running()) {
      weak_this->stats_task_.Stop();
      RTC_LOG(LS_INFO) << "Stopped periodic stats task safely.";
    }
  });
}

void Conductor::PrintStats() {
  if (!peer_connection_) return;
  auto stats_callback = rtc::make_ref_counted<PeriodicStatsCallback>(rtc::Thread::Current());
  peer_connection_->GetStats(stats_callback.get());
}


bool Conductor::InitializePeerConnection() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  RTC_DCHECK(!peer_connection_factory_);
  RTC_DCHECK(!peer_connection_);

  if (!signaling_thread_.get()) {
    signaling_thread_ = rtc::Thread::CreateWithSocketServer();
    signaling_thread_->SetName("signaling_thread", nullptr);
    signaling_thread_->Start();
  }
  peer_connection_factory_ = webrtc::CreatePeerConnectionFactory(
      nullptr /* network_thread */, nullptr /* worker_thread */,
      signaling_thread_.get(), nullptr /* default_adm */,
      webrtc::CreateBuiltinAudioEncoderFactory(),
      webrtc::CreateBuiltinAudioDecoderFactory(),
      std::make_unique<webrtc::VideoEncoderFactoryTemplate<
          //webrtc::LibvpxVp8EncoderTemplateAdapter
          //webrtc::LibvpxVp9EncoderTemplateAdapter
          webrtc::OpenH264EncoderTemplateAdapter
          //webrtc::LibaomAv1EncoderTemplateAdapter
          >>(),
      std::make_unique<webrtc::VideoDecoderFactoryTemplate<
          //webrtc::LibvpxVp8DecoderTemplateAdapter
          //webrtc::LibvpxVp9DecoderTemplateAdapter
          webrtc::OpenH264DecoderTemplateAdapter
          //webrtc::Dav1dDecoderTemplateAdapter
          >>(),
      nullptr /* audio_mixer */, nullptr /* audio_processing */);

  if (!peer_connection_factory_) {
    main_wnd_->MessageBox("Error", "Failed to initialize PeerConnectionFactory",
                          true);
    DeletePeerConnection();
    return false;
  }

  if (!CreatePeerConnection()) {
    main_wnd_->MessageBox("Error", "CreatePeerConnection failed", true);
    DeletePeerConnection();
  }

  AddTracks();

  StartPeriodicStats();

  return peer_connection_ != nullptr;
}

bool Conductor::ReinitializePeerConnectionForLoopback() {
  loopback_ = true;
  std::vector<rtc::scoped_refptr<webrtc::RtpSenderInterface>> senders =
      peer_connection_->GetSenders();
  peer_connection_ = nullptr;
  // Loopback is only possible if encryption is disabled.
  webrtc::PeerConnectionFactoryInterface::Options options;
  options.disable_encryption = true;
  peer_connection_factory_->SetOptions(options);
  if (CreatePeerConnection()) {
    for (const auto& sender : senders) {
      peer_connection_->AddTrack(sender->track(), sender->stream_ids());
    }
    peer_connection_->CreateOffer(
        this, webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
  }
  options.disable_encryption = false;
  peer_connection_factory_->SetOptions(options);
  return peer_connection_ != nullptr;
}

bool Conductor::CreatePeerConnection() {
  RTC_DCHECK(peer_connection_factory_);
  RTC_DCHECK(!peer_connection_);

  webrtc::PeerConnectionInterface::RTCConfiguration config;
  config.sdp_semantics = webrtc::SdpSemantics::kUnifiedPlan;
  webrtc::PeerConnectionInterface::IceServer server;
  server.uri = GetPeerConnectionString();
  config.servers.push_back(server);

  webrtc::PeerConnectionDependencies pc_dependencies(this);
  auto error_or_peer_connection =
      peer_connection_factory_->CreatePeerConnectionOrError(
          config, std::move(pc_dependencies));
  if (error_or_peer_connection.ok()) {
    peer_connection_ = std::move(error_or_peer_connection.value());
  }
  return peer_connection_ != nullptr;
}

void Conductor::DeletePeerConnection() {
  StopPeriodicStats();
  main_wnd_->StopLocalRenderer();
  main_wnd_->StopRemoteRenderer();
  peer_connection_ = nullptr;
  peer_connection_factory_ = nullptr;
  peer_id_ = -1;
  loopback_ = false;
}

void Conductor::EnsureStreamingUI() {
  RTC_DCHECK(peer_connection_);
  if (main_wnd_->IsWindow()) {
    if (main_wnd_->current_ui() != MainWindow::STREAMING)
      main_wnd_->SwitchToStreamingUI();
  }
}

//
// PeerConnectionObserver implementation.
//

void Conductor::OnAddTrack(
    rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver,
    const std::vector<rtc::scoped_refptr<webrtc::MediaStreamInterface>>&
        streams) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " " << receiver->id();
  main_wnd_->QueueUIThreadCallback(NEW_TRACK_ADDED,
                                   receiver->track().release());
}

void Conductor::OnRemoveTrack(
    rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " " << receiver->id();
  main_wnd_->QueueUIThreadCallback(TRACK_REMOVED, receiver->track().release());
}

void Conductor::OnIceCandidate(const webrtc::IceCandidateInterface* candidate) {
  RTC_LOG(LS_INFO) << __FUNCTION__ << " " << candidate->sdp_mline_index();
  // For loopback test. To save some connecting delay.
  if (loopback_) {
    if (!peer_connection_->AddIceCandidate(candidate)) {
      RTC_LOG(LS_WARNING) << "Failed to apply the received candidate";
    }
    return;
  }

  Json::Value jmessage;
  jmessage[kCandidateSdpMidName] = candidate->sdp_mid();
  jmessage[kCandidateSdpMlineIndexName] = candidate->sdp_mline_index();
  std::string sdp;
  if (!candidate->ToString(&sdp)) {
    RTC_LOG(LS_ERROR) << "Failed to serialize candidate";
    return;
  }
  jmessage[kCandidateSdpName] = sdp;

  Json::StreamWriterBuilder factory;
  SendMessage(Json::writeString(factory, jmessage));
}

//
// PeerConnectionClientObserver implementation.
//

void Conductor::OnSignedIn() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  main_wnd_->SwitchToPeerList(client_->peers());
}

void Conductor::OnDisconnected() {
  RTC_LOG(LS_INFO) << __FUNCTION__;

  DeletePeerConnection();

  if (main_wnd_->IsWindow())
    main_wnd_->SwitchToConnectUI();
}

void Conductor::OnPeerConnected(int id, const std::string& name) {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  // Refresh the list if we're showing it.
  if (main_wnd_->current_ui() == MainWindow::LIST_PEERS)
    main_wnd_->SwitchToPeerList(client_->peers());
}

void Conductor::OnPeerDisconnected(int id) {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  if (id == peer_id_) {
    RTC_LOG(LS_INFO) << "Our peer disconnected";
    main_wnd_->QueueUIThreadCallback(PEER_CONNECTION_CLOSED, NULL);
  } else {
    // Refresh the list if we're showing it.
    if (main_wnd_->current_ui() == MainWindow::LIST_PEERS)
      main_wnd_->SwitchToPeerList(client_->peers());
  }
}

void Conductor::OnMessageFromPeer(int peer_id, const std::string& message) {
  RTC_DCHECK(peer_id_ == peer_id || peer_id_ == -1);
  RTC_DCHECK(!message.empty());

  if (!peer_connection_.get()) {
    RTC_DCHECK(peer_id_ == -1);
    peer_id_ = peer_id;
    is_sender_ = false;
    if (!InitializePeerConnection()) {
      RTC_LOG(LS_ERROR) << "Failed to initialize our PeerConnection instance";
      client_->SignOut();
      return;
    }
  } else if (peer_id != peer_id_) {
    RTC_DCHECK(peer_id_ != -1);
    RTC_LOG(LS_WARNING)
        << "Received a message from unknown peer while already in a "
           "conversation with a different peer.";
    return;
  }

  Json::CharReaderBuilder factory;
  std::unique_ptr<Json::CharReader> reader =
      absl::WrapUnique(factory.newCharReader());
  Json::Value jmessage;
  if (!reader->parse(message.data(), message.data() + message.length(),
                     &jmessage, nullptr)) {
    RTC_LOG(LS_WARNING) << "Received unknown message. " << message;
    return;
  }
  std::string type_str;
  std::string json_object;

  rtc::GetStringFromJsonObject(jmessage, kSessionDescriptionTypeName,
                               &type_str);
  if (!type_str.empty()) {
    if (type_str == "offer-loopback") {
      // This is a loopback call.
      // Recreate the peerconnection with DTLS disabled.
      if (!ReinitializePeerConnectionForLoopback()) {
        RTC_LOG(LS_ERROR) << "Failed to initialize our PeerConnection instance";
        DeletePeerConnection();
        client_->SignOut();
      }
      return;
    }
    absl::optional<webrtc::SdpType> type_maybe =
        webrtc::SdpTypeFromString(type_str);
    if (!type_maybe) {
      RTC_LOG(LS_ERROR) << "Unknown SDP type: " << type_str;
      return;
    }
    webrtc::SdpType type = *type_maybe;
    std::string sdp;
    if (!rtc::GetStringFromJsonObject(jmessage, kSessionDescriptionSdpName,
                                      &sdp)) {
      RTC_LOG(LS_WARNING)
          << "Can't parse received session description message.";
      return;
    }
    webrtc::SdpParseError error;
    std::unique_ptr<webrtc::SessionDescriptionInterface> session_description =
        webrtc::CreateSessionDescription(type, sdp, &error);
    if (!session_description) {
      RTC_LOG(LS_WARNING)
          << "Can't parse received session description message. "
             "SdpParseError was: "
          << error.description;
      return;
    }
    RTC_LOG(LS_INFO) << " Received session description :" << message;
    peer_connection_->SetRemoteDescription(
        DummySetSessionDescriptionObserver::Create().get(),
        session_description.release());
    if (type == webrtc::SdpType::kOffer) {
      peer_connection_->CreateAnswer(
          this, webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
    }
  } else {
    std::string sdp_mid;
    int sdp_mlineindex = 0;
    std::string sdp;
    if (!rtc::GetStringFromJsonObject(jmessage, kCandidateSdpMidName,
                                      &sdp_mid) ||
        !rtc::GetIntFromJsonObject(jmessage, kCandidateSdpMlineIndexName,
                                   &sdp_mlineindex) ||
        !rtc::GetStringFromJsonObject(jmessage, kCandidateSdpName, &sdp)) {
      RTC_LOG(LS_WARNING) << "Can't parse received message.";
      return;
    }
    webrtc::SdpParseError error;
    std::unique_ptr<webrtc::IceCandidateInterface> candidate(
        webrtc::CreateIceCandidate(sdp_mid, sdp_mlineindex, sdp, &error));
    if (!candidate.get()) {
      RTC_LOG(LS_WARNING) << "Can't parse received candidate message. "
                             "SdpParseError was: "
                          << error.description;
      return;
    }
    if (!peer_connection_->AddIceCandidate(candidate.get())) {
      RTC_LOG(LS_WARNING) << "Failed to apply the received candidate";
      return;
    }
    RTC_LOG(LS_INFO) << " Received candidate :" << message;
  }
}

void Conductor::OnMessageSent(int err) {
  // Process the next pending message if any.
  main_wnd_->QueueUIThreadCallback(SEND_MESSAGE_TO_PEER, NULL);
}

void Conductor::OnServerConnectionFailure() {
  main_wnd_->MessageBox("Error", ("Failed to connect to " + server_).c_str(),
                        true);
}

//
// MainWndCallback implementation.
//

void Conductor::StartLogin(const std::string& server, int port) {
  if (client_->is_connected())
    return;
  server_ = server;
  client_->Connect(server, port, GetPeerName());
}

void Conductor::DisconnectFromServer() {
  if (client_->is_connected())
    client_->SignOut();
}

void Conductor::ConnectToPeer(int peer_id) {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  RTC_DCHECK(peer_id_ == -1);
  RTC_DCHECK(peer_id != -1);

  if (peer_connection_.get()) {
    main_wnd_->MessageBox(
        "Error", "We only support connecting to one peer at a time", true);
    return;
  }

  is_sender_ = true;
  if (InitializePeerConnection()) {
    peer_id_ = peer_id;
    peer_connection_->CreateOffer(
        this, webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
  } else {
    main_wnd_->MessageBox("Error", "Failed to initialize PeerConnection", true);
  }
}

void Conductor::AddTracks() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  if (!peer_connection_->GetSenders().empty()) {
    return;  // Already added tracks.
  }
  
  if (!is_sender_) {
    main_wnd_->SwitchToStreamingUI();
    return;  // Do not add tracks if we are not the sender.
  }

  rtc::scoped_refptr<ImageSequenceVideoTrackSource> video_source = ImageSequenceVideoTrackSource::Create(
      ImageSequenceVideoTrackSource::Options{
          .pattern = "/data/zh/videos_dir/AIC20-c001/%06d.png",
          .start_index = 1,
          .end_index = 1800,
          .fps = 30.0,
          .fixed_width = 320,
          .fixed_height = 240,
          .queue_capacity = 16,
          .threads = 2,
          .loop_missing = false,
      });

  if (video_source) {
    rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track_(
        peer_connection_factory_->CreateVideoTrack(video_source, kVideoLabel));
    main_wnd_->StartLocalRenderer(video_track_.get());

    auto result_or_error = peer_connection_->AddTrack(video_track_, {kStreamId});
    if (!result_or_error.ok()) {
      RTC_LOG(LS_ERROR) << "Failed to add video track to PeerConnection: "
                        << result_or_error.error().message();
    }
  } else {
    RTC_LOG(LS_ERROR) << "Add VideoTrackSource failed";
  }
  main_wnd_->SwitchToStreamingUI();
}

void Conductor::DisconnectFromCurrentPeer() {
  RTC_LOG(LS_INFO) << __FUNCTION__;
  if (peer_connection_.get()) {
    client_->SendHangUp(peer_id_);
    DeletePeerConnection();
  }

  if (main_wnd_->IsWindow())
    main_wnd_->SwitchToPeerList(client_->peers());
}

void Conductor::UIThreadCallback(int msg_id, void* data) {
  switch (msg_id) {
    case PEER_CONNECTION_CLOSED:
      RTC_LOG(LS_INFO) << "PEER_CONNECTION_CLOSED";
      DeletePeerConnection();

      if (main_wnd_->IsWindow()) {
        if (client_->is_connected()) {
          main_wnd_->SwitchToPeerList(client_->peers());
        } else {
          main_wnd_->SwitchToConnectUI();
        }
      } else {
        DisconnectFromServer();
      }
      break;

    case SEND_MESSAGE_TO_PEER: {
      RTC_LOG(LS_INFO) << "SEND_MESSAGE_TO_PEER";
      std::string* msg = reinterpret_cast<std::string*>(data);
      if (msg) {
        // For convenience, we always run the message through the queue.
        // This way we can be sure that messages are sent to the server
        // in the same order they were signaled without much hassle.
        pending_messages_.push_back(msg);
      }

      if (!pending_messages_.empty() && !client_->IsSendingMessage()) {
        msg = pending_messages_.front();
        pending_messages_.pop_front();

        if (!client_->SendToPeer(peer_id_, *msg) && peer_id_ != -1) {
          RTC_LOG(LS_ERROR) << "SendToPeer failed";
          DisconnectFromServer();
        }
        delete msg;
      }

      if (!peer_connection_.get())
        peer_id_ = -1;

      break;
    }

    case NEW_TRACK_ADDED: {
      auto* track = reinterpret_cast<webrtc::MediaStreamTrackInterface*>(data);
      if (track->kind() == webrtc::MediaStreamTrackInterface::kVideoKind) {
        auto* video_track = static_cast<webrtc::VideoTrackInterface*>(track);
        main_wnd_->StartRemoteRenderer(video_track);
      }
      track->Release();
      break;
    }

    case TRACK_REMOVED: {
      // Remote peer stopped sending a track.
      auto* track = reinterpret_cast<webrtc::MediaStreamTrackInterface*>(data);
      track->Release();
      break;
    }

    default:
      RTC_DCHECK_NOTREACHED();
      break;
  }
}

void Conductor::OnSuccess(webrtc::SessionDescriptionInterface* desc) {
  peer_connection_->SetLocalDescription(
      DummySetSessionDescriptionObserver::Create().get(), desc);

  std::string sdp;
  desc->ToString(&sdp);

  // For loopback test. To save some connecting delay.
  if (loopback_) {
    // Replace message type from "offer" to "answer"
    std::unique_ptr<webrtc::SessionDescriptionInterface> session_description =
        webrtc::CreateSessionDescription(webrtc::SdpType::kAnswer, sdp);
    peer_connection_->SetRemoteDescription(
        DummySetSessionDescriptionObserver::Create().get(),
        session_description.release());
    return;
  }

  Json::Value jmessage;
  jmessage[kSessionDescriptionTypeName] =
      webrtc::SdpTypeToString(desc->GetType());
  jmessage[kSessionDescriptionSdpName] = sdp;

  Json::StreamWriterBuilder factory;
  SendMessage(Json::writeString(factory, jmessage));
}

void Conductor::OnFailure(webrtc::RTCError error) {
  RTC_LOG(LS_ERROR) << ToString(error.type()) << ": " << error.message();
}

void Conductor::SendMessage(const std::string& json_object) {
  std::string* msg = new std::string(json_object);
  main_wnd_->QueueUIThreadCallback(SEND_MESSAGE_TO_PEER, msg);
}
