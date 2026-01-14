/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "modules/video_coding/svc/dynamic_structure_controller.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/algorithm/container.h"
#include "api/transport/rtp/dependency_descriptor.h"
#include "api/video/video_frame.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "system_wrappers/include/field_trial.h"
#include "absl/strings/numbers.h"
#include "rtc_base/trace_event.h"



namespace webrtc {

void DynamicStructureController::SetMode(std::string mode) {
  mode_ = mode;
}

void DynamicStructureController::OnNextFrame(const VideoFrame& frame, bool restart) {
  webrtc::MutexLock lock(&mutex_);

  if (restart){
    recent_frames_list_.clear();
    last_decoded_frame_id_.reset();
    for (int i = 0; i < 8; i++) slots_info_[i].frame_id.reset();
  }
  current_frame_id_ = frame.id();
  frame_count_++;
}

void DynamicStructureController::OnReceivedAckFrameDecoded(uint64_t frame_id) {
  webrtc::MutexLock lock(&mutex_);
  last_decoded_frame_id_ = frame_id;
}

DynamicStructureController::StreamLayersConfig
DynamicStructureController::StreamConfig() const {
  StreamLayersConfig result;
  result.num_spatial_layers = 1;
  result.num_temporal_layers = 1;
  result.uses_reference_scaling = false;
  return result;
}

FrameDependencyStructure
DynamicStructureController::DependencyStructure() const {
  FrameDependencyStructure s;
  s.num_decode_targets = 1;
  s.num_chains = 1;
  s.decode_target_protected_by_chain = {0};

  FrameDependencyTemplate key;
  key.spatial_id = 0;
  key.temporal_id = 0;
  key.decode_target_indications = {DecodeTargetIndication::kSwitch};
  key.chain_diffs = {0};  
  s.templates.push_back(key);

  FrameDependencyTemplate delta;
  delta.spatial_id = 0;
  delta.temporal_id = 0;
  delta.decode_target_indications = {DecodeTargetIndication::kSwitch};
  delta.chain_diffs = {1};
  delta.frame_diffs = {1};
  s.templates.push_back(delta);

  return s;
}

// video-expr: get fixed step from field trial Exp-FixedReferenceStep
int GetFixedReferenceStep() {
    std::string s = webrtc::field_trial::FindFullName("Exp-FixedReferenceStep");
    int val = 0;
    if (s.empty() || s == "Disabled") return 1;
    if (!absl::SimpleAtoi(s, &val)) return 1;
    return val; 
}

std::vector<ScalableVideoController::LayerFrameConfig>
DynamicStructureController::NextFrameConfig_KstepAckedRoot(bool restart) {
  LayerFrameConfig cfg;
  cfg.S(0).T(0);
  int position = frame_count_ % 7;
  if (restart || frame_count_ == 1) {
    cfg.Id(0).Keyframe().Update(position);
    current_frame_ref_frame_id_ = 0;
    current_frame_importance_ = 2;
    UpdateSlot(position, current_frame_id_);
    root_frame_slot_id_ = position;
    root_frame_id_ = current_frame_id_;
    TRACE_EVENT_INSTANT1("video-expr",
                         "DynamicStructureController::NextFrameConfig_AckedOnly",
                         "json", absl::StrFormat(
                           R"({"current_frame_id":%u, "ref_frame_id":null, "ref_slot_id":null})",
                           current_frame_id_
                         ));
    TRACE_EVENT_INSTANT1("video-expr",
                         "DynamicStructureController::UpdateSlot",
                         "json", absl::StrFormat(
                           R"({"slot_id":%u, "frame_id":%u})",
                           root_frame_slot_id_,
                           root_frame_id_
                         ));
  } else {
    cfg.Id(1);
    FrameInfo* frame_info = nullptr; 
    if (last_decoded_frame_id_.has_value()){
      frame_info = FindFrameInfoByFrameId(last_decoded_frame_id_.value());
      if (frame_info){
        RTC_DCHECK(frame_info->slot_id.has_value());
        cfg.Reference(frame_info->slot_id.value());
      }
    }
    if (!frame_info) {   // indicate no recent decoded frame
      RTC_CHECK(!recent_frames_list_.empty());
      frame_info = recent_frames_list_.front().get();
      cfg.Reference(frame_info->slot_id.value());
      RTC_LOG(LS_WARNING) << "No recent acked decoded frame available, frame" << current_frame_id_ << " referencing the oldest referencable frame" << frame_info->frame_id;
    }
    current_frame_ref_frame_id_ = frame_info->frame_id;
    current_frame_importance_ = 0;
    TRACE_EVENT_INSTANT1("video-expr",
                         "DynamicStructureController::NextFrameConfig_AckedOnly",
                         "json", absl::StrFormat(
                           R"({"current_frame_id":%u, "ref_frame_id":%u, "ref_slot_id":%u})",
                           current_frame_id_,
                           frame_info->frame_id,
                           frame_info->slot_id.value()
                         ));
    cfg.Update(position);
    UpdateSlot(position, current_frame_id_);
    TRACE_EVENT_INSTANT1("video-expr",
                         "DynamicStructureController::UpdateSlot",
                         "json", absl::StrFormat(
                           R"({"slot_id":%u, "frame_id":%u})",
                           position,
                           current_frame_id_
                         ));
  }
}

std::vector<ScalableVideoController::LayerFrameConfig>
DynamicStructureController::NextFrameConfig_Kstep(bool restart) {
  static int fixed_step = GetFixedReferenceStep();
  LayerFrameConfig cfg;
  cfg.S(0).T(0);
  if (restart || frame_count_ == 1) {
    cfg.Id(0).Keyframe().Update(0);
    current_frame_ref_frame_id_ = 0;
    current_frame_importance_ = 2;
    UpdateSlot(0, current_frame_id_);
  } else {
    cfg.Id(1);  
    RTC_DCHECK(!recent_frames_list_.empty());
    FrameInfo* frame_info = recent_frames_list_.front().get();
    cfg.Reference(frame_info->slot_id.value());
    current_frame_ref_frame_id_ = frame_info->frame_id;

    if (frame_count_ % fixed_step == 0){
      cfg.Update(0);
      UpdateSlot(0, current_frame_id_);
      current_frame_importance_ = 1;
    } else {
      current_frame_importance_ = 0;
    }
  }
  return {cfg};
}

std::vector<ScalableVideoController::LayerFrameConfig>
DynamicStructureController::NextFrameConfig_AckedOnly(bool restart) {
  LayerFrameConfig cfg;
  cfg.S(0).T(0);
  int position = frame_count_ % 8;
  if (restart || frame_count_ == 1) {
    cfg.Id(0).Keyframe().Update(position);
    current_frame_ref_frame_id_ = 0;
    current_frame_importance_ = 2;
    UpdateSlot(position, current_frame_id_);
    TRACE_EVENT_INSTANT1("video-expr",
                         "DynamicStructureController::NextFrameConfig_AckedOnly",
                         "json", absl::StrFormat(
                           R"({"current_frame_id":%u, "ref_frame_id":null, "ref_slot_id":null})",
                           current_frame_id_
                         ));
    TRACE_EVENT_INSTANT1("video-expr",
                         "DynamicStructureController::UpdateSlot",
                         "json", absl::StrFormat(
                           R"({"slot_id":%u, "frame_id":%u})",
                           current_frame_id_ % 8,
                           current_frame_id_
                         ));
  } else {
    cfg.Id(1);
    FrameInfo* frame_info = nullptr; 
    if (last_decoded_frame_id_.has_value()){
      frame_info = FindFrameInfoByFrameId(last_decoded_frame_id_.value());
      if (frame_info){
        RTC_DCHECK(frame_info->slot_id.has_value());
        cfg.Reference(frame_info->slot_id.value());
      }
    }
    if (!frame_info) {   // indicate no recent decoded frame
      RTC_CHECK(!recent_frames_list_.empty());
      frame_info = recent_frames_list_.front().get();
      cfg.Reference(frame_info->slot_id.value());
      RTC_LOG(LS_WARNING) << "No recent acked decoded frame available, frame" << current_frame_id_ << " referencing the oldest referencable frame" << frame_info->frame_id;
    }
    current_frame_ref_frame_id_ = frame_info->frame_id;
    current_frame_importance_ = 0;
    TRACE_EVENT_INSTANT1("video-expr",
                         "DynamicStructureController::NextFrameConfig_AckedOnly",
                         "json", absl::StrFormat(
                           R"({"current_frame_id":%u, "ref_frame_id":%u, "ref_slot_id":%u})",
                           current_frame_id_,
                           frame_info->frame_id,
                           frame_info->slot_id.value()
                         ));
    cfg.Update(position);
    UpdateSlot(position, current_frame_id_);
    TRACE_EVENT_INSTANT1("video-expr",
                         "DynamicStructureController::UpdateSlot",
                         "json", absl::StrFormat(
                           R"({"slot_id":%u, "frame_id":%u})",
                           position,
                           current_frame_id_
                         ));
  }

  return {cfg};
}


std::vector<ScalableVideoController::LayerFrameConfig>
DynamicStructureController::NextFrameConfig(bool restart) {
  {
    webrtc::MutexLock lock(&mutex_);
    if (mode_ == "Acked-only"){
      return NextFrameConfig_AckedOnly(restart);
    } else if (mode_ == "K-step"){
      return NextFrameConfig_Kstep(restart);
    } else {
      RTC_DCHECK_NOTREACHED();
      return {};
    } 
  }
}

GenericFrameInfo DynamicStructureController::OnEncodeDone(
    const LayerFrameConfig& config) {
  GenericFrameInfo frame_info;
  frame_info.encoder_buffers = config.Buffers();
  if (config.IsKeyframe()) {
    for (auto& buffer : frame_info.encoder_buffers) {
      buffer.referenced = false;
    }
  }
  frame_info.decode_target_indications = {DecodeTargetIndication::kSwitch};
  frame_info.part_of_chain = {true};
  return frame_info;
}

}  // namespace webrtc
