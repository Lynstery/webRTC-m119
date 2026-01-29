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

#include <cstdint>
#include <cstdlib>
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

// video-expr: get fixed step from field trial Exp-FixedReferenceStep
int GetFixedReferenceStep() {
    std::string s = webrtc::field_trial::FindFullName("Exp-FixedReferenceStep");
    int val = 0;
    if (s.empty() || s == "Disabled") return 1;
    if (!absl::SimpleAtoi(s, &val)) return 1;
    return val; 
}

// video-expr: get fixed step from field trial Exp-FixedFirebreakRefPrevProb

bool GetFixedFirebreakRefPrevProbEnabled() {
    std::string s = webrtc::field_trial::FindFullName("Exp-FixedFirebreakRefPrevProb");
    if (s.empty() || s == "Disabled") return false;
    return true; 
}

int GetFixedFirebreakRefPrevProb() {
    std::string s = webrtc::field_trial::FindFullName("Exp-FixedFirebreakRefPrevProb");
    int val = 0;
    if (s.empty() || s == "Disabled") return 0;
    if (!absl::SimpleAtoi(s, &val)) return 0;
    return val; 
}

bool GetMaxChainLengthEnabled() {
    std::string s = webrtc::field_trial::FindFullName("Exp-MaxChainLength");
    int val = 0;
    if (s.empty() || s == "Disabled") return false;
    if (!absl::SimpleAtoi(s, &val)) return false;
    return true;
}

int GetMaxChainLength() {
    std::string s = webrtc::field_trial::FindFullName("Exp-MaxChainLength");
    int val = 0;
    if (s.empty() || s == "Disabled") return 6;
    if (!absl::SimpleAtoi(s, &val)) return 6;
    return val; 
}

// video-expr: get fixed step from field trial Exp-FixedFirebreakRefPrevStep
int GetFixedFirebreakRefPrevStep() {
    std::string s = webrtc::field_trial::FindFullName("Exp-FixedFirebreakRefPrevStep");
    int val = 0;
    if (s.empty() || s == "Disabled") return 0;
    if (!absl::SimpleAtoi(s, &val)) return 0;
    return val; 
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

DynamicStructureController::StreamLayersConfig
DynamicStructureController::StreamConfig() const {
  StreamLayersConfig result;
  result.num_spatial_layers = 1;
  result.num_temporal_layers = 1;
  result.uses_reference_scaling = false;
  return result;
}

GenericFrameInfo DynamicStructureController::OnEncodeDone(
    const LayerFrameConfig& config) {
  webrtc::MutexLock lock(&mutex_);
  status_updated_ = false;
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

void DynamicStructureController::SetMode(std::string mode) {
  mode_ = mode;
}

void DynamicStructureController::OnNextFrame(const VideoFrame& frame, bool restart) {
  webrtc::MutexLock lock(&mutex_);
  if (restart){
    Reset();
    last_key_frame_id_ = frame.id();
  }
  current_frame_id_ = frame.id();
  frame_count_++;
}

// Should change the functions implementation below to achieve RTT-aware slot management and RTT, loss-aware reference management.

void DynamicStructureController::UpdateFireBreakStatus(int g) {
  if (status_updated_) return;
  TRACE_EVENT_INSTANT1("video-expr", "FireBreak:UpdateStatus", "json", absl::StrFormat("{\"g\": %d, \"current_frame_id\": %llu}", g, current_frame_id_));
  double lambda_s = 0.05, alpha = 0.4, beta = 0.5;
  g_avg_ = (1 - lambda_s) * g_avg_ + lambda_s * g;
  if (g == 1){
    g_aimd_ = std::min(g_aimd_ + alpha, 1.0);
  } else {
    g_aimd_ = std::max(g_aimd_ * beta, 0.1);      
  }
  status_updated_ = true;
}

void DynamicStructureController::OnReceivedAckFrameDecoded(uint64_t frame_id) {
  webrtc::MutexLock lock(&mutex_);
  
  TRACE_EVENT_INSTANT1("video-expr", "FireBreak:Acked", "json", absl::StrFormat("{\"frame_id\": %llu, \"current_frame_id\": %llu}", frame_id, current_frame_id_));

  if (last_decoded_frame_id_.has_value() && last_decoded_frame_id_.value() + 1 < frame_id) {
    TRACE_EVENT_INSTANT1("video-expr", "FireBreak:DetectAckGap", "json", absl::StrFormat("{\"from_frame_id\": %llu, \"to_frame_id\": %llu}", last_decoded_frame_id_.value(), frame_id));
    UpdateFireBreakStatus(0);
  }

  last_decoded_frame_id_ = frame_id;
  for (auto it = reference_frames_list_.begin(); it != reference_frames_list_.end(); ++it) {
    if ((*it)->frame_id == frame_id) {
      (*it)->acked = true;
    }
  }
  // update feedback delay in frames
  uint64_t current_feedback_delay_frames_ = current_frame_id_ + 1 - frame_id;
  if (!feedback_delay_frames_.has_value()){
    feedback_delay_frames_ = current_feedback_delay_frames_;
  } else {
    feedback_delay_frames_ = static_cast<uint64_t>(
      0.5 * feedback_delay_frames_.value() + 0.5 * current_feedback_delay_frames_);
  }
}

void DynamicStructureController::OnRttUpdate(int64_t rtt_ms) {
  webrtc::MutexLock lock(&mutex_);
  // delayed rtt info from RTCP, we may not use it
  rtt_ms_ = rtt_ms;
  rtt_ms_ema_ = static_cast<uint64_t>(0.9 * rtt_ms_ema_ + 0.1 * rtt_ms_); 
}

void DynamicStructureController::OnPacketLossRateUpdate(float packet_loss_rate) {
  // do nothing for now
  webrtc::MutexLock lock(&mutex_);
  TRACE_EVENT_INSTANT1("video-expr", "FireBreak:OnPacketLossRateUpdate", "json", absl::StrFormat("{\"current_frame_id\": %llu, \"loss_rate_mul_10000\": %llu}", current_frame_id_, static_cast<uint64_t>(packet_loss_rate * 10000)));
}

void DynamicStructureController::OnLossNotification(const VideoEncoder::LossNotification& loss_notification) {
  // do nothing for now
  webrtc::MutexLock lock(&mutex_);
  TRACE_EVENT_INSTANT1("video-expr", "FireBreak:OnLossNotification", "json", absl::StrFormat("{\"current_frame_id\": %llu, \"ts_of_last_received\": %llu, \"ts_of_last_decodable\": %llu }", current_frame_id_, loss_notification.timestamp_of_last_received, loss_notification.timestamp_of_last_decodable));
}


std::vector<ScalableVideoController::LayerFrameConfig>
DynamicStructureController::NextFrameConfig_FireBreak(bool restart) {
  static bool fixed_ref_prev_prob_enabled = GetFixedFirebreakRefPrevProbEnabled();
  static int fixed_ref_prev_prob = GetFixedFirebreakRefPrevProb();
  static bool max_chain_length_enabled = GetMaxChainLengthEnabled();
  static int max_chain_length = GetMaxChainLength(); 
  static int D = GetMaxChainLength();
  LayerFrameConfig cfg;
  cfg.S(0).T(0);
  if (restart || frame_count_ == 1) {
    int position = 0;
    cfg.Id(0).Keyframe().Update(position).Update(7);
    current_frame_ref_frame_id_ = 0;
    current_frame_importance_ = 2;
    UpdateSlot(position, current_frame_id_);
    conservative_slot_id_ = position;
  } else {
    cfg.Id(1);

    // ---- select reference frame ----

    FrameInfo* ref_frame_info = nullptr; 
    FrameInfo last_frame_info = FrameInfo(current_frame_id_ - 1, 7);
    
    if (last_decoded_frame_id_.has_value()) last_decoded_frame_ids_when_encode_.push_back(last_decoded_frame_id_.value());
    if (last_decoded_frame_ids_when_encode_.size() > 10) {
      last_decoded_frame_ids_when_encode_.pop_front();
    }
    if (last_decoded_frame_ids_when_encode_.size() >= 4){
      // check if the last 3 frames are the same
      auto it = last_decoded_frame_ids_when_encode_.end();
      uint64_t last1 = *(--it);
      uint64_t last2 = *(--it);
      uint64_t last3 = *(--it);
      uint64_t last4 = *(--it);
      if (last1 == last3){
        // consecutive acked frame id not increasing, likely due to high loss
        TRACE_EVENT_INSTANT1("video-expr", "FireBreak:DetectAckStall", "json", absl::StrFormat("{ \"current_frame_id\": %llu}", current_frame_id_));
        UpdateFireBreakStatus(0);
      }
    }

    if (!status_updated_){
      // no feedback yet, assume good status
      UpdateFireBreakStatus(1);
    }
    
    // hyperparameters
    if (fixed_ref_prev_prob_enabled) D = 100;
    if (max_chain_length_enabled) D = max_chain_length;  

    double p_max = 0.95;
    double p = p_max * g_aimd_ * g_avg_;
    
    if (fixed_ref_prev_prob_enabled) p = static_cast<double>(fixed_ref_prev_prob) / 100.0;

    bool ref_previous_frame = (std::rand() % 100) < p * 100; // firebreak_ref_prev_prob_;
    
    TRACE_EVENT_INSTANT1("video-expr", "FireBreak:RefPrevProb", "json", 
      absl::StrFormat("{\"current_frame_id\": %llu, \"last_decoded_frame_id\": %llu, \"g_aimd\": %.3f, \"g_avg\": %.3f, \"ref_prev_prob\": %.3f}", 
        current_frame_id_, last_decoded_frame_id_.value_or(0),  g_aimd_, g_avg_, p));
    
        

    if (ref_previous_frame && current_chain_length_ < D) {
      // try to reference last frame
      current_chain_length_++;
      ref_frame_info = &last_frame_info;
      cfg.Reference(7);
      TRACE_EVENT_INSTANT1("video-expr", "FireBreak:Reference", "json",
        absl::StrFormat("{\"current_frame_id\": %llu, \"ref_frame_id\": %llu, \"ref_slot_id\": %llu, \"type\": \"recent\"}",
          current_frame_id_, current_frame_id_ -1, 7));
      
    } else {
      current_chain_length_ = 0;
      // try to reference last acked decoded frame
      for (auto & frame : reference_frames_list_){
        if(frame->slot_id.has_value() && frame->acked && (ref_frame_info == nullptr || ref_frame_info->frame_id < frame->frame_id)){
          ref_frame_info = frame.get();
        }
      }
      if (ref_frame_info){
        cfg.Reference(ref_frame_info->slot_id.value());
        TRACE_EVENT_INSTANT1("video-expr", "FireBreak:Reference", "json",
          absl::StrFormat("{\"current_frame_id\": %llu, \"ref_frame_id\": %llu, \"ref_slot_id\": %llu, \"type\": \"acked\"}",
            current_frame_id_, ref_frame_info->frame_id, ref_frame_info->slot_id.value()));
      } 
      if (!ref_frame_info) {   // indicate no recent decoded frame
        ref_frame_info = &last_frame_info;
        cfg.Reference(7);
        RTC_LOG(LS_WARNING) << "When trying to reference Acked frame, no recent acked decoded frame available, frame" << current_frame_id_ << " referencing the recent frame" << current_frame_id_ -1;
        TRACE_EVENT_INSTANT1("video-expr", "FireBreak:Reference", "json",
          absl::StrFormat("{\"current_frame_id\": %llu, \"ref_frame_id\": %llu, \"ref_slot_id\": %llu, \"type\": \"acked-not-found\"}",
            current_frame_id_, current_frame_id_ -1, 7));
      }
    }
    current_frame_ref_frame_id_ = ref_frame_info->frame_id;
    current_frame_importance_ = 0;
    
    // ---- select refresh position ----
    
    /*
    if (reference_frames_list_.front()->slot_id.value() != conservative_slot_id_){
      // update the oldest frame if it is not the conservative slot
      int position = reference_frames_list_.front()->slot_id.value();
      cfg.Update(position);
      UpdateSlot(position, current_frame_id_);
      TRACE_EVENT_INSTANT1("video-expr", "FireBreak:Update", "json",
        absl::StrFormat("{\"current_frame_id\": %llu, \"refresh_slot_id\": %u, \"type\": \"quick-evict\"}",
          current_frame_id_, position));
    } else 
    */

    // always refresh the last slot 7, the last slot is used as the temporary slot for referencing previous frame, 
    // so kNumSlots = 8 - 1 = 7 slots are used for window refresh
    cfg.Update(7); 
    FrameInfo* lastest_acked_frame_info = GetLatestAckedFrameInfo();
    if (lastest_acked_frame_info && lastest_acked_frame_info->slot_id.value() != conservative_slot_id_){
      // update conservative slot when it is not the latest acked frame, the latest acked frame becomes the new conservative slot
      int position = conservative_slot_id_;
      conservative_slot_id_ = lastest_acked_frame_info->slot_id.value();
      cfg.Update(position);
      UpdateSlot(position, current_frame_id_);
      TRACE_EVENT_INSTANT1("video-expr", "FireBreak:Update", "json",
        absl::StrFormat("{\"current_frame_id\": %llu, \"refresh_slot_id\": %u, \"type\": \"change-conservative\"}",
          current_frame_id_, position));
    } else {
      // refresh window slots besides the conservative slot
      if (feedback_delay_frames_.has_value() && feedback_delay_frames_.value() > 0){
        // ceil((feedback_delay_frames_ - 1) / (kNumSlots -1)) + 1
        refresh_rate_  = (feedback_delay_frames_.value() - 1 + ((kNumSlots - 1) - 1)) / (kNumSlots - 1) + 1; 
      }
      if (frame_count_ % refresh_rate_ == 0){
        if (HasEmptySlot()){
          int position = GetFirstEmptySlotId();
          cfg.Update(position);
          UpdateSlot(position, current_frame_id_);
          TRACE_EVENT_INSTANT1("video-expr", "FireBreak:Update", "json",
            absl::StrFormat("{\"current_frame_id\": %llu, \"refresh_slot_id\": %u, \"type\": \"round-robin\"}",
              current_frame_id_, position));
        } else {
          FrameInfo* target_frame_info = nullptr;
          for (auto & frame : reference_frames_list_){
            if (frame->slot_id.value() != conservative_slot_id_ && (target_frame_info == nullptr || target_frame_info->frame_id > frame->frame_id)){
              target_frame_info = frame.get();
            }
          }
          int position = target_frame_info->slot_id.value();
          cfg.Update(position);
          UpdateSlot(position, current_frame_id_);
          TRACE_EVENT_INSTANT1("video-expr", "FireBreak:Update", "json",
            absl::StrFormat("{\"current_frame_id\": %llu, \"refresh_slot_id\": %u, \"type\": \"round-robin\"}",
              current_frame_id_, position));
        }
      }
    }
  }
  return {cfg};
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
    RTC_DCHECK(!reference_frames_list_.empty());
    FrameInfo* frame_info = reference_frames_list_.front().get();
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
DynamicStructureController::NextFrameConfig_IntraOnly(bool restart) {
  LayerFrameConfig cfg;
  cfg.S(0).T(0);
  if (restart || frame_count_ == 1) {
    cfg.Id(0).Keyframe().Update(0);
    current_frame_ref_frame_id_ = 0;
    current_frame_importance_ = 2;
    UpdateSlot(0, current_frame_id_);
  } else {
    cfg.Id(1);  
    RTC_DCHECK(!reference_frames_list_.empty());
    FrameInfo* frame_info = reference_frames_list_.front().get();
    cfg.Reference(frame_info->slot_id.value());
    current_frame_ref_frame_id_ = frame_info->frame_id;
    current_frame_importance_ = 0;
  }
  return {cfg};
}

std::vector<ScalableVideoController::LayerFrameConfig>
DynamicStructureController::NextFrameConfig_FireBreak_S(bool restart) {
  //static int fixed_ref_prev_prob = GetFixedFirebreakRefPrevProb();
  static int fixed_ref_prev_step = GetFixedFirebreakRefPrevStep();
  LayerFrameConfig cfg;
  cfg.S(0).T(0);
  int position = frame_count_ % kNumSlots;
  if (restart || frame_count_ == 1) {
    cfg.Id(0).Keyframe().Update(position);
    current_frame_ref_frame_id_ = 0;
    current_frame_importance_ = 2;
    UpdateSlot(position, current_frame_id_);
  } else {
    cfg.Id(1);
    FrameInfo* frame_info = nullptr; 
    
    //bool ref_previous_frame = (std::rand() % 100) < fixed_ref_prev_prob;
    bool ref_previous_frame = ((frame_count_ % fixed_ref_prev_step) == 0);
    

    if (ref_previous_frame){
      // try to reference last frame
      RTC_CHECK(!reference_frames_list_.empty());
      frame_info = reference_frames_list_.back().get();
      cfg.Reference(frame_info->slot_id.value());
    } else {
      // try to reference last acked decoded frame
      if (last_decoded_frame_id_.has_value()){
        frame_info = FindFrameInfoByFrameId(last_decoded_frame_id_.value());
        if (frame_info){
          RTC_DCHECK(frame_info->slot_id.has_value());
          cfg.Reference(frame_info->slot_id.value());
        } 
      }
      if (!frame_info) {   // indicate no recent decoded frame
        RTC_CHECK(!reference_frames_list_.empty());
        frame_info = reference_frames_list_.back().get();
        cfg.Reference(frame_info->slot_id.value());
        RTC_LOG(LS_WARNING) << "When trying to reference Acked frame, no recent acked decoded frame available, frame" << current_frame_id_ << " referencing the recent frame" << frame_info->frame_id;
      }
    }
    
    current_frame_ref_frame_id_ = frame_info->frame_id;
    current_frame_importance_ = 0;
    cfg.Update(position);
    UpdateSlot(position, current_frame_id_);
  }
  return {cfg};
}

std::vector<ScalableVideoController::LayerFrameConfig>
DynamicStructureController::NextFrameConfig_FireBreak_R(bool restart) {
  static int fixed_ref_prev_prob = GetFixedFirebreakRefPrevProb();
  LayerFrameConfig cfg;
  cfg.S(0).T(0);
  int position = frame_count_ % kNumSlots;
  if (restart || frame_count_ == 1) {
    cfg.Id(0).Keyframe().Update(position);
    current_frame_ref_frame_id_ = 0;
    current_frame_importance_ = 2;
    UpdateSlot(position, current_frame_id_);
  } else {
    cfg.Id(1);
    FrameInfo* frame_info = nullptr; 
    
    bool ref_previous_frame = (std::rand() % 100) < fixed_ref_prev_prob;

    if (ref_previous_frame){
      // try to reference last frame
      RTC_CHECK(!reference_frames_list_.empty());
      frame_info = reference_frames_list_.back().get();
      cfg.Reference(frame_info->slot_id.value());
    } else {
      // try to reference last acked decoded frame
      if (last_decoded_frame_id_.has_value()){
        frame_info = FindFrameInfoByFrameId(last_decoded_frame_id_.value());
        if (frame_info){
          RTC_DCHECK(frame_info->slot_id.has_value());
          cfg.Reference(frame_info->slot_id.value());
        } 
      }
      if (!frame_info) {   // indicate no recent decoded frame
        RTC_CHECK(!reference_frames_list_.empty());
        frame_info = reference_frames_list_.back().get();
        cfg.Reference(frame_info->slot_id.value());
        RTC_LOG(LS_WARNING) << "When trying to reference Acked frame, no recent acked decoded frame available, frame" << current_frame_id_ << " referencing the recent frame" << frame_info->frame_id;
      }
    }
    
    current_frame_ref_frame_id_ = frame_info->frame_id;
    current_frame_importance_ = 0;
    cfg.Update(position);
    UpdateSlot(position, current_frame_id_);
  }
  return {cfg};
}


std::vector<ScalableVideoController::LayerFrameConfig>
DynamicStructureController::NextFrameConfig_AckedOnly(bool restart) {
  LayerFrameConfig cfg;
  cfg.S(0).T(0);
  int position = frame_count_ % kNumSlots;
  if (restart || frame_count_ == 1) {
    cfg.Id(0).Keyframe().Update(position);
    current_frame_ref_frame_id_ = 0;
    current_frame_importance_ = 2;
    UpdateSlot(position, current_frame_id_);
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
      RTC_CHECK(!reference_frames_list_.empty());
      frame_info = reference_frames_list_.back().get();
      cfg.Reference(frame_info->slot_id.value());
      RTC_LOG(LS_WARNING) << "When trying to reference Acked frame, no recent acked decoded frame available, frame" << current_frame_id_ << " referencing the recent frame" << frame_info->frame_id;
    }
    current_frame_ref_frame_id_ = frame_info->frame_id;
    current_frame_importance_ = 0;
    cfg.Update(position);
    UpdateSlot(position, current_frame_id_);
  }

  return {cfg};
}


std::vector<ScalableVideoController::LayerFrameConfig>
DynamicStructureController::NextFrameConfig(bool restart) {
  {
    webrtc::MutexLock lock(&mutex_);
    if (mode_ == "Intra-only"){
      return NextFrameConfig_IntraOnly(restart);
    } else if (mode_ == "K-step"){
      return NextFrameConfig_Kstep(restart);
    } else if (mode_ == "FireBreak"){
      return NextFrameConfig_FireBreak(restart);
    } else {
      RTC_LOG(LS_WARNING) << "DynamicStructureController::NextFrameConfig unrecognized mode: "
                        << mode_;
      RTC_DCHECK_NOTREACHED();
      return {};
    }
  }
}


}  // namespace webrtc
