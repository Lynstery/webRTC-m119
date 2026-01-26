/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef MODULES_VIDEO_CODING_SVC_DYNAMIC_STRUCTURE_CONTROLLER_H_
#define MODULES_VIDEO_CODING_SVC_DYNAMIC_STRUCTURE_CONTROLLER_H_

#include <cstdint>
#include <list>
#include <memory>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/base/log_severity.h"
#include "absl/types/optional.h"
#include "api/transport/rtp/dependency_descriptor.h"
#include "api/video/video_bitrate_allocation.h"
#include "api/video/video_frame.h"
#include "common_video/generic_frame_descriptor/generic_frame_info.h"
#include "modules/video_coding/generic_decoder.h"
#include "modules/video_coding/svc/scalable_video_controller.h"
#include "modules/video_coding/utility/quality_scaler.h"
#include "rtc_base/checks.h"

namespace webrtc {

class DynamicStructureController : public ScalableVideoController {
  const int kNumSlots = 7; // slot 0-6 used for acked reference, slot 7 for previous frame
 public:
  DynamicStructureController() = default;
  ~DynamicStructureController() override = default;

  StreamLayersConfig StreamConfig() const override;
  FrameDependencyStructure DependencyStructure() const override;

  void OnRatesUpdated(const VideoBitrateAllocation& bitrates) override {}
  std::vector<LayerFrameConfig> NextFrameConfig(bool restart) override;
  GenericFrameInfo OnEncodeDone(const LayerFrameConfig& config) override;

  void SetMode(std::string mode) override;
  void OnNextFrame(const VideoFrame& frame, bool restart) override;
  void OnReceivedAckFrameDecoded(uint64_t frame_id) override;

  uint64_t GetCurrentFrameRefFrameId() override { return current_frame_ref_frame_id_; }
  uint64_t GetCurrentFrameImportance() override { return current_frame_importance_; }
  void OnRttUpdate(int64_t rtt_ms) override;
  void OnLossNotification(const VideoEncoder::LossNotification& loss_notification) override;
  void OnPacketLossRateUpdate(float packet_loss_rate) override;
  std::vector<LayerFrameConfig> NextFrameConfig_Kstep(bool restart);
  std::vector<LayerFrameConfig> NextFrameConfig_IntraOnly(bool restart);
  std::vector<LayerFrameConfig> NextFrameConfig_AckedOnly(bool restart);
  std::vector<LayerFrameConfig> NextFrameConfig_FireBreak_R(bool restart);
  std::vector<LayerFrameConfig> NextFrameConfig_FireBreak_S(bool restart);
  std::vector<LayerFrameConfig> NextFrameConfig_FireBreak(bool restart);

  class FrameInfo {
   public:
    FrameInfo(uint64_t frame_id, int slot_id)
        : frame_id(frame_id), slot_id(slot_id) {}
    uint64_t frame_id;
    bool acked = false; 
    absl::optional<int> slot_id;
  };
  class SlotInfo {
   public:
    SlotInfo() = default;
    absl::optional<uint64_t> frame_id;
  };

  FrameInfo* FindFrameInfoByFrameId(uint64_t frame_id) {
    auto it = absl::c_lower_bound(
        reference_frames_list_, frame_id,
        [](const std::unique_ptr<FrameInfo>& frame_info, uint64_t fid) {
          return frame_info->frame_id < fid;
        });
    if (it != reference_frames_list_.end() &&
        (*it)->frame_id == frame_id) {
      return it->get();
    }
    return nullptr;
  }

  bool DeleteFrameInfoByFrameId(uint64_t frame_id) {
    auto it = absl::c_lower_bound(
        reference_frames_list_, frame_id,
        [](const std::unique_ptr<FrameInfo>& frame_info, uint64_t fid) {
          return frame_info->frame_id < fid;
        });
    if (it != reference_frames_list_.end() &&
        (*it)->frame_id == frame_id) {
      reference_frames_list_.erase(it);
      return true;
    }
    return false;
  }

  void UpdateSlot(int slot_id, uint64_t frame_id) {
    if (slots_info_[slot_id].frame_id.has_value()) {
      uint64_t old_frame_id = slots_info_[slot_id].frame_id.value();
      FrameInfo* old_frame_info = FindFrameInfoByFrameId(old_frame_id);
      if (old_frame_info) {
        // only keep frames that are in slots
        DeleteFrameInfoByFrameId(old_frame_id); 
      }
    }
    slots_info_[slot_id].frame_id = frame_id;
    reference_frames_list_.emplace_back(std::make_unique<FrameInfo>(frame_id, slot_id));
    RTC_DCHECK(reference_frames_list_.size() <= 8);
  }

  void Reset() {
    reference_frames_list_.clear();
    for (int i = 0; i < kNumSlots; ++i) {
      slots_info_[i].frame_id.reset();
    }
    last_decoded_frame_id_.reset();
    conservative_slot_id_ = -1;
    frame_count_ = 0;
  }
  FrameInfo* GetLatestAckedFrameInfo() {
    for (auto it = reference_frames_list_.rbegin(); it != reference_frames_list_.rend(); ++it){
      if ((*it)->acked){
        return it->get();
      }
    }
    return nullptr;
  }
  
  bool HasEmptySlot() {
    for (int i = 0; i < kNumSlots; ++i) {
      if (!slots_info_[i].frame_id.has_value()) {
        return true;
      }
    }
    return false;
  }

  int GetFirstEmptySlotId() {
    for (int i = 0; i < kNumSlots; ++i) {
      if (!slots_info_[i].frame_id.has_value()) {
        return i;
      }
    }
    return -1;
  }

 private:
  webrtc::Mutex mutex_;
  std::string mode_ = "K-step";
  uint64_t frame_count_ = 0;
  uint64_t current_frame_id_ = 0;
  SlotInfo slots_info_[8];
  std::list<std::unique_ptr<FrameInfo>> reference_frames_list_;
  int conservative_slot_id_;
  uint64_t last_key_frame_id_ = 0;
  absl::optional<uint64_t> last_decoded_frame_id_;
  uint64_t current_frame_ref_frame_id_ = 0;
  uint64_t current_frame_importance_ = 0;
  
  uint64_t rtt_ms_ = 100; 
  uint64_t rtt_ms_ema_ = 100; 
  absl::optional<uint64_t> feedback_delay_frames_;
  int refresh_rate_ = 1;
};

}  // namespace webrtc

#endif  // MODULES_VIDEO_CODING_SVC_DYNAMIC_STRUCTURE_CONTROLLER_H