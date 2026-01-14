/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/fec_controller_frame_based.h"  // NOLINT

#include <stdlib.h>

#include <algorithm>
#include <cstddef>
#include <string>

#include "modules/include/module_fec_types.h"
#include "rtc_base/logging.h"
#include "system_wrappers/include/field_trial.h"
#include "absl/strings/numbers.h"
#include "rtc_base/helpers.h"

namespace webrtc {

const float kProtectionOverheadRateThreshold = 0.5;

FecControllerFrameBased::FecControllerFrameBased(
    Clock* clock,
    VCMProtectionCallback* protection_callback)
    : clock_(clock),
      protection_callback_(protection_callback),
      max_payload_size_(1460),
      overhead_threshold_(GetProtectionOverheadRateThreshold()) {}

FecControllerFrameBased::FecControllerFrameBased(Clock* clock)
    : clock_(clock),
      max_payload_size_(1460),
      overhead_threshold_(GetProtectionOverheadRateThreshold()) {}

FecControllerFrameBased::~FecControllerFrameBased(void) {
}

void FecControllerFrameBased::SetProtectionCallback(
    VCMProtectionCallback* protection_callback) {
  protection_callback_ = protection_callback;
}

void FecControllerFrameBased::SetEncodingData(size_t width,
                                           size_t height,
                                           size_t num_temporal_layers,
                                           size_t max_payload_size) {
  MutexLock lock(&mutex_);
  max_payload_size_ = max_payload_size;
}

float FecControllerFrameBased::GetProtectionOverheadRateThreshold() {
  return 100; // No overhead limit for frame-based FEC.
}


// In this implementation, we use frame-based FEC, so UpdateFecRates only uses to get network stats.
uint32_t FecControllerFrameBased::UpdateFecRates(
    uint32_t estimated_bitrate_bps,
    int actual_framerate_fps,
    uint8_t fraction_lost,
    std::vector<bool> loss_mask_vector,
    int64_t round_trip_time_ms) {
  // Sanity check.
  if (actual_framerate_fps < 1.0) {
    actual_framerate_fps = 1.0;
  }
  FecProtectionParams delta_fec_params = {.fec_rate = frame_based_fec_rate_, .max_fec_frames = 1, .fec_mask_type = kFecMaskRandom};
  FecProtectionParams key_fec_params = {.fec_rate = frame_based_fec_rate_, .max_fec_frames = 1, .fec_mask_type = kFecMaskRandom};
  // Update protection callback with protection settings.
  uint32_t sent_video_rate_bps = 0;
  uint32_t sent_nack_rate_bps = 0;
  uint32_t sent_fec_rate_bps = 0;
  // Rate cost of the protection methods.
  float protection_overhead_rate = 0.0f;
  // TODO(Marco): Pass FEC protection values per layer.
  protection_callback_->ProtectionRequest(
      &delta_fec_params, &key_fec_params, &sent_video_rate_bps,
      &sent_nack_rate_bps, &sent_fec_rate_bps);
  uint32_t sent_total_rate_bps =
      sent_video_rate_bps + sent_nack_rate_bps + sent_fec_rate_bps;
  // Estimate the overhead costs of the next second as staying the same
  // wrt the source bitrate.
  if (sent_total_rate_bps > 0) {
    protection_overhead_rate =
        static_cast<float>(sent_nack_rate_bps + sent_fec_rate_bps) /
        sent_total_rate_bps;
  }
  // Cap the overhead estimate to a threshold, default is 50%.
  protection_overhead_rate =
      std::min(protection_overhead_rate, overhead_threshold_);
  // Source coding rate: total rate - protection overhead.
  
  /*
   RTC_LOG(LS_INFO)
    << "[FECStats]"
    << " key_fec_rate=" << key_fec_params.fec_rate << " | "
    << " delta_fec_rate=" << delta_fec_params.fec_rate << " | "
    << " key_max_fec_frames=" << key_fec_params.max_fec_frames << " | "
    << " delta_max_fec_frames=" << delta_fec_params.max_fec_frames << " | "
    << " sent_video_rate_bps=" << sent_video_rate_bps << " | "
    << " sent_nack_rate_bps=" << sent_nack_rate_bps << " | "
    << " sent_fec_rate_bps=" << sent_fec_rate_bps;
  */

  return estimated_bitrate_bps * (1.0 - protection_overhead_rate);
}

void FecControllerFrameBased::SetProtectionMethod(bool enable_fec,
                                               bool enable_nack) {
}

void FecControllerFrameBased::UpdateWithEncodedData(
    const EncodedImage& encoded_image) {
  const size_t encoded_length = encoded_image.size();
  MutexLock lock(&mutex_);
  if (encoded_length > 0) {
    current_frame_ref_frame_id_ = encoded_image.frame_reference_idx_;
    current_frame_importance_ = encoded_image.frame_reference_importance_;
    int fixed_fec_ratio = rtc::GetFixedFECRatio();
    if (current_frame_importance_ == 0) {
      frame_based_fec_rate_ = fixed_fec_ratio;
    } else if (current_frame_importance_ == 1) {
      frame_based_fec_rate_ = fixed_fec_ratio + 10; 
    } else {
      frame_based_fec_rate_ = fixed_fec_ratio + 20; 
    }
    // update fec rate
    FecProtectionParams delta_fec_params = {.fec_rate = frame_based_fec_rate_, .max_fec_frames = 1, .fec_mask_type = kFecMaskRandom};
    FecProtectionParams key_fec_params = {.fec_rate = frame_based_fec_rate_, .max_fec_frames = 1, .fec_mask_type = kFecMaskRandom};
    uint32_t sent_video_rate_bps = 0;
    uint32_t sent_nack_rate_bps = 0;
    uint32_t sent_fec_rate_bps = 0;
    protection_callback_->ProtectionRequest(
        &delta_fec_params, &key_fec_params, &sent_video_rate_bps,
        &sent_nack_rate_bps, &sent_fec_rate_bps);
  }
}

bool FecControllerFrameBased::UseLossVectorMask() {
  return true;
}

}  // namespace webrtc
