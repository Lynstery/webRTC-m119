/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/video_coding/utility/decoded_frames_history.h"

#include <algorithm>

#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

namespace webrtc {
namespace video_coding {

DecodedFramesHistory::DecodedFramesHistory(size_t window_size)
    : buffer_(window_size) {}

DecodedFramesHistory::~DecodedFramesHistory() = default;

void DecodedFramesHistory::InsertDecoded(int64_t frame_id,
                                         uint32_t timestamp) {
  last_decoded_frame_ = frame_id;
  last_decoded_frame_timestamp_ = timestamp;

  int index = FrameIdToIndex(frame_id);

  if (!max_frame_id_seen_) {
    // First decoded frame.
    std::fill(buffer_.begin(), buffer_.end(), false);
    buffer_[index] = true;
    max_frame_id_seen_ = frame_id;
    return;
  }

  // If this frame extends the window forward, clear newly uncovered slots.
  if (frame_id > *max_frame_id_seen_) {
    int64_t delta = frame_id - *max_frame_id_seen_;
    if (delta >= static_cast<int64_t>(buffer_.size())) {
      std::fill(buffer_.begin(), buffer_.end(), false);
    } else {
      int from = FrameIdToIndex(*max_frame_id_seen_ + 1);
      int to = FrameIdToIndex(frame_id);
      if (from <= to) {
        std::fill(buffer_.begin() + from, buffer_.begin() + to + 1, false);
      } else {
        std::fill(buffer_.begin() + from, buffer_.end(), false);
        std::fill(buffer_.begin(), buffer_.begin() + to + 1, false);
      }
    }
    max_frame_id_seen_ = frame_id;
  }

  buffer_[index] = true;
}

bool DecodedFramesHistory::WasDecoded(int64_t frame_id) const {
  if (!max_frame_id_seen_)
    return false;

  if (frame_id <= *max_frame_id_seen_ -
                      static_cast<int64_t>(buffer_.size())) {
    RTC_LOG(LS_WARNING)
        << "Referencing a frame out of the decoded history window. "
           "Assuming undecoded.";
    return false;
  }

  if (frame_id > *max_frame_id_seen_)
    return false;

  return buffer_[FrameIdToIndex(frame_id)];
}

void DecodedFramesHistory::Clear() {
  last_decoded_frame_timestamp_.reset();
  last_decoded_frame_.reset();
  std::fill(buffer_.begin(), buffer_.end(), false);
  last_frame_id_.reset();
  max_frame_id_seen_.reset();
}

absl::optional<int64_t> DecodedFramesHistory::GetLastDecodedFrameId() const {
  return last_decoded_frame_;
}

absl::optional<uint32_t> DecodedFramesHistory::GetLastDecodedFrameTimestamp()
    const {
  return last_decoded_frame_timestamp_;
}

int DecodedFramesHistory::FrameIdToIndex(int64_t frame_id) const {
  int m = frame_id % buffer_.size();
  return m >= 0 ? m : m + buffer_.size();
}

}  // namespace video_coding
}  // namespace webrtc
