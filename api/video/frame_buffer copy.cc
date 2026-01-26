/*
 *  Copyright (c) 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "api/video/frame_buffer.h"

#include <algorithm>
#include <cstdint>
#include <queue>

#include "absl/algorithm/container.h"
#include "absl/strings/str_format.h"
#include "absl/container/inlined_vector.h"
#include "api/units/timestamp.h"
#include "api/video/i420_buffer.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/sequence_number_util.h"
#include "rtc_base/trace_event.h"

namespace webrtc {
namespace {
bool ValidReferences(const EncodedFrame& frame) {
  // All references must point backwards, and duplicates are not allowed.
  for (size_t i = 0; i < frame.num_references; ++i) {
    if (frame.references[i] >= frame.Id())
      return false;

    for (size_t j = i + 1; j < frame.num_references; ++j) {
      if (frame.references[i] == frame.references[j])
        return false;
    }
  }

  return true;
}

// Since FrameBuffer::FrameNode is private it can't be used in the function
// signature, hence the FrameIteratorT type.
template <typename FrameIteratorT>
rtc::ArrayView<const int64_t> GetReferences(const FrameIteratorT& it) {
  return {it->second.encoded_frame->references,
          std::min<size_t>(it->second.encoded_frame->num_references,
                           EncodedFrame::kMaxFrameReferences)};
}

template <typename FrameIteratorT>
int64_t GetFrameId(const FrameIteratorT& it) {
  return it->first;
}

template <typename FrameIteratorT>
uint32_t GetTimestamp(const FrameIteratorT& it) {
  return it->second.encoded_frame->RtpTimestamp();
}

template <typename FrameIteratorT>
bool IsLastFrameInTemporalUnit(const FrameIteratorT& it) {
  return it->second.encoded_frame->is_last_spatial_layer;
}
}  // namespace

FrameBuffer::FrameBuffer(int max_size,
                         int max_decode_history,
                         const FieldTrialsView& field_trials)
    : legacy_frame_id_jump_behavior_(
          !field_trials.IsDisabled("WebRTC-LegacyFrameIdJumpBehavior")),
      max_size_(max_size),
      decodable_frames_(DecodableFrameOrder{&frames_}),
      decoded_frame_history_(max_decode_history) {}

bool FrameBuffer::InsertFrame(std::unique_ptr<EncodedFrame> frame,
                               Timestamp deadline) {

  TRACE_EVENT_INSTANT1(
      "video-expr", "Frame:ToFrameBuffer",
      "json",
      absl::StrFormat(
          R"({"rtp_ts":%u, "picture_id":%llu, "tracking_id":%u })",
          frame->RtpTimestamp(),
          frame->Id(),
          frame->VideoFrameTrackingId().value_or(0)
        )
  );

  if (!ValidReferences(*frame)) {
    RTC_DLOG(LS_WARNING) << "Frame " << frame->Id()
                         << " has invalid references, dropping frame.";
    return false;
  }

  // CRITICAL: Check if THIS SPECIFIC frame was already decoded
  // (NOT if we've decoded past it in ID order)
  if (decoded_frame_history_.WasDecoded(frame->Id())) {
    RTC_LOG(LS_WARNING) << "Frame " << frame->Id()
                        << " was already decoded, dropping duplicate.";
    return false;
  }
  // make sure only single reference frames
  RTC_DCHECK(frame->num_references <= 1);

  // Validate reference distance constraint
  if (frame->num_references == 1) {
    int64_t ref_distance = frame->Id() - frame->references[0];
    if (ref_distance > kMaxReferenceDistance || ref_distance <= 0) {
      RTC_LOG(LS_WARNING) << "Frame " << frame->Id()
                          << " has invalid reference distance: " << ref_distance;
      return false;
    }
  }

  // Check if frame is too old to be useful
  if (!frames_.empty()) {
    int64_t min_frame_id = frames_.begin()->first;
    if (frame->Id() < min_frame_id - kMaxReferenceDistance) {
      RTC_LOG(LS_WARNING) << "Frame " << frame->Id()
                          << " is too old (min in buffer: " << min_frame_id << ")";
      return false;
    }
  }

  // Buffer size check and keyframe handling
  if (frames_.size() >= max_size_) {
    if (frame->is_keyframe()) {
      RTC_DLOG(LS_WARNING) << "Keyframe " << frame->Id()
                           << " inserted into full buffer, clearing buffer.";
      Clear();
    } else if (!MakeRoomForFrame(frame->Id(), deadline)) {
      return false;
    }
  }

  int64_t frame_id = frame->Id();

  // Determine parent
  int64_t parent_id = -1;
  if (frame->num_references == 1) {
    parent_id = frame->references[0];
  }

  // Create node
  FrameNode node;
  node.encoded_frame = std::move(frame);
  node.parent_id = parent_id;
  node.decode_deadline = deadline;
  node.subtree_min_deadline = deadline;  // Initially just this frame's deadline
  node.is_decodable = false;
  node.continuous = false;

  // Insert into map
  auto [it, inserted] = frames_.emplace(frame_id, std::move(node));
  if (!inserted) {
    return false;  // Duplicate frame ID
  }

  // CRITICAL: Scan for existing frames that reference this frame (children)
  // This handles the case where children arrived before parent
  auto potential_child_start = frames_.upper_bound(frame_id);
  auto potential_child_end = frames_.upper_bound(frame_id + kMaxReferenceDistance);

  for (auto other_it = potential_child_start; other_it != potential_child_end; ++other_it) {
    if (other_it->second.parent_id == frame_id) {
      it->second.children_ids.push_back(other_it->first);
      // Propagate deadline from child to this node
      if (other_it->second.subtree_min_deadline < it->second.subtree_min_deadline) {
        it->second.subtree_min_deadline = other_it->second.subtree_min_deadline;
      }
    }
  }

  // Update parent's children list (if parent still in buffer)
  if (parent_id != -1) {
    auto parent_it = frames_.find(parent_id);
    if (parent_it != frames_.end()) {
      parent_it->second.children_ids.push_back(frame_id);
      // Propagate deadline upward
      PropagateDeadlineUpward(parent_it);
    }
  }

  // Check decodability and add to queue if ready
  UpdateDecodability(it);

  TRACE_EVENT_INSTANT1(
      "video-expr", "FrameBuffer:Inserted",
      "json",
      absl::StrFormat(
          R"({"picture_id":%llu, "parent_id": %lld, "current_decodable": %u })",
          it->second.encoded_frame->Id(),
          it->second.parent_id,
          it->second.is_decodable ? 1 : 0
        )
  );
  
  // CRITICAL: Propagate continuity to newly discovered children
  if (it->second.continuous) {
    for (int64_t child_id : it->second.children_ids) {
      auto child_it = frames_.find(child_id);
      if (child_it != frames_.end()) {
        UpdateDecodability(child_it);
      }
    }
  }

  return true;
}

absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4>
FrameBuffer::ExtractNextDecodableTemporalUnit() {
  // Wrapper for backward compatibility - extracts single frame
  absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4> result;
  auto frame = ExtractNextDecodableByPriority();
  if (frame) {
    result.push_back(std::move(frame));
  }
  return result;
}

std::unique_ptr<EncodedFrame> FrameBuffer::ExtractNextDecodableByPriority() {
  if (decodable_frames_.empty()) {
    return nullptr;
  }

  // Get highest priority frame (earliest subtree_min_deadline)
  int64_t frame_id = *decodable_frames_.begin();
  decodable_frames_.erase(decodable_frames_.begin());

  auto it = frames_.find(frame_id);
  if (it == frames_.end()) {
    RTC_CHECK_NOTREACHED();
    return nullptr;  // Should not happen
  }

  // Extract the frame
  std::unique_ptr<EncodedFrame> result = std::move(it->second.encoded_frame);
  std::vector<int64_t> children = std::move(it->second.children_ids);

  // Mark as decoded
  uint32_t timestamp = result->RtpTimestamp();
  decoded_frame_history_.InsertDecoded(frame_id, timestamp);
  
  TRACE_EVENT_INSTANT1(
      "video-expr", "FrameBuffer:Extract Decodable",
      "json",
      absl::StrFormat(
          R"({"picture_id":%llu, "rtp_ts": %u })",
          frame_id,
          timestamp
        )
  );

  // Remove from buffer
  frames_.erase(it);

  // Make children decodable
  for (int64_t child_id : children) {
    auto child_it = frames_.find(child_id);
    if (child_it != frames_.end()) {
      UpdateDecodability(child_it);
    }
  }

  return result;
}

void FrameBuffer::DropNextDecodableTemporalUnit() {
  if (decodable_frames_.empty()) {
    return;
  }
  RTC_LOG(LS_WARNING) << "Dropping next decodable frame from buffer.";
  // Get highest priority frame (earliest subtree_min_deadline)
  int64_t frame_id = *decodable_frames_.begin();
  decodable_frames_.erase(decodable_frames_.begin());

  TRACE_EVENT_INSTANT1(
      "video-expr", "FrameBuffer:Drop Decodable",
      "json",
      absl::StrFormat(
          R"({"picture_id":%llu })",
          frame_id
        )
  );

  auto it = frames_.find(frame_id);
  if (it == frames_.end()) {
    return;  // Should not happen
  }

  // Remove from buffer
  CleanupFrameReferences(it);
  frames_.erase(it);

  num_dropped_frames_++;
}

absl::optional<int64_t> FrameBuffer::LastContinuousFrameId() const {
  // Not meaningful with out-of-order decoding
  RTC_CHECK_NOTREACHED();
  return absl::nullopt;
}

absl::optional<int64_t> FrameBuffer::LastContinuousTemporalUnitFrameId() const {
  // Not meaningful with out-of-order decoding
  RTC_CHECK_NOTREACHED();
  return absl::nullopt;
}

absl::optional<FrameBuffer::DecodabilityInfo>
FrameBuffer::DecodableTemporalUnitsInfo() const {
  // Map to new interface
  auto info = NextDecodableFrameInfo();
  if (!info) {
    return absl::nullopt;
  }
  return DecodabilityInfo{
    .next_rtp_timestamp = info->rtp_timestamp,
    .last_rtp_timestamp = info->rtp_timestamp  // Single frame
  };
}

absl::optional<FrameBuffer::NextDecodableInfo>
FrameBuffer::NextDecodableFrameInfo() const {
  if (decodable_frames_.empty()) {
    return absl::nullopt;
  }

  int64_t frame_id = *decodable_frames_.begin();
  auto it = frames_.find(frame_id);

  RTC_DCHECK(it != frames_.end());

  return NextDecodableInfo{
    .rtp_timestamp = it->second.encoded_frame->RtpTimestamp(),
    .deadline = it->second.decode_deadline,
    .frame_id = frame_id
  };
}

int FrameBuffer::GetTotalNumberOfContinuousTemporalUnits() const {
  // Not meaningful with out-of-order decoding
  RTC_CHECK_NOTREACHED();
  return 0;
}

int FrameBuffer::GetTotalNumberOfDroppedFrames() const {
  return num_dropped_frames_;
}

size_t FrameBuffer::CurrentSize() const {
  return frames_.size();
}

bool FrameBuffer::IsContinuous(const FrameIterator& it) const {
  for (int64_t reference : GetReferences(it)) {
    if (decoded_frame_history_.WasDecoded(reference)) {
      continue;
    }

    auto reference_frame_it = frames_.find(reference);
    if (reference_frame_it != frames_.end() &&
        reference_frame_it->second.continuous) {
      continue;
    }

    return false;
  }

  return true;
}

void FrameBuffer::UpdateDecodability(FrameIterator frame_it) {
  int64_t parent_id = frame_it->second.parent_id;

  bool is_decodable = false;
  if (parent_id == -1) {
    // I-frame: always decodable
    is_decodable = true;
  } else if (decoded_frame_history_.WasDecoded(parent_id)) {
    // Parent already decoded
    is_decodable = true;
  } else {
    // Check if parent is in buffer and continuous
    auto parent_it = frames_.find(parent_id);
    if (parent_it != frames_.end() && parent_it->second.continuous) {
      is_decodable = true;
    }
  }

  if (is_decodable && !frame_it->second.is_decodable) {
    frame_it->second.is_decodable = true;
    frame_it->second.continuous = true;
    decodable_frames_.insert(frame_it->first);
    TRACE_EVENT_INSTANT1(
        "video-expr", "FrameBuffer:New Decodable",
        "json",
        absl::StrFormat(
            R"({"picture_id":%llu })",
            frame_it->first
          )
    );
  }
}

void FrameBuffer::PropagateDeadlineUpward(FrameIterator frame_it) {
  Timestamp child_min_deadline = frame_it->second.subtree_min_deadline;

  // Walk up the tree
  int64_t parent_id = frame_it->second.parent_id;
  while (parent_id != -1) {
    auto parent_it = frames_.find(parent_id);
    if (parent_it == frames_.end()) break;

    // If this child has earlier deadline, update parent
    if (child_min_deadline < parent_it->second.subtree_min_deadline) {
      parent_it->second.subtree_min_deadline = child_min_deadline;
      parent_id = parent_it->second.parent_id;  // Continue upward
    } else {
      break;  // No change needed further up
    }
  }
}

void FrameBuffer::PropagateContinuity(int64_t frame_id) {
  // Walk the tree, not the map order
  std::queue<int64_t> to_check;
  to_check.push(frame_id);

  while (!to_check.empty()) {
    int64_t current_id = to_check.front();
    to_check.pop();

    auto it = frames_.find(current_id);
    if (it == frames_.end() || it->second.continuous) {
      continue;
    }

    if (IsContinuous(it)) {
      it->second.continuous = true;
      it->second.is_decodable = true;
      decodable_frames_.insert(current_id);
      TRACE_EVENT_INSTANT1(
        "video-expr", "FrameBuffer:New Decodable",
        "json",
        absl::StrFormat(
            R"({"picture_id":%llu })",
            current_id
          )
      );

      // Check all children
      for (int64_t child_id : it->second.children_ids) {
        to_check.push(child_id);
      }
    }
  }
}

void FrameBuffer::FindNextAndLastDecodableTemporalUnit() {
  RTC_CHECK_NOTREACHED();
}

void FrameBuffer::CleanupFrameReferences(FrameIterator it) {
  int64_t frame_id = it->first;

  // Remove from decodable set
  decodable_frames_.erase(frame_id);

  // Remove from parent's children list
  if (it->second.parent_id != -1) {
    auto parent_it = frames_.find(it->second.parent_id);
    if (parent_it != frames_.end()) {
      auto& children = parent_it->second.children_ids;
      children.erase(
        std::remove(children.begin(), children.end(), frame_id),
        children.end()
      );
    }
  }

  // Mark all children as undecodable
  for (int64_t child_id : it->second.children_ids) {
    auto child_it = frames_.find(child_id);
    if (child_it != frames_.end()) {
      child_it->second.is_decodable = false;
      child_it->second.continuous = false;
      decodable_frames_.erase(child_id);
    }
  }
}

bool FrameBuffer::MakeRoomForFrame(int64_t new_frame_id,
                                    Timestamp new_deadline) {
  // Step 1: Find the maximum frame ID currently in buffer
  int64_t max_frame_id_in_buffer = frames_.empty() ?
      new_frame_id : frames_.rbegin()->first;

  int64_t max_relevant_id = std::max(new_frame_id, max_frame_id_in_buffer);

  // Step 2: Calculate minimum frame ID that could still be useful
  int64_t min_useful_frame_id = max_relevant_id - kMaxReferenceDistance;

  // Step 3: Remove all unreachable frames (age-based dropping)
  auto it = frames_.begin();
  while (it != frames_.end() && it->first < min_useful_frame_id) {
    CleanupFrameReferences(it);
    it = frames_.erase(it);
    num_dropped_frames_++;
  }

  // Step 4: If still full after cleanup, drop oldest remaining frame
  if (frames_.size() >= max_size_) {
    auto oldest_it = frames_.begin();
    CleanupFrameReferences(oldest_it);
    frames_.erase(oldest_it);
    num_dropped_frames_++;
  }

  return frames_.size() < max_size_;
}

void FrameBuffer::Clear() {
  frames_.clear();
  decodable_frames_.clear();
  decoded_frame_history_.Clear();
}

}  // namespace webrtc
