/*
 *  Copyright (c) 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_VIDEO_FRAME_BUFFER_H_
#define API_VIDEO_FRAME_BUFFER_H_

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <utility>

#include "absl/container/inlined_vector.h"
#include "absl/types/optional.h"
#include "api/field_trials_view.h"
#include "api/video/encoded_frame.h"
#include "modules/video_coding/utility/decoded_frames_history.h"

namespace webrtc {
// The high level idea of the FrameBuffer is to order frames received from the
// network into a decodable stream. Frames are order by frame ID, and grouped
// into temporal units by timestamp. A temporal unit is decodable after all
// referenced frames outside the unit has been decoded, and a temporal unit is
// continuous if all referenced frames are directly or indirectly decodable.
// The FrameBuffer is thread-unsafe.
class FrameBuffer {
 public:
  // Maximum backward reference distance in frame IDs for bounded child discovery.
  static constexpr int64_t kMaxReferenceDistance = 10;

  struct DecodabilityInfo {
    uint32_t next_rtp_timestamp;
    uint32_t last_rtp_timestamp;
  };

  struct NextDecodableInfo {
    uint32_t rtp_timestamp;
    Timestamp deadline;
    int64_t frame_id;
  };

  // The `max_size` determines the maximum number of frames the buffer will
  // store, and max_decode_history determines how far back (by frame ID) the
  // buffer will store if a frame was decoded or not.
  FrameBuffer(int max_size,
              int max_decode_history,
              // TODO(hta): remove field trials!
              const FieldTrialsView& field_trials);
  FrameBuffer(const FrameBuffer&) = delete;
  FrameBuffer& operator=(const FrameBuffer&) = delete;
  FrameBuffer(FrameBuffer&&) = delete;
  FrameBuffer& operator=(FrameBuffer&&) = delete;
  ~FrameBuffer() = default;

  // Inserted frames may only reference backwards, and must have no duplicate
  // references. Frame insertion will fail if `frame` is a duplicate, has
  // already been decoded, invalid, or if the buffer is full and the frame is
  // not a keyframe. Returns true if the frame was successfully inserted.
  bool InsertFrame(std::unique_ptr<EncodedFrame> frame, Timestamp deadline);

  absl::InlinedVector<std::unique_ptr<EncodedFrame>, 4>
  ExtractNextDecodableTemporalUnit();

  // NEW: Extract single frame by priority (replaces temporal unit extraction)
  std::unique_ptr<EncodedFrame> ExtractNextDecodableByPriority();

  // Drop all frames in the next decodable unit.
  void DropNextDecodableTemporalUnit();

  absl::optional<int64_t> LastContinuousFrameId() const;

  // Deprecated: Returns nullopt for out-of-order decoder
  [[deprecated("Not applicable with out-of-order decoding")]]
  absl::optional<int64_t> LastContinuousTemporalUnitFrameId() const;

  absl::optional<DecodabilityInfo> DecodableTemporalUnitsInfo() const;
  absl::optional<NextDecodableInfo> NextDecodableFrameInfo() const;

  // Deprecated: Returns 0 for out-of-order decoder
  [[deprecated("Not applicable with out-of-order decoding")]]
  int GetTotalNumberOfContinuousTemporalUnits() const;

  int GetTotalNumberOfDroppedFrames() const;
  size_t CurrentSize() const;

  bool CheckSlotAvailable(int64_t frame_id);

 private:
  struct FrameNode {
    std::unique_ptr<EncodedFrame> encoded_frame;

    // Tree structure
    int64_t parent_id = -1;  // -1 for I-frames, otherwise the single reference
    std::vector<int64_t> children_ids;  // Frames depending on this one

    // Scheduling metadata
    Timestamp decode_deadline = Timestamp::MinusInfinity();  // When this frame should be displayed
    Timestamp subtree_min_deadline = Timestamp::MinusInfinity();  // Earliest deadline among all descendants

    // State tracking
    bool is_decodable = false;  // True if parent is decoded or this is I-frame
    bool continuous = false;  // Keep for backward compatibility
  };

  using FrameMap = std::map<int64_t, FrameNode>;
  using FrameIterator = FrameMap::iterator;

  struct TemporalUnit {
    // Both first and last are inclusive.
    FrameIterator first_frame;
    FrameIterator last_frame;
  };

  struct DecodableFrameOrder {
    // NOTE: This comparator accesses frames_ptr during set operations.
    // Safe because std::map lookups don't invalidate during insertion,
    // and we only insert/erase from frames_ before/after set operations.
    // Invalid frames are ordered last (lowest priority for extraction).
    const FrameMap* const frames_ptr;

    bool operator()(int64_t a_id, int64_t b_id) const noexcept {
      RTC_DCHECK(frames_ptr != nullptr);
      auto a_it = frames_ptr->find(a_id);
      auto b_it = frames_ptr->find(b_id);

      // Invalid frames are ordered last (lowest priority for extraction)
      if (a_it == frames_ptr->end()) return false;
      if (b_it == frames_ptr->end()) return true;

      // Earlier deadline = higher priority
      //if (a_it->second.subtree_min_deadline != b_it->second.subtree_min_deadline) {
      //  return a_it->second.subtree_min_deadline < b_it->second.subtree_min_deadline;
      //}

      // Tiebreak by frame ID for deterministic ordering
      return a_id < b_id;
    }
  };

  bool IsContinuous(const FrameIterator& it) const;
  void PropagateContinuity(int64_t frame_id);
  void PropagateDeadlineUpward(FrameIterator frame_it);
  void UpdateDecodability(FrameIterator frame_it);
  
  [[deprecated("Not applicable with out-of-order decoding")]]
  void FindNextAndLastDecodableTemporalUnit();  // Keep for now

  bool MakeRoomForFrame(int64_t new_frame_id, Timestamp new_deadline);
  void CleanupFrameReferences(FrameIterator it);
  void Clear();

  const bool legacy_frame_id_jump_behavior_;
  const size_t max_size_;
  FrameMap frames_;
  std::set<int64_t, DecodableFrameOrder> decodable_frames_;
  video_coding::DecodedFramesHistory decoded_frame_history_;

  int num_dropped_frames_ = 0;
};

}  // namespace webrtc

#endif  // API_VIDEO_FRAME_BUFFER_H_
