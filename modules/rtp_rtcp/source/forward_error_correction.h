/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_RTP_RTCP_SOURCE_FORWARD_ERROR_CORRECTION_H_
#define MODULES_RTP_RTCP_SOURCE_FORWARD_ERROR_CORRECTION_H_

#include <stddef.h>
#include <stdint.h>

#include <cstdint>
#include <list>
#include <memory>
#include <vector>

#include "absl/container/inlined_vector.h"
#include "api/scoped_refptr.h"
#include "api/units/timestamp.h"
#include "modules/include/module_fec_types.h"
#include "modules/rtp_rtcp/include/rtp_header_extension_map.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/forward_error_correction_internal.h"
#include "rtc_base/copy_on_write_buffer.h"
#include "modules/include/module_common_types_public.h"

namespace webrtc {

class FecHeaderReader;
class FecHeaderWriter;

// Performs codec-independent forward error correction (FEC), based on RFC 5109.
// Option exists to enable unequal protection (UEP) across packets.
// This is not to be confused with protection within packets
// (referred to as uneven level protection (ULP) in RFC 5109).
// TODO(brandtr): Split this class into a separate encoder
// and a separate decoder.
class ForwardErrorCorrection {
 public:
  // TODO(holmer): As a next step all these struct-like packet classes should be
  // refactored into proper classes, and their members should be made private.
  // This will require parts of the functionality in forward_error_correction.cc
  // and receiver_fec.cc to be refactored into the packet classes.
  class Packet {
   public:
    Packet() : data(0), ref_count_(0) {}
    ~Packet() = default;

    // Add a reference.
    int32_t AddRef() { return ++ref_count_; }

    // Release a reference. Will delete the object if the reference count
    // reaches zero.
    int32_t Release() {
      int32_t ref_count = --ref_count_;
      if (ref_count == 0) {
        delete this;
      }
      return ref_count;
    }

    rtc::CopyOnWriteBuffer data;  // Packet data.

   private:
    int32_t ref_count_;  // Counts the number of references to a packet.
  };

  // TODO(holmer): Refactor into a proper class.
  class SortablePacket {
   public:
    // Functor which returns true if the sequence number of `first`
    // is < the sequence number of `second`. Should only ever be called for
    // packets belonging to the same SSRC.
    struct LessThan {
      template <typename S, typename T>
      bool operator()(const S& first, const T& second) const {
        RTC_DCHECK_EQ(first->ssrc, second->ssrc);
        return IsNewerSequenceNumber(second->seq_num, first->seq_num);
      }
    };

    uint32_t ssrc;
    uint16_t seq_num;
  };

  // Used for the input to DecodeFec().
  class ReceivedPacket : public SortablePacket {
   public:
    ReceivedPacket() = default;
    ~ReceivedPacket() = default;

    bool is_fec;  // Set to true if this is an FEC packet and false
                  // otherwise.
    bool is_recovered;
    RtpHeaderExtensionMap extensions;
    rtc::scoped_refptr<Packet> pkt;  // Pointer to the packet storage.
  };

  // The recovered list parameter of DecodeFec() references structs of
  // this type.
  // TODO(holmer): Refactor into a proper class.
  class RecoveredPacket : public SortablePacket {
   public:
    RecoveredPacket() = default;
    ~RecoveredPacket() = default;

    bool was_recovered;  // Will be true if this packet was recovered by
                         // the FEC. Otherwise it was a media packet passed in
                         // through the received packet list.
    bool returned;  // True when the packet already has been returned to the
                    // caller through the callback.
    rtc::scoped_refptr<Packet> pkt;  // Pointer to the packet storage.
  };

  // Used to link media packets to their protecting FEC packets.
  //
  // TODO(holmer): Refactor into a proper class.
  class ProtectedPacket : public SortablePacket {
   public:
    ProtectedPacket() = default;
    ~ProtectedPacket() = default;

    rtc::scoped_refptr<ForwardErrorCorrection::Packet> pkt;
  };

  using ProtectedPacketList = std::list<std::unique_ptr<ProtectedPacket>>;

  struct ProtectedStream {
    uint32_t ssrc = 0;
    uint16_t seq_num_base = 0;
    size_t packet_mask_offset = 0;  // Relative start of FEC header.
    size_t packet_mask_size = 0;
  };

  // Used for internal storage of received FEC packets in a list.
  //
  // TODO(holmer): Refactor into a proper class.
  class ReceivedFecPacket : public SortablePacket {
   public:
    // SSRC count is limited by 4 bits of CSRC count in RTP header (max 15).
    // Since most of the time number of SSRCs will be low (probably 1 most of
    // the time) setting this value to 4 for optimization.
    static constexpr size_t kInlinedSsrcsVectorSize = 4;

    ReceivedFecPacket() = default;
    ~ReceivedFecPacket() = default;
    // List of media packets that this FEC packet protects.
    ProtectedPacketList protected_packets;
    // RS related vars.
    uint8_t block_id;
    uint8_t num_fec_packets_in_block;
    uint8_t num_media_packets_in_block;
    uint8_t index_in_block;
    // RTP header fields.
    uint32_t ssrc;
    // FEC header fields.
    size_t fec_header_size;
    absl::InlinedVector<ProtectedStream, kInlinedSsrcsVectorSize>
        protected_streams;
    size_t protection_length;
    // Raw data.
    rtc::scoped_refptr<ForwardErrorCorrection::Packet> pkt;
  };

  using PacketList = std::list<std::unique_ptr<Packet>>;
  using RecoveredPacketList = std::list<std::unique_ptr<RecoveredPacket>>;
  using ReceivedFecPacketList = std::list<std::unique_ptr<ReceivedFecPacket>>;

  virtual ~ForwardErrorCorrection() = 0;
  
  virtual std::string GetFecName() const = 0;

  // Creates a ForwardErrorCorrection tailored for a specific FEC scheme.
  static std::unique_ptr<ForwardErrorCorrection> CreateUlpfec(uint32_t ssrc);
  static std::unique_ptr<ForwardErrorCorrection> CreateFlexfec(
      uint32_t ssrc,
      uint32_t protected_media_ssrc);
  
  virtual int NumFecPackets(int num_media_packets, int protection_factor) = 0;

  // Generates a list of FEC packets from supplied media packets.
  //
  // Input:  media_packets          List of media packets to protect, of type
  //                                Packet. All packets must belong to the
  //                                same frame and the list must not be empty.
  // Input:  protection_factor      FEC protection overhead in the [0, 255]
  //                                domain. To obtain 100% overhead, or an
  //                                equal number of FEC packets as
  //                                media packets, use 255.
  // Input:  num_important_packets  The number of "important" packets in the
  //                                frame. These packets may receive greater
  //                                protection than the remaining packets.
  //                                The important packets must be located at the
  //                                start of the media packet list. For codecs
  //                                with data partitioning, the important
  //                                packets may correspond to first partition
  //                                packets.
  // Input:  use_unequal_protection Parameter to enable/disable unequal
  //                                protection (UEP) across packets. Enabling
  //                                UEP will allocate more protection to the
  //                                num_important_packets from the start of the
  //                                media_packets.
  // Input:  fec_mask_type          The type of packet mask used in the FEC.
  //                                Random or bursty type may be selected. The
  //                                bursty type is only defined up to 12 media
  //                                packets. If the number of media packets is
  //                                above 12, the packet masks from the random
  //                                table will be selected.
  // Output: fec_packets            List of pointers to generated FEC packets,
  //                                of type Packet. Must be empty on entry.
  //                                The memory available through the list will
  //                                be valid until the next call to
  //                                EncodeFec().
  //
  // Returns 0 on success, -1 on failure.
  //
  virtual int EncodeFec(const PacketList& media_packets,
                uint8_t protection_factor,
                int num_important_packets,
                bool use_unequal_protection,
                FecMaskType fec_mask_type,
                std::list<Packet*>* fec_packets) = 0;

  // Decodes a list of received media and FEC packets. It will parse the
  // `received_packets`, storing FEC packets internally, and move
  // media packets to `recovered_packets`. The recovered list will be
  // sorted by ascending sequence number and have duplicates removed.
  // The function should be called as new packets arrive, and
  // `recovered_packets` will be progressively assembled with each call.
  // When the function returns, `received_packets` will be empty.
  //
  // The caller will allocate packets submitted through `received_packets`.
  // The function will handle allocation of recovered packets.
  //
  // Input:  received_packets   List of new received packets, of type
  //                            ReceivedPacket, belonging to a single
  //                            frame. At output the list will be empty,
  //                            with packets either stored internally,
  //                            or accessible through the recovered list.
  // Output: recovered_packets  List of recovered media packets, of type
  //                            RecoveredPacket, belonging to a single
  //                            frame. The memory available through the
  //                            list will be valid until the next call to
  //                            DecodeFec().
  //
  struct DecodeFecResult {
    // Number of recovered media packets using FEC.
    size_t num_recovered_packets = 0;
  };

  virtual DecodeFecResult DecodeFec(const ReceivedPacket& received_packet,
                            RecoveredPacketList* recovered_packets) = 0;

  // Gets the maximum size of the FEC headers in bytes, which must be
  // accounted for as packet overhead.
  size_t MaxPacketOverhead() const;

  // Reset internal states from last frame and clear `recovered_packets`.
  // Frees all memory allocated by this class.
  virtual void ResetState(RecoveredPacketList* recovered_packets) = 0;
  // TODO(brandtr): Remove these functions when the Packet classes
  // have been refactored.
  static uint16_t ParseSequenceNumber(const uint8_t* packet);
  static uint32_t ParseSsrc(const uint8_t* packet);
  static uint32_t ParseTimestamp(const uint8_t* packet);

  ForwardErrorCorrection(std::unique_ptr<FecHeaderReader> fec_header_reader,
                         std::unique_ptr<FecHeaderWriter> fec_header_writer,
                         uint32_t ssrc,
                         uint32_t protected_media_ssrc);

  // These SSRCs are only used by the decoder.
  const uint32_t ssrc_;
  const uint32_t protected_media_ssrc_;

  std::unique_ptr<FecHeaderReader> fec_header_reader_;
  std::unique_ptr<FecHeaderWriter> fec_header_writer_;

  std::vector<Packet> generated_fec_packets_;
  ReceivedFecPacketList received_fec_packets_;

  // Arrays used to avoid dynamically allocating memory when generating
  // the packet masks.
  // (There are never more than `kUlpfecMaxMediaPackets` FEC packets generated.)
  uint8_t packet_masks_[kUlpfecMaxMediaPackets * kUlpfecMaxPacketMaskSize];
  uint8_t tmp_packet_masks_[kUlpfecMaxMediaPackets * kUlpfecMaxPacketMaskSize];
  size_t packet_mask_size_;
};

// Classes derived from FecHeader{Reader,Writer} encapsulate the
// specifics of reading and writing FEC header for, e.g., ULPFEC
// and FlexFEC.
class FecHeaderReader {
 public:
  virtual ~FecHeaderReader();

  // The maximum number of media packets that can be covered by one FEC packet.
  size_t MaxMediaPackets() const;

  // The maximum number of FEC packets that is supported, per call
  // to ForwardErrorCorrection::EncodeFec().
  size_t MaxFecPackets() const;

  // Parses FEC header and stores information in ReceivedFecPacket members.
  virtual bool ReadFecHeader(
      ForwardErrorCorrection::ReceivedFecPacket* fec_packet) const = 0;

 protected:
  FecHeaderReader(size_t max_media_packets, size_t max_fec_packets);

  const size_t max_media_packets_;
  const size_t max_fec_packets_;
};

class FecHeaderWriter {
 public:
  struct ProtectedStream {
    uint32_t ssrc = 0;
    uint16_t seq_num_base = 0;
    rtc::ArrayView<const uint8_t> packet_mask;
  };

  virtual ~FecHeaderWriter();

  // The maximum number of media packets that can be covered by one FEC packet.
  size_t MaxMediaPackets() const;

  // The maximum number of FEC packets that is supported, per call
  // to ForwardErrorCorrection::EncodeFec().
  size_t MaxFecPackets() const;

  // The maximum overhead (in bytes) per packet, due to FEC headers.
  size_t MaxPacketOverhead() const;

  // Calculates the minimum packet mask size needed (in bytes),
  // given the discrete options of the ULPFEC masks and the bits
  // set in the current packet mask.
  virtual size_t MinPacketMaskSize(const uint8_t* packet_mask,
                                   size_t packet_mask_size) const = 0;

  // The header size (in bytes), given the packet mask size.
  virtual size_t FecHeaderSize(size_t packet_mask_size) const = 0;

  // Writes FEC header.
  virtual void FinalizeFecHeader(
      rtc::ArrayView<const ProtectedStream> protected_streams,
      ForwardErrorCorrection::Packet& fec_packet) const = 0;

 protected:
  FecHeaderWriter(size_t max_media_packets,
                  size_t max_fec_packets,
                  size_t max_packet_overhead);

  const size_t max_media_packets_;
  const size_t max_fec_packets_;
  const size_t max_packet_overhead_;
};

}  // namespace webrtc

#endif  // MODULES_RTP_RTCP_SOURCE_FORWARD_ERROR_CORRECTION_H_
