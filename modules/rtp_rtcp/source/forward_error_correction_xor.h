/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_RTP_RTCP_SOURCE_FORWARD_ERROR_CORRECTION_XOR_H_
#define MODULES_RTP_RTCP_SOURCE_FORWARD_ERROR_CORRECTION_XOR_H_

#include <stddef.h>
#include <stdint.h>

#include <list>
#include <memory>
#include <vector>

#include "absl/container/inlined_vector.h"
#include "api/scoped_refptr.h"
#include "modules/include/module_fec_types.h"
#include "modules/rtp_rtcp/include/rtp_header_extension_map.h"
#include "rtc_base/copy_on_write_buffer.h"
#include "modules/rtp_rtcp/source/forward_error_correction.h"

namespace webrtc {

class XorForwardErrorCorrection : public ForwardErrorCorrection {
public:

    ~XorForwardErrorCorrection();

    std::string GetFecName() const override {
        return "XOR";
    }

    int EncodeFec(const PacketList& media_packets,
                        uint8_t protection_factor,
                        int num_important_packets,
                        bool use_unequal_protection,
                        FecMaskType fec_mask_type,
                        std::list<Packet*>* fec_packets) override;

    DecodeFecResult DecodeFec(const ReceivedPacket& received_packet,
                            RecoveredPacketList* recovered_packets) override;

    void ResetState(RecoveredPacketList* recovered_packets) override;

    XorForwardErrorCorrection(std::unique_ptr<FecHeaderReader> fec_header_reader,
                         std::unique_ptr<FecHeaderWriter> fec_header_writer,
                         uint32_t ssrc,
                         uint32_t protected_media_ssrc);
    int NumFecPackets(int num_media_packets, int protection_factor) override;
 private:
  // Analyzes `media_packets` for holes in the sequence and inserts zero columns
  // into the `packet_mask` where those holes are found. Zero columns means that
  // those packets will have no protection.
  // Returns the number of bits used for one row of the new packet mask.
  // Requires that `packet_mask` has at least 6 * `num_fec_packets` bytes
  // allocated.
  int InsertZerosInPacketMasks(const PacketList& media_packets,
                               size_t num_fec_packets);

  // Writes FEC payloads and some recovery fields in the FEC headers.
  void GenerateFecPayloads(const PacketList& media_packets,
                           size_t num_fec_packets);

  // Writes the FEC header fields that are not written by GenerateFecPayloads.
  // This includes writing the packet masks.
  void FinalizeFecHeaders(size_t num_fec_packets,
                          uint32_t media_ssrc,
                          uint16_t seq_num_base);

  // Inserts the `received_packet` into the internal received FEC packet list
  // or into `recovered_packets`.
  void InsertPacket(const ReceivedPacket& received_packet,
                    RecoveredPacketList* recovered_packets);

  // Inserts the `received_packet` into `recovered_packets`. Deletes duplicates.
  void InsertMediaPacket(RecoveredPacketList* recovered_packets,
                         const ReceivedPacket& received_packet);

  
  // Assigns pointers to the recovered packet from all FEC packets which cover
  // it.
  // Note: This reduces the complexity when we want to try to recover a packet
  // since we don't have to find the intersection between recovered packets and
  // packets covered by the FEC packet.
  void UpdateCoveringFecPackets(const RecoveredPacket& packet);

  // Insert `received_packet` into internal FEC list. Deletes duplicates.
  void InsertFecPacket(const RecoveredPacketList& recovered_packets,
                       const ReceivedPacket& received_packet);

  // Assigns pointers to already recovered packets covered by `fec_packet`.
  static void AssignRecoveredPackets(
      const RecoveredPacketList& recovered_packets,
      ReceivedFecPacket* fec_packet);

  // Attempt to recover missing packets, using the internally stored
  // received FEC packets.
  size_t AttemptRecovery(RecoveredPacketList* recovered_packets);

  // Initializes headers and payload before the XOR operation
  // that recovers a packet.
  static bool StartPacketRecovery(const ReceivedFecPacket& fec_packet,
                                  RecoveredPacket* recovered_packet);

  // Performs XOR between the first 8 bytes of `src` and `dst` and stores
  // the result in `dst`. The 3rd and 4th bytes are used for storing
  // the length recovery field.
  static void XorHeaders(const Packet& src, Packet* dst);

  // Performs XOR between the payloads of `src` and `dst` and stores the result
  // in `dst`. The parameter `dst_offset` determines at  what byte the
  // XOR operation starts in `dst`. In total, `payload_length` bytes are XORed.
  static void XorPayloads(const Packet& src,
                          size_t payload_length,
                          size_t dst_offset,
                          Packet* dst);

  // Finalizes recovery of packet by setting RTP header fields.
  // This is not specific to the FEC scheme used.
  static bool FinishPacketRecovery(const ReceivedFecPacket& fec_packet,
                                   RecoveredPacket* recovered_packet);

  // Recover a missing packet.
  static bool RecoverPacket(const ReceivedFecPacket& fec_packet,
                            RecoveredPacket* recovered_packet);

  // Get the number of missing media packets which are covered by `fec_packet`.
  // An FEC packet can recover at most one packet, and if zero packets are
  // missing the FEC packet can be discarded. This function returns 2 when two
  // or more packets are missing.
  static int NumCoveredPacketsMissing(const ReceivedFecPacket& fec_packet);

  // Discards old packets in `recovered_packets`, which are no longer relevant
  // for recovering lost packets.
  void DiscardOldRecoveredPackets(RecoveredPacketList* recovered_packets);

  // Checks if the FEC packet is old enough and no longer relevant for
  // recovering lost media packets.
  bool IsOldFecPacket(const ReceivedFecPacket& fec_packet,
                      const RecoveredPacketList* recovered_packets);
 
};


}  // namespace webrtc

#endif  // MODULES_RTP_RTCP_SOURCE_FORWARD_ERROR_CORRECTION_H_
