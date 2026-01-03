/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/rtp_rtcp/source/forward_error_correction.h"

#include <string.h>
#include <sys/types.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "absl/algorithm/container.h"
#include "modules/include/module_common_types_public.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/byte_io.h"
#include "modules/rtp_rtcp/source/flexfec_03_header_reader_writer.h"
#include "modules/rtp_rtcp/source/forward_error_correction_internal.h"
#include "modules/rtp_rtcp/source/rtp_util.h"
#include "modules/rtp_rtcp/source/ulpfec_header_reader_writer.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/platform_thread_types.h"
#include "rtc_base/trace_event.h"
#include "absl/strings/str_format.h"
#include "rtc_base/numerics/mod_ops.h"
#include "modules/rtp_rtcp/source/forward_error_correction_rs.h" 
#include "rtc_base/copy_on_write_buffer.h"

extern "C" {
#include "jerasure.h"
#include "galois.h"
}

namespace webrtc {


static void DebugPrintDataAndCoding(int k,
                                    int m,
                                    int w,
                                    int size,
                                    char** data,
                                    char** coding,
                                    int max_print_bytes = 60) {
  const int unit = w / 8;  // 每个 GF 元素字节数
  const int print_size = std::min(size, max_print_bytes);

  printf("==== Jerasure Debug Print ====\n");
  printf("k=%d m=%d w=%d size=%d (print %d bytes)\n",
         k, m, w, size, print_size);

  
  for (int i = 0; i < k; ++i){
    if (data && data[i]) {
      printf("D%-2d:", i);
      for (int j = 0; j < print_size; j += unit) {
        printf(" ");
        for (int x = 0; x < unit && j + x < print_size; ++x) {
          printf("%02x", (unsigned char)data[i][j + x]);
        }
      }
      printf("\n");
    }
  }
  for (int i = 0; i < m; ++i) {
   if (coding && coding[i]) {
      printf("C%-2d:", i);
      for (int j = 0; j < print_size; j += unit) {
        printf(" ");
        for (int x = 0; x < unit && j + x < print_size; ++x) {
          printf("%02x", (unsigned char)coding[i][j + x]);
        }
      }
      printf("\n");
    }
  }
  printf("================================\n\n");
}

std::vector<int> JerasureRsCodec::BuildCauchyMatrix(int k, int m, int w) {
  std::vector<int> matrix(m * k);
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < k; ++j) {
      matrix[i * k + j] = galois_single_divide(1, i ^ (m + j), w);
    }
  }
  return matrix;
}

JerasureRsCodec::JerasureRsCodec(int k, int m, int w)
    : k_(k), m_(m), w_(w), matrix_(BuildCauchyMatrix(k, m, w)) {
  RTC_CHECK_GT(k_, 0);
  RTC_CHECK_GT(m_, 0);
  RTC_CHECK(w_ == 8 || w_ == 16 || w_ == 32);

  if (w_ < 32) {
    RTC_CHECK_LE(k_ + m_, (1 << w_)) << "k+m must be <= 2^w";
  }
}

JerasureRsCodec::~JerasureRsCodec() = default;

bool JerasureRsCodec::Encode(char** data_shards,
                             char** coding_shards,
                             int shard_size) {
  RTC_CHECK(data_shards);
  RTC_CHECK(coding_shards);
  RTC_CHECK_EQ(shard_size % sizeof(long), 0);
  /*
  printf(">>> RS Encode: BEFORE\n");
  DebugPrintDataAndCoding(k_, m_, w_,
                           shard_size,
                           data_shards,
                           coding_shards);
  */
  jerasure_matrix_encode(k_, m_, w_,
                         matrix_.data(),
                         data_shards,
                         coding_shards,
                         shard_size);
  /*
  printf(">>> RS Encode: AFTER\n");
  DebugPrintDataAndCoding(k_, m_, w_,
                           shard_size,
                           data_shards,
                           coding_shards);
  */
  return true;
}

bool JerasureRsCodec::Decode(int* erasures,
                             char** data_shards,
                             char** coding_shards,
                             int shard_size) {
  RTC_CHECK(erasures);
  RTC_CHECK(data_shards);
  RTC_CHECK(coding_shards);
  RTC_CHECK_EQ(shard_size % sizeof(long), 0);
  /*
  printf(">>> RS Decode: BEFORE\n");
  printf("Erasures: ");
  for (int i = 0; erasures[i] != -1; ++i) {
    printf("%d ", erasures[i]);
  }
  printf("\n");

  DebugPrintDataAndCoding(k_, m_, w_,
                           shard_size,
                           data_shards,
                           coding_shards);
  */

  const int ret = jerasure_matrix_decode(k_, m_, w_,
                                         matrix_.data(),
                                         /*row_k_ones=*/0,
                                         erasures,
                                         data_shards,
                                         coding_shards,
                                         shard_size);

  /*
  printf(">>> RS Decode: AFTER (ret=%d)\n", ret);
  DebugPrintDataAndCoding(k_, m_, w_,
                           shard_size,
                           data_shards,
                           coding_shards);
  */
  return ret == 0;
}

namespace {
// Transport header size in bytes. Assume UDP/IPv4 as a reasonable minimum.
constexpr size_t kTransportOverhead = 28;

constexpr uint16_t kOldSequenceThreshold = 0x3fff;
constexpr size_t kMaxNumFecPackets = 40;
constexpr size_t kMaxNumRecoveredPackets = 300;
}  // namespace

ReedSolomonForwardErrorCorrection::ReedSolomonForwardErrorCorrection(
    std::unique_ptr<FecHeaderReader> fec_header_reader,
    std::unique_ptr<FecHeaderWriter> fec_header_writer,
    uint32_t ssrc,
    uint32_t protected_media_ssrc)
    : ForwardErrorCorrection(std::move(fec_header_reader), std::move(fec_header_writer), ssrc, protected_media_ssrc){
      received_fec_packets_.clear();
      for (size_t pf = 0; pf < kMaxBlockNum; ++pf) fec_blocks_[pf].reset();
    }
    
ReedSolomonForwardErrorCorrection::~ReedSolomonForwardErrorCorrection() = default;


// sender side

int ReedSolomonForwardErrorCorrection::EncodeFec(const PacketList& media_packets,
                                      uint8_t protection_factor,
                                      int num_important_packets,
                                      bool use_unequal_protection,
                                      FecMaskType fec_mask_type,
                                      std::list<Packet*>* fec_packets) {
  const size_t num_media_packets = media_packets.size();

  // Sanity check arguments.
  RTC_DCHECK_GT(num_media_packets, 0);
  RTC_DCHECK_GE(num_important_packets, 0);
  RTC_DCHECK_LE(num_important_packets, num_media_packets);
  RTC_DCHECK(fec_packets->empty());
  const size_t max_media_packets = fec_header_writer_->MaxMediaPackets();
  if (num_media_packets > max_media_packets) {
    RTC_LOG(LS_WARNING) << "Can't protect " << num_media_packets
                        << " media packets per frame. Max is "
                        << max_media_packets << ".";
    return -1;
  }

  // Error check the media packets.
  for (const auto& media_packet : media_packets) {
    RTC_DCHECK(media_packet);
    if (media_packet->data.size() < kRtpHeaderSize) {
      RTC_LOG(LS_WARNING) << "Media packet " << media_packet->data.size()
                          << " bytes "
                             "is smaller than RTP header.";
      return -1;
    }
    // Ensure the FEC packets will fit in a typical MTU.
    if (media_packet->data.size() + MaxPacketOverhead() + kTransportOverhead >
        IP_PACKET_SIZE) {
      RTC_LOG(LS_WARNING) << "Media packet " << media_packet->data.size()
                          << " bytes "
                             "with overhead is larger than "
                          << IP_PACKET_SIZE << " bytes.";
    }
  }

  // Prepare generated FEC packets.
  size_t num_fec_packets = NumFecPackets(num_media_packets, protection_factor);
  if (num_fec_packets == 0) {
    return 0;
  }
  for (size_t i = 0; i < num_fec_packets; ++i) {
    generated_fec_packets_[i].data.EnsureCapacity(IP_PACKET_SIZE);
    memset(generated_fec_packets_[i].data.MutableData(), 0, IP_PACKET_SIZE);
    // Use this as a marker for untouched packets.
    generated_fec_packets_[i].data.SetSize(0);
    fec_packets->push_back(&generated_fec_packets_[i]);
  }

  internal::PacketMaskTable mask_table(fec_mask_type, num_media_packets);
  packet_mask_size_ = internal::PacketMaskSize(num_media_packets);

  auto tmp = new uint8_t[packet_mask_size_]; 
  memset(tmp, 0, packet_mask_size_);
  // fill packet masks with num_media_packets bits set to 1
  for (size_t i = 0; i < num_media_packets; ++i) tmp[i / 8] |= (1 << (7 - (i % 8)));
  for (size_t i = 0; i < num_fec_packets; ++i) {
    memcpy(packet_masks_ + i * packet_mask_size_, tmp, packet_mask_size_); 
  }
  delete[] tmp; 

  // check missing seq nums 
  uint16_t last_seq_num = ParseSequenceNumber(media_packets.back()->data.data());
  uint16_t first_seq_num = ParseSequenceNumber(media_packets.front()->data.data());
  size_t total_missing_seq_nums = num_media_packets <= 1 ? 0 : 
    static_cast<uint16_t>(last_seq_num - first_seq_num) - num_media_packets + 1;
  RTC_CHECK_EQ(total_missing_seq_nums, 0) << "Non-zero missing seq nums here!";
  
  // Write FEC packets to `generated_fec_packets_`.
  GenerateFecPayloads(media_packets, num_fec_packets);

  const uint32_t media_ssrc = ParseSsrc(media_packets.front()->data.data());
  const uint16_t seq_num_base = ParseSequenceNumber(media_packets.front()->data.data());
  FinalizeFecHeaders(num_fec_packets, media_ssrc, seq_num_base);

  return 0;
}

int ReedSolomonForwardErrorCorrection::NumFecPackets(int num_media_packets,
                                          int protection_factor) {
  // Result in Q0 with an unsigned round.
  int num_fec_packets = (num_media_packets * protection_factor + (1 << 7)) >> 8;
  // Generate at least one FEC packet if we need protection.
  if (protection_factor > 0 && num_fec_packets == 0) {
    num_fec_packets = 1;
  }
  RTC_DCHECK_LE(num_fec_packets, num_media_packets);
  return num_fec_packets;
}

void ReedSolomonForwardErrorCorrection::GenerateFecPayloads(
    const PacketList& media_packets,
    size_t num_fec_packets) {
  RTC_DCHECK(!media_packets.empty());
  
  size_t num_media_packets = media_packets.size();
  
  // calculate max payload length
  size_t max_media_payload_length = 0;  
  for(auto media_packets_it = media_packets.cbegin(); media_packets_it != media_packets.end(); ++media_packets_it) {
    Packet* const media_packet = media_packets_it->get();
    size_t media_payload_length =
        media_packet->data.size() - kRtpHeaderSize;
    if (media_payload_length > max_media_payload_length) {
      max_media_payload_length = media_payload_length;
    }
  }
  
  std::vector<rtc::CopyOnWriteBuffer> media_data_buffers(num_media_packets);
  std::vector<rtc::CopyOnWriteBuffer> coding_data_buffers(num_fec_packets);

  size_t shard_size = 2 + kRtpHeaderSize + max_media_payload_length; // 2 bytes length + header + payload
  shard_size = (shard_size + sizeof(long) -1) / sizeof(long) * sizeof(long); // align to long size

  TRACE_EVENT_INSTANT1("video-expr", "RSFEC:Generate FEC Packets",
                     "json",
                     absl::StrFormat(
                         R"({"num_media":%u, "num_fec":%u, "first_seq":%u, "last_seq":%u, "max_media_payload_length":%u, "shard_size":%u})",
                         num_media_packets, num_fec_packets, ParseSequenceNumber(media_packets.front()->data.data()), ParseSequenceNumber(media_packets.back()->data.data()), max_media_payload_length, shard_size));
  /*
  RTC_LOG(LS_INFO) << "RSFEC: Generating " << num_fec_packets << " FEC packets to protect "
               << num_media_packets << " media packets, from seq " << ParseSequenceNumber(media_packets.front()->data.data()) << " to " << ParseSequenceNumber(media_packets.back()->data.data()) << ", max media payload length="
               << max_media_payload_length << ", shard_size=" << shard_size << ".";
  */

  current_block_id_ = (current_block_id_ + 1) % kMaxBlockNum;

  int idx = 0;
  for(auto media_packets_it = media_packets.cbegin(); media_packets_it != media_packets.end(); ++media_packets_it) {
    Packet* const media_packet = media_packets_it->get();

    media_data_buffers[idx].SetSize(shard_size);
    memset(media_data_buffers[idx].MutableData(), 0, shard_size);

    // Copy length
    uint8_t payload_length_network_order[2];
    ByteWriter<uint16_t>::WriteBigEndian(payload_length_network_order,
                                       media_packet->data.size() - kRtpHeaderSize);
    memcpy(media_data_buffers[idx].MutableData(), payload_length_network_order, 2);
    // Copy header + payload
    memcpy(media_data_buffers[idx].MutableData() + 2, media_packet->data.data(), media_packet->data.size());

    ++idx;
  }
  for (size_t i = 0; i < num_fec_packets; ++i) {
    coding_data_buffers[i].SetSize(shard_size);
    memset(coding_data_buffers[i].MutableData(), 0, shard_size);
  }
  // build data ptr arrays
  std::vector<char*> data_shards_vec(num_media_packets);
  std ::vector<char*> coding_shards_vec(num_fec_packets);
  for (size_t i = 0; i < num_media_packets; ++i) {
    data_shards_vec[i] = reinterpret_cast<char*>(media_data_buffers[i].MutableData());
  }
  for (size_t i = 0; i < num_fec_packets; ++i) {
    coding_shards_vec[i] = reinterpret_cast<char*>(coding_data_buffers[i].MutableData());
  }
  // encode
  JerasureRsCodec rs_codec(num_media_packets, num_fec_packets, 8);
  bool ret = rs_codec.Encode(data_shards_vec.data(), coding_shards_vec.data(), shard_size);
  RTC_CHECK(ret) << "RS encoding failed";

  // copy coding data to generated_fec_packets_

  for (size_t i = 0; i < num_fec_packets; ++i) {
    Packet* const fec_packet = &generated_fec_packets_[i];
    size_t pkt_mask_idx = i * packet_mask_size_;
    const size_t min_packet_mask_size = fec_header_writer_->MinPacketMaskSize(
        &packet_masks_[pkt_mask_idx], packet_mask_size_);
    const size_t fec_header_size =
        fec_header_writer_->FecHeaderSize(min_packet_mask_size);
    
    // RTC_LOG(LS_INFO) << "RSFEC: Generated FEC packet " << i << " with shard size=" << shard_size << ", fec_header_size=" << fec_header_size << ", payload_length=" << (shard_size + fec_header_size) << ".";

    size_t fec_packet_length = fec_header_size + shard_size;
    fec_packet->data.SetSize(fec_packet_length);
    // fill fec header. block information: block id, num of media packets and fec packets. index of this fec packet in the block.
    memset(fec_packet->data.MutableData(), 0, fec_header_size);
    // fill block id in the second byte of the header
    uint8_t block_id = static_cast<uint8_t>(current_block_id_);
    ByteWriter<uint8_t>::WriteBigEndian(fec_packet->data.MutableData() + 1, block_id);
    // fill num of fec packets in the third byte of the header
    ByteWriter<uint8_t>::WriteBigEndian(fec_packet->data.MutableData() + 2, static_cast<uint8_t>(num_fec_packets));
    // fill index of this fec packet in the fourth byte of the header
    ByteWriter<uint8_t>::WriteBigEndian(fec_packet->data.MutableData() + 3, static_cast<uint8_t>(i));
    // after 8 bytes, will be filled later by FinalizeFecHeader()

    // fill fec payload
    memcpy(fec_packet->data.MutableData() + fec_header_size, coding_data_buffers[i].data(), shard_size);
  }
}

int ReedSolomonForwardErrorCorrection::InsertZerosInPacketMasks(
    const PacketList& media_packets,
    size_t num_fec_packets) {
  size_t num_media_packets = media_packets.size();
  if (num_media_packets <= 1) {
    return num_media_packets;
  }
  uint16_t last_seq_num =
      ParseSequenceNumber(media_packets.back()->data.data());
  uint16_t first_seq_num =
      ParseSequenceNumber(media_packets.front()->data.data());
  size_t total_missing_seq_nums =
      static_cast<uint16_t>(last_seq_num - first_seq_num) - num_media_packets +
      1;
  if (total_missing_seq_nums == 0) {
    // All sequence numbers are covered by the packet mask.
    // No zero insertion required.
    return num_media_packets;
  }
  const size_t max_media_packets = fec_header_writer_->MaxMediaPackets();
  if (total_missing_seq_nums + num_media_packets > max_media_packets) {
    return -1;
  }
  // Allocate the new mask.
  size_t tmp_packet_mask_size =
      internal::PacketMaskSize(total_missing_seq_nums + num_media_packets);
  memset(tmp_packet_masks_, 0, num_fec_packets * tmp_packet_mask_size);

  auto media_packets_it = media_packets.cbegin();
  uint16_t prev_seq_num = first_seq_num;
  ++media_packets_it;

  // Insert the first column.
  internal::CopyColumn(tmp_packet_masks_, tmp_packet_mask_size, packet_masks_,
                       packet_mask_size_, num_fec_packets, 0, 0);
  size_t new_bit_index = 1;
  size_t old_bit_index = 1;
  // Insert zeros in the bit mask for every hole in the sequence.
  while (media_packets_it != media_packets.end()) {
    if (new_bit_index == max_media_packets) {
      // We can only cover up to 48 packets.
      break;
    }
    uint16_t seq_num = ParseSequenceNumber((*media_packets_it)->data.data());
    const int num_zeros_to_insert =
        static_cast<uint16_t>(seq_num - prev_seq_num - 1);
    if (num_zeros_to_insert > 0) {
      internal::InsertZeroColumns(num_zeros_to_insert, tmp_packet_masks_,
                                  tmp_packet_mask_size, num_fec_packets,
                                  new_bit_index);
    }
    new_bit_index += num_zeros_to_insert;
    internal::CopyColumn(tmp_packet_masks_, tmp_packet_mask_size, packet_masks_,
                         packet_mask_size_, num_fec_packets, new_bit_index,
                         old_bit_index);
    ++new_bit_index;
    ++old_bit_index;
    prev_seq_num = seq_num;
    ++media_packets_it;
  }
  if (new_bit_index % 8 != 0) {
    // We didn't fill the last byte. Shift bits to correct position.
    for (uint16_t row = 0; row < num_fec_packets; ++row) {
      int new_byte_index = row * tmp_packet_mask_size + new_bit_index / 8;
      tmp_packet_masks_[new_byte_index] <<= (7 - (new_bit_index % 8));
    }
  }
  // Replace the old mask with the new.
  memcpy(packet_masks_, tmp_packet_masks_,
         num_fec_packets * tmp_packet_mask_size);
  return new_bit_index;
}

void ReedSolomonForwardErrorCorrection::FinalizeFecHeaders(size_t num_fec_packets,
                                                uint32_t media_ssrc,
                                                uint16_t seq_num_base) {
  for (size_t i = 0; i < num_fec_packets; ++i) {
    const FecHeaderWriter::ProtectedStream protected_streams[] = {
        {.ssrc = media_ssrc,
         .seq_num_base = seq_num_base,
         .packet_mask = {&packet_masks_[i * packet_mask_size_],
                         packet_mask_size_}}};
    fec_header_writer_->FinalizeFecHeader(protected_streams,
                                          generated_fec_packets_[i]);
  }
}

void ReedSolomonForwardErrorCorrection::ResetState(
    RecoveredPacketList* recovered_packets) {
  // Free the memory for any existing recovered packets, if the caller hasn't.
  recovered_packets->clear();
  received_fec_packets_.clear();
  for (size_t pf = 0; pf < kMaxBlockNum; ++pf) fec_blocks_[pf].reset();
}

// receiver side

void ReedSolomonForwardErrorCorrection::InsertMediaPacket(
    RecoveredPacketList* recovered_packets,
    const ReceivedPacket& received_packet) {
  RTC_DCHECK_EQ(received_packet.ssrc, protected_media_ssrc_);
  TRACE_EVENT0("video-expr", "ReedSolomonForwardErrorCorrection:InsertMediaPacket"); 
  
  TRACE_EVENT_INSTANT1("video-expr", "FlexFEC:Receive RTP Packet",
    "json",
    absl::StrFormat(
        R"({"seq":%u , "payload_length": %u })",
        received_packet.seq_num, received_packet.pkt->data.size() - kRtpHeaderSize
    )
  );

  //RTC_LOG(LS_INFO) << "RSFEC: Received media packet with seq_num="
  //             << received_packet.seq_num << ", payload_length=" << received_packet.pkt->data.size() - kRtpHeaderSize << ".";
  auto insert_pos = absl::c_lower_bound(*recovered_packets, &received_packet, SortablePacket::LessThan());  
  if (insert_pos != recovered_packets->end() &&
      (*insert_pos)->seq_num == received_packet.seq_num) {
    // Duplicate packet, no need to add to list.
    return;
  }

  /*
  // Search for duplicate packets.
  for (const auto& recovered_packet : *recovered_packets) {
    RTC_DCHECK_EQ(recovered_packet->ssrc, received_packet.ssrc);
    if (recovered_packet->seq_num == received_packet.seq_num) {
      // Duplicate packet, no need to add to list.
      return;
    }
  }
  */

  std::unique_ptr<RecoveredPacket> recovered_packet(new RecoveredPacket());
  // This "recovered packet" was not recovered using parity packets.
  recovered_packet->was_recovered = false;
  // This media packet has already been passed on.
  recovered_packet->returned = true;
  recovered_packet->ssrc = received_packet.ssrc;
  recovered_packet->seq_num = received_packet.seq_num;
  recovered_packet->pkt = received_packet.pkt;
  // position, and then just insert the new packet. Would get rid of the sort.
  RecoveredPacket* recovered_packet_ptr = recovered_packet.get();
  recovered_packets->insert(insert_pos, std::move(recovered_packet));
  //recovered_packets->push_back(std::move(recovered_packet));
  //recovered_packets->sort(SortablePacket::LessThan());
  UpdateCoveringFecPackets(*recovered_packet_ptr);
}

void ReedSolomonForwardErrorCorrection::UpdateCoveringFecPackets(
    const RecoveredPacket& packet) {
  for (size_t block_id = 0; block_id < kMaxBlockNum; ++block_id) {
    auto& block = fec_blocks_[block_id];
    if (block && block->decoded == false) {
      auto fec_packet = block->fec_packets[block->first_received_fec_packet_id];
      auto it = absl::c_lower_bound(
          fec_packet->protected_packets,
          &packet,
          SortablePacket::LessThan());
      if (it != fec_packet->protected_packets.end() && (*it)->seq_num == packet.seq_num && 
          it->get()->pkt == nullptr) {
        block->received_media_packet_count++;
        it->get()->pkt = packet.pkt;
      }
    }
  }
}


void ReedSolomonForwardErrorCorrection::InsertFecPacket(
    const RecoveredPacketList& recovered_packets,
    const ReceivedPacket& received_packet) {

  TRACE_EVENT0("video-expr", "ReedSolomonForwardErrorCorrection:InsertFecPacket"); 

  RTC_DCHECK_EQ(received_packet.ssrc, ssrc_);

  // Check for duplicate.
  auto insert_pos = absl::c_lower_bound(
      received_fec_packets_, &received_packet, SortablePacket::LessThan());
  if (insert_pos != received_fec_packets_.end() &&
      (*insert_pos)->seq_num == received_packet.seq_num) {
    // Drop duplicate FEC packet data.
    return;
  }
  /*
  for (const auto& existing_fec_packet : received_fec_packets_) {
    RTC_DCHECK_EQ(existing_fec_packet->ssrc, received_packet.ssrc);
    if (existing_fec_packet->seq_num == received_packet.seq_num) {
      // Drop duplicate FEC packet data.
      return;
    }
  }
  */

  std::unique_ptr<ReceivedFecPacket> fec_packet(new ReceivedFecPacket());
  fec_packet->pkt = received_packet.pkt;
  fec_packet->ssrc = received_packet.ssrc;
  fec_packet->seq_num = received_packet.seq_num;
  
  uint8_t* const data = fec_packet->pkt->data.MutableData();
  fec_packet->block_id = ByteReader<uint8_t>::ReadBigEndian(&data[1]);   
  fec_packet->num_fec_packets_in_block = ByteReader<uint8_t>::ReadBigEndian(&data[2]);
  fec_packet->index_in_block = ByteReader<uint8_t>::ReadBigEndian(&data[3]);

  // Parse ULPFEC/FlexFEC header specific info.
  bool ret = fec_header_reader_->ReadFecHeader(fec_packet.get());
  if (!ret) {
    return;
  }

  RTC_CHECK_EQ(fec_packet->protected_streams.size(), 1);

  if (fec_packet->protected_streams[0].ssrc != protected_media_ssrc_) {
    RTC_LOG(LS_INFO)
        << "Received FEC packet is protecting an unknown media SSRC; dropping.";
    return;
  }

  if (fec_packet->protected_streams[0].packet_mask_offset +
          fec_packet->protected_streams[0].packet_mask_size >
      fec_packet->pkt->data.size()) {
    RTC_LOG(LS_INFO) << "Received corrupted FEC packet; dropping.";
    return;
  }

  // Parse packet mask from header and represent as protected packets.
  for (uint16_t byte_idx = 0;
       byte_idx < fec_packet->protected_streams[0].packet_mask_size;
       ++byte_idx) {
    uint8_t packet_mask =
        fec_packet->pkt
            ->data[fec_packet->protected_streams[0].packet_mask_offset +
                   byte_idx];
    for (uint16_t bit_idx = 0; bit_idx < 8; ++bit_idx) {
      if (packet_mask & (1 << (7 - bit_idx))) {
        std::unique_ptr<ProtectedPacket> protected_packet(
            new ProtectedPacket());
        // This wraps naturally with the sequence number.
        protected_packet->ssrc = protected_media_ssrc_;
        protected_packet->seq_num = static_cast<uint16_t>(
            fec_packet->protected_streams[0].seq_num_base + (byte_idx << 3) +
            bit_idx);
        protected_packet->pkt = nullptr;
        fec_packet->protected_packets.push_back(std::move(protected_packet));
      }
    }
  }
  /*
  RTC_LOG(LS_INFO) << "RSFEC: Received FEC packet with seq_num="
               << fec_packet->seq_num << ", block_id=" << static_cast<int>(fec_packet->block_id) << ", index_in_block=" << static_cast<int>(fec_packet->index_in_block) << ", header_size=" << fec_packet->fec_header_size << ", payload_length=" << fec_packet->pkt->data.size() - fec_packet->fec_header_size
               << ", protecting " <<
        [&]() {
          std::string seqs;
          bool first = true;
          for (const auto& protected_packet : fec_packet->protected_packets) {
            if (!first) {
              seqs += ",";
            }
            seqs += absl::StrFormat("%u", protected_packet->seq_num);
            first = false;
          }
          return seqs;
        }();
  */

  // video-expr: log received FEC packet and its protected packets
  TRACE_EVENT_INSTANT1("video-expr", "FlexFEC:Receive FEC Packet",
    "json",
    absl::StrFormat(
        R"({"fec_seq":%u, "block_id": %u, "index_in_block": %u, "header_size": %u, "payload_length": %u, "protected_seqs": [ %s ]})",
        fec_packet->seq_num, 
        fec_packet->block_id,
        fec_packet->index_in_block,
        fec_packet->fec_header_size,
        fec_packet->pkt->data.size() - fec_packet->fec_header_size,
        [&]() {
          std::string seqs;
          bool first = true;
          for (const auto& protected_packet : fec_packet->protected_packets) {
            if (!first) {
              seqs += ",";
            }
            seqs += absl::StrFormat("%u", protected_packet->seq_num);
            first = false;
          }
          return seqs;
        }()
    )
  ); 

  if (fec_packet->protected_packets.empty()) {
    // All-zero packet mask; we can discard this FEC packet.
    RTC_LOG(LS_WARNING) << "Received FEC packet has an all-zero packet mask.";
  } else {

    
    // find in fec_blocks_
    auto& block = fec_blocks_[fec_packet->block_id];
    if (block && 
      block->fec_packets[block->first_received_fec_packet_id]->protected_streams[0].seq_num_base != 
      fec_packet->protected_streams[0].seq_num_base) {
      block.reset(); // old block, reset
    }

    if (!block) {
      // first fec packet of this block
      // find recovered packets that are covered by this fec packet

      size_t received_protected_count = 0;
      auto rec_it = absl::c_lower_bound(
          recovered_packets,
          fec_packet->protected_packets.front().get(),
          SortablePacket::LessThan()
      );

      auto rec_it_end = absl::c_upper_bound(
          recovered_packets,
          fec_packet->protected_packets.back().get(),
          SortablePacket::LessThan()
      );

      for (auto& protected_packet : fec_packet->protected_packets) {
        uint16_t seq = protected_packet->seq_num;

        while (rec_it != rec_it_end && (*rec_it)->seq_num < seq) {
          ++rec_it;
        }
        if (rec_it == rec_it_end) {
          break;
        }
        if ((*rec_it)->seq_num == seq) {
          protected_packet->pkt = (*rec_it)->pkt;
          ++received_protected_count;
        }
      } 

      block = std::make_unique<RsFecBlock>();
      block->block_id = fec_packet->block_id;
      block->num_fec_packets = fec_packet->num_fec_packets_in_block;
      block->num_media_packets = fec_packet->protected_packets.size();
      block->received_media_packet_count = received_protected_count;
      block->received_fec_packet_count = 1;
      block->fec_packets[fec_packet->index_in_block] = fec_packet.get();
      block->first_received_fec_packet_id = fec_packet->index_in_block;
    } else {
      block->fec_packets[fec_packet->index_in_block] = fec_packet.get();
      block->received_fec_packet_count += 1;
    }
    
    //received_fec_packets_.push_back(std::move(fec_packet));
    //received_fec_packets_.sort(SortablePacket::LessThan());
    received_fec_packets_.insert(insert_pos, std::move(fec_packet)); 

    while (received_fec_packets_.size() > kMaxNumFecPackets) {
      const auto& packet_to_remove = received_fec_packets_.front();
      auto& block = fec_blocks_[packet_to_remove->block_id];
      if (block && 
        block->fec_packets[block->first_received_fec_packet_id]->protected_streams[0].seq_num_base 
        == packet_to_remove->protected_streams[0].seq_num_base) {
        block.reset(); // old block, reset
      }
      received_fec_packets_.pop_front();
    }
  }
}

void ReedSolomonForwardErrorCorrection::InsertPacket(
    const ReceivedPacket& received_packet,
    RecoveredPacketList* recovered_packets) {
  if (!received_fec_packets_.empty() &&
      received_packet.ssrc == received_fec_packets_.front()->ssrc) {
    auto it = received_fec_packets_.begin();
    while (it != received_fec_packets_.end()) {
      uint16_t seq_num_diff = MinDiff(received_packet.seq_num, (*it)->seq_num);
      if (seq_num_diff > kOldSequenceThreshold) {
        auto & block = fec_blocks_[(*it)->block_id];
        if (block && 
          block->fec_packets[block->first_received_fec_packet_id]->protected_streams[0].seq_num_base 
          == (*it)->protected_streams[0].seq_num_base) {
          block.reset(); // old block, reset
        }
        it = received_fec_packets_.erase(it);
      } else {
        // No need to keep iterating, since `received_fec_packets_` is sorted.
        break;
      }
    }
  }

  if (received_packet.is_fec) {
    InsertFecPacket(*recovered_packets, received_packet);
  } else {
    InsertMediaPacket(recovered_packets, received_packet);
  }

  DiscardOldRecoveredPackets(recovered_packets);
}

size_t ReedSolomonForwardErrorCorrection::AttemptRecovery(
    RecoveredPacketList* recovered_packets) {
  TRACE_EVENT0("video-expr", "ReedSolomonForwardErrorCorrection:AttemptRecovery"); 

  size_t num_recovered_packets = 0;
  
  for (size_t block_idx = 0; block_idx < fec_blocks_.size(); ++block_idx) {
    if (fec_blocks_[block_idx] == nullptr) continue;
    RsFecBlock& block = *(fec_blocks_[block_idx].get());
    if (!block.CanDecode()) {
      continue;
    }
    if (block.GetReceivedMediaPacketCount() == block.num_media_packets) {
      // No missing packets to recover.
      continue;
    }
    if (block.decoded) {
      continue;
    }
    
    RTC_LOG(LS_INFO) << "RSFEC: Decoding block " << static_cast<int>(block.block_id)
                       << " with " << block.num_media_packets << " media packets, "
                       << block.num_fec_packets << " FEC packets, "
                       << block.GetReceivedMediaPacketCount()
                       << " received media packets."
                       << " and " << block.GetReceivedFecPacketCount()
                       << " received FEC packets.";

    const int k = block.num_media_packets;
    const int m = block.num_fec_packets;
    const int total_shards = k + m;

    ReceivedFecPacket* any_fec = block.fec_packets[block.first_received_fec_packet_id];

    RTC_DCHECK_EQ(block.num_media_packets, any_fec->protected_packets.size());

    const size_t fec_header_size = any_fec->fec_header_size;
    const size_t shard_size =
        any_fec->pkt->data.size() - fec_header_size;

    std::vector<rtc::CopyOnWriteBuffer> data_shards(k);
    std::vector<rtc::CopyOnWriteBuffer> coding_shards(m);

    std::vector<char*> data_ptrs(k, nullptr);
    std::vector<char*> coding_ptrs(m, nullptr);

    std::vector<int> erasures;
    erasures.reserve(total_shards + 1);

    auto prot_it = any_fec->protected_packets.begin();
    for (int i = 0; i < k; ++i) {
      data_shards[i].SetSize(shard_size);
      memset(data_shards[i].MutableData(), 0, shard_size);
      if ((*prot_it)->pkt != nullptr) {
        auto pkt = (*prot_it)->pkt;
        RTC_DCHECK_LE(2 + pkt->data.size(), shard_size);
        uint8_t payload_length_network_order[2];
        ByteWriter<uint16_t>::WriteBigEndian(payload_length_network_order, pkt->data.size() - kRtpHeaderSize);
        memcpy(data_shards[i].MutableData(), payload_length_network_order, 2);
        // Copy header + payload
        memcpy(data_shards[i].MutableData() + 2, pkt->data.data(), pkt->data.size());
      } else {
        erasures.push_back(i);
      }
      data_ptrs[i] = reinterpret_cast<char*>(data_shards[i].MutableData());
      prot_it++;
    }

    for (int j = 0; j < m; ++j) {
      coding_shards[j].SetSize(shard_size);
      memset(coding_shards[j].MutableData(), 0, shard_size);

      auto packet = block.fec_packets[j];
      if (packet != nullptr) {
        ReceivedFecPacket* fec = packet;
        RTC_DCHECK_EQ(fec->fec_header_size, fec_header_size);
        RTC_DCHECK_EQ(fec->pkt->data.size() - fec_header_size, shard_size);
        memcpy(coding_shards[j].MutableData(),
               fec->pkt->data.data() + fec_header_size,
               shard_size);
      } else {
        erasures.push_back(k + j);
      }

      coding_ptrs[j] = reinterpret_cast<char*>(coding_shards[j].MutableData());
    }

    erasures.push_back(-1);

    JerasureRsCodec rs_codec(k, m, 8);
    if (!rs_codec.Decode(erasures.data(),
                          data_ptrs.data(),
                          coding_ptrs.data(),
                          shard_size)) {
      RTC_LOG(LS_WARNING) << "RS decode failed for block "
                          << static_cast<int>(block.block_id);
      block.decoded = true;
      continue;
    }
    block.decoded = true;
    prot_it = any_fec->protected_packets.begin();
    for (int i = 0; i < k; ++i) {
      if ((*prot_it)->pkt == nullptr) {
        auto data = data_shards[i].data();
        const size_t media_payload_size = ByteReader<uint16_t>::ReadBigEndian(&data[0]);
        std::unique_ptr<RecoveredPacket> recovered_packet(new RecoveredPacket());
        recovered_packet->pkt = new Packet();
        recovered_packet->pkt->data.EnsureCapacity(IP_PACKET_SIZE);
        recovered_packet->pkt->data.SetSize(media_payload_size + kRtpHeaderSize);
        recovered_packet->returned = false;
        recovered_packet->was_recovered = true;
        memcpy(recovered_packet->pkt->data.MutableData(), data + 2, kRtpHeaderSize + media_payload_size);
        ++num_recovered_packets;
        recovered_packet->seq_num = ParseSequenceNumber(recovered_packet->pkt->data.data());
        recovered_packet->ssrc = ParseSsrc(recovered_packet->pkt->data.data());
        TRACE_EVENT_INSTANT1("video-expr", "FlexFEC:Recovered RTP Packet",
          "json",
          absl::StrFormat(
              R"({"seq":%u })",
              recovered_packet->seq_num
          )
        );
        /*
        RTC_LOG(LS_INFO) << "RSFEC: Recovered packet with seq_num="
                         << recovered_packet->seq_num << " payload_size="
                         << media_payload_size;
        */
        RecoveredPacket* recovered_packet_ptr = recovered_packet.get();
        UpdateCoveringFecPackets(*recovered_packet_ptr);
        
        auto insert_pos = absl::c_lower_bound(*recovered_packets, recovered_packet_ptr, SortablePacket::LessThan());
        recovered_packets->insert(insert_pos, std::move(recovered_packet));
        // recovered_packets->push_back(std::move(recovered_packet));
        // recovered_packets->sort(SortablePacket::LessThan());
      }
      prot_it++;
    }
  }

  return num_recovered_packets;
}

void ReedSolomonForwardErrorCorrection::DiscardOldRecoveredPackets(
    RecoveredPacketList* recovered_packets) {
  while (recovered_packets->size() > kMaxNumRecoveredPackets) {
    recovered_packets->pop_front();
  }
  RTC_DCHECK_LE(recovered_packets->size(), kMaxNumRecoveredPackets);
}

ForwardErrorCorrection::DecodeFecResult ReedSolomonForwardErrorCorrection::DecodeFec(
    const ReceivedPacket& received_packet,
    RecoveredPacketList* recovered_packets) {
  RTC_DCHECK(recovered_packets);
               
  InsertPacket(received_packet, recovered_packets);

               
  DecodeFecResult decode_result;
  decode_result.num_recovered_packets = AttemptRecovery(recovered_packets);
  return decode_result;
}

}  // namespace webrtc
