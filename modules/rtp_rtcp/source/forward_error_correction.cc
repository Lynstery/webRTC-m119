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

#include <algorithm>
#include <utility>

#include "absl/algorithm/container.h"
#include "modules/include/module_common_types_public.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/byte_io.h"
#include "modules/rtp_rtcp/source/flexfec_03_header_reader_writer.h"
#include "modules/rtp_rtcp/source/forward_error_correction_internal.h"
#include "modules/rtp_rtcp/source/ulpfec_header_reader_writer.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/trace_event.h"
#include "absl/strings/str_format.h"
#include "rtc_base/numerics/mod_ops.h"
#include "modules/rtp_rtcp/source/forward_error_correction.h"
#include "modules/rtp_rtcp/source/forward_error_correction_xor.h"

namespace webrtc {


ForwardErrorCorrection::ForwardErrorCorrection(
    std::unique_ptr<FecHeaderReader> fec_header_reader,
    std::unique_ptr<FecHeaderWriter> fec_header_writer,
    uint32_t ssrc,
    uint32_t protected_media_ssrc)
    : ssrc_(ssrc),
      protected_media_ssrc_(protected_media_ssrc),
      fec_header_reader_(std::move(fec_header_reader)),
      fec_header_writer_(std::move(fec_header_writer)),
      generated_fec_packets_(fec_header_writer_->MaxFecPackets()),
      packet_mask_size_(0) {}

ForwardErrorCorrection::~ForwardErrorCorrection() = default;

std::unique_ptr<ForwardErrorCorrection> ForwardErrorCorrection::CreateUlpfec(
    uint32_t ssrc) {
  std::unique_ptr<FecHeaderReader> fec_header_reader(new UlpfecHeaderReader());
  std::unique_ptr<FecHeaderWriter> fec_header_writer(new UlpfecHeaderWriter());
  return std::unique_ptr<ForwardErrorCorrection>(new XorForwardErrorCorrection(
      std::move(fec_header_reader), std::move(fec_header_writer), ssrc, ssrc));
}

std::unique_ptr<ForwardErrorCorrection> ForwardErrorCorrection::CreateFlexfec(
    uint32_t ssrc,
    uint32_t protected_media_ssrc) {
  std::unique_ptr<FecHeaderReader> fec_header_reader(
      new Flexfec03HeaderReader());
  std::unique_ptr<FecHeaderWriter> fec_header_writer(
      new Flexfec03HeaderWriter());
  return std::unique_ptr<ForwardErrorCorrection>(new XorForwardErrorCorrection(
      std::move(fec_header_reader), std::move(fec_header_writer), ssrc,
      protected_media_ssrc));
}

uint16_t ForwardErrorCorrection::ParseSequenceNumber(const uint8_t* packet) {
  return (packet[2] << 8) + packet[3];
}

uint32_t ForwardErrorCorrection::ParseSsrc(const uint8_t* packet) {
  return (packet[8] << 24) + (packet[9] << 16) + (packet[10] << 8) + packet[11];
}


size_t ForwardErrorCorrection::MaxPacketOverhead() const {
  return fec_header_writer_->MaxPacketOverhead();
}

FecHeaderReader::FecHeaderReader(size_t max_media_packets,
                                 size_t max_fec_packets)
    : max_media_packets_(max_media_packets),
      max_fec_packets_(max_fec_packets) {}

FecHeaderReader::~FecHeaderReader() = default;

size_t FecHeaderReader::MaxMediaPackets() const {
  return max_media_packets_;
}

size_t FecHeaderReader::MaxFecPackets() const {
  return max_fec_packets_;
}

FecHeaderWriter::FecHeaderWriter(size_t max_media_packets,
                                 size_t max_fec_packets,
                                 size_t max_packet_overhead)
    : max_media_packets_(max_media_packets),
      max_fec_packets_(max_fec_packets),
      max_packet_overhead_(max_packet_overhead) {}

FecHeaderWriter::~FecHeaderWriter() = default;

size_t FecHeaderWriter::MaxMediaPackets() const {
  return max_media_packets_;
}

size_t FecHeaderWriter::MaxFecPackets() const {
  return max_fec_packets_;
}

size_t FecHeaderWriter::MaxPacketOverhead() const {
  return max_packet_overhead_;
}

}  // namespace webrtc
