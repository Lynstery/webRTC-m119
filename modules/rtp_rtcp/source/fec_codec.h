#ifndef MODULES_RTP_RTCP_SOURCE_FEC_CODEC_H_
#define MODULES_RTP_RTCP_SOURCE_FEC_CODEC_H_

#include "modules/rtp_rtcp/source/forward_error_correction.h"

namespace webrtc {
    class XorFecCodec : public ForwardErrorCorrection::FecCodec {
    public:
        XorFecCodec() = default;
        ~XorFecCodec() override = default;

    void Encode(
        const ForwardErrorCorrection::PacketList& media_packets,
        size_t num_fec_packets,
        size_t packet_mask_size,
        const uint8_t* packet_masks,
        const std::vector<size_t>& fec_header_sizes,
        std::vector<ForwardErrorCorrection::Packet>* fec_packets) override;

    size_t Decode(
        ForwardErrorCorrection::ReceivedFecPacketList* received_fec_packets,
        ForwardErrorCorrection::RecoveredPacketList* recovered_packets) override;
    };

}  // namespace webrtc
#endif  // MODULES_RTP_RTCP_SOURCE_FEC_CODEC_H_