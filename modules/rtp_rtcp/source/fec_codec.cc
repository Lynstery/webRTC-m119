#include "modules/rtp_rtcp/source/fec_codec.h"
#include "modules/rtp_rtcp/source/forward_error_correction.h"
#include "modules/rtp_rtcp/source/forward_error_correction_internal.h"
#include "modules/rtp_rtcp/source/ulpfec_header_reader_writer.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/numerics/mod_ops.h"
#include "rtc_base/trace_event.h"
#include "absl/strings/str_format.h"

#include "modules/rtp_rtcp/source/fec_codec.h"
namespace webrtc {
    int XorFecCodec::Encode(
            const ForwardErrorCorrection::PacketList& media_packets,
            size_t num_fec_packets,
            size_t packet_mask_size,
            const uint8_t* packet_masks,
            const std::vector<size_t>& fec_header_sizes,
            std::vector<ForwardErrorCorrection::Packet>* fec_packets){
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
      int num_fec_packets = NumFecPackets(num_media_packets, protection_factor);
      if (num_fec_packets == 0) {
        //return 0;
        num_fec_packets = 1;  // video-expr: Controll FEC ratio. Always generate at least one FEC packet.
      }
      for (int i = 0; i < num_fec_packets; ++i) {
        generated_fec_packets_[i].data.EnsureCapacity(IP_PACKET_SIZE);
        memset(generated_fec_packets_[i].data.MutableData(), 0, IP_PACKET_SIZE);
        // Use this as a marker for untouched packets.
        generated_fec_packets_[i].data.SetSize(0);
        fec_packets->push_back(&generated_fec_packets_[i]);
      }

      internal::PacketMaskTable mask_table(fec_mask_type, num_media_packets);
      packet_mask_size_ = internal::PacketMaskSize(num_media_packets);
      memset(packet_masks_, 0, num_fec_packets * packet_mask_size_);
      internal::GeneratePacketMasks(num_media_packets, num_fec_packets,
                                    num_important_packets, use_unequal_protection,
                                    &mask_table, packet_masks_);

      // Adapt packet masks to missing media packets.
      int num_mask_bits = InsertZerosInPacketMasks(media_packets, num_fec_packets);
      if (num_mask_bits < 0) {
        RTC_LOG(LS_INFO) << "Due to sequence number gaps, cannot protect media "
                            "packets with a single block of FEC packets.";
        fec_packets->clear();
        return -1;
      }
      packet_mask_size_ = internal::PacketMaskSize(num_mask_bits);

      // Write FEC packets to `generated_fec_packets_`.
      GenerateFecPayloads(media_packets, num_fec_packets);
      // TODO(brandtr): Generalize this when multistream protection support is
      // added.
      const uint32_t media_ssrc = ParseSsrc(media_packets.front()->data.data());
      const uint16_t seq_num_base =
          ParseSequenceNumber(media_packets.front()->data.data());
      FinalizeFecHeaders(num_fec_packets, media_ssrc, seq_num_base);

      return 0;
    }



    size_t XorFecCodec::Decode(
        ForwardErrorCorrection::ReceivedFecPacketList* received_fec_packets,
        ForwardErrorCorrection::RecoveredPacketList* recovered_packets){
        return 0;
    }
}
