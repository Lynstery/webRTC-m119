#ifndef MODULES_VIDEO_CODING_CODECS_H264_H264_ENCODER_NVENC_H_
#define MODULES_VIDEO_CODING_CODECS_H264_H264_ENCODER_NVENC_H_

#ifdef WEBRTC_USE_H264

#include <memory>
#include <vector>

#include "absl/types/optional.h"
#include "api/video/i420_buffer.h"
#include "api/video/video_codec_constants.h"
#include "api/video_codecs/video_encoder.h"
#include "common_video/h264/h264_bitstream_parser.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "modules/video_coding/svc/scalable_video_controller.h"
#include "modules/video_coding/utility/quality_scaler.h"

#include "NvEncoder/NvEncoder.h"
#include "NvEncoder/NvEncoderCuda.h"
#include <cuda.h>

namespace webrtc {

class H264EncoderNvenc : public H264Encoder {
 public:
  struct LayerConfig {
    int simulcast_idx = 0;
    int width = -1;
    int height = -1;
    bool sending = true;
    bool key_frame_request = false;
    float max_frame_rate = 0;
    uint32_t target_bps = 0;
    uint32_t max_bps = 0;
    bool frame_dropping_on = false;
    int key_frame_interval = 0;
    int num_temporal_layers = 1;

    void SetStreamState(bool send_stream);
  };

 public:
  explicit H264EncoderNvenc(const cricket::VideoCodec& codec);
  ~H264EncoderNvenc() override;

  int32_t InitEncode(const VideoCodec* codec_settings,
                     const VideoEncoder::Settings& settings) override;
  int32_t Release() override;

  int32_t RegisterEncodeCompleteCallback(
      EncodedImageCallback* callback) override;

  void SetRates(const RateControlParameters& parameters) override;

  int32_t Encode(const VideoFrame& frame,
                 const std::vector<VideoFrameType>* frame_types) override;

  EncoderInfo GetEncoderInfo() const override;

  H264PacketizationMode PacketizationModeForTesting() const {
    return packetization_mode_;
  }

 private:
  bool InitCudaContext();
  bool InitNvencEncoder();
  bool ReconfigureNvencBitrateFramerate(uint32_t bitrate_bps,
                                        float framerate_fps);

  void CopyBitstreamToEncodedImage(const NvEncOutputFrame& nv_frame,
                                   EncodedImage* encoded_image);

  // Metrics
  void ReportInit();
  void ReportError();

  int NumberOfThreads(int width, int height, int number_of_cores) const;

 private:
  // NVENC / CUDA
  CUcontext cu_context_ = nullptr;
  int cuda_device_index_ = 0;
  std::unique_ptr<NvEncoderCuda> nv_encoder_;
  NV_ENC_INITIALIZE_PARAMS nv_init_params_;
  NV_ENC_CONFIG nv_encode_config_;
  bool nvenc_initialized_ = false;

  // WebRTC 配置（当前只支持单路）
  std::vector<LayerConfig> configurations_;
  std::vector<EncodedImage> encoded_images_;
  std::vector<std::unique_ptr<ScalableVideoController>> svc_controllers_;
  absl::InlinedVector<absl::optional<ScalabilityMode>, kMaxSimulcastStreams>
      scalability_modes_;

  VideoCodec codec_;
  H264PacketizationMode packetization_mode_;
  size_t max_payload_size_;
  int32_t number_of_cores_;
  absl::optional<int> encoder_thread_limit_;
  EncodedImageCallback* encoded_image_callback_;

  webrtc::H264BitstreamParser h264_bitstream_parser_;

  bool has_reported_init_;
  bool has_reported_error_;

  uint8_t tl0sync_limit_;
  bool force_next_idr_ = false;
};

}  // namespace webrtc

#endif  // WEBRTC_USE_H264

#endif  // MODULES_VIDEO_CODING_CODECS_H264_H264_ENCODER_NVENC_H_