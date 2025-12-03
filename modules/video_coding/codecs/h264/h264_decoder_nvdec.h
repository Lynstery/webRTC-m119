#ifndef MODULES_VIDEO_CODING_CODECS_H264_H264_DECODER_NVDEC_H_
#define MODULES_VIDEO_CODING_CODECS_H264_H264_DECODER_NVDEC_H_

#ifdef WEBRTC_USE_H264  // 与原 H264DecoderImpl 一致的保护

#include <memory>
#include <cuda.h>

#include "modules/video_coding/codecs/h264/include/h264.h"

// NvCodec SDK
#include "NvDecoder/NvDecoder.h"

namespace webrtc {

class H264DecoderNvdec : public H264Decoder {
 public:
  H264DecoderNvdec();
  ~H264DecoderNvdec() override;

  bool Configure(const Settings& settings) override;
  int32_t Release() override;

  int32_t RegisterDecodeCompleteCallback(
      DecodedImageCallback* callback) override;

  int32_t Decode(const EncodedImage& input_image,
                 bool missing_frames,
                 int64_t render_time_ms) override;

  const char* ImplementationName() const override { return "NVDEC"; }

 private:
  bool InitCuda();
  bool EnsureDecoderCreated();

  bool IsInitialized() const { return nv_decoder_ != nullptr; }

 private:
  CUcontext cu_context_ = nullptr;
  CUdevice cu_device_ = 0;
  bool own_context_ = false;

  std::unique_ptr<NvDecoder> nv_decoder_;
  DecodedImageCallback* decoded_image_callback_ = nullptr;
};

}  // namespace webrtc

#endif  // WEBRTC_USE_H264
#endif  // MODULES_VIDEO_CODING_CODECS_H264_H264_DECODER_NVDEC_H_