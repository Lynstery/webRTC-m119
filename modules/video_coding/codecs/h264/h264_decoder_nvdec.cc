#ifdef WEBRTC_USE_H264

#include "modules/video_coding/codecs/h264/h264_decoder_nvdec.h"

#include <limits>

#include "api/video/i420_buffer.h"
#include "common_video/include/video_frame_buffer.h"
#include "rtc_base/logging.h"

// libyuv 用于 NV12 -> I420
#include "third_party/libyuv/include/libyuv.h"

namespace webrtc {

H264DecoderNvdec::H264DecoderNvdec() = default;

H264DecoderNvdec::~H264DecoderNvdec() {
  Release();
}

bool H264DecoderNvdec::InitCuda() {
  // 简单版本：永远用 device 0，新建一个 context
  CUresult cu_res = cuInit(0);
  if (cu_res != CUDA_SUCCESS) {
    RTC_LOG(LS_ERROR) << "cuInit failed, code=" << cu_res;
    return false;
  }

  cu_res = cuDeviceGet(&cu_device_, 0);
  if (cu_res != CUDA_SUCCESS) {
    RTC_LOG(LS_ERROR) << "cuDeviceGet failed, code=" << cu_res;
    return false;
  }

  cu_res = cuCtxCreate(&cu_context_, 0, cu_device_);
  if (cu_res != CUDA_SUCCESS) {
    RTC_LOG(LS_ERROR) << "cuCtxCreate failed, code=" << cu_res;
    cu_context_ = nullptr;
    return false;
  }

  own_context_ = true;
  return true;
}

bool H264DecoderNvdec::Configure(const Settings& settings) {
  if (settings.codec_type() != kVideoCodecH264) {
    RTC_LOG(LS_ERROR) << "H264DecoderNvdec::Configure: codec type is not H264.";
    return false;
  }

  // 重新配置前先清理
  Release();

  if (!cu_context_) {
    if (!InitCuda()) {
      RTC_LOG(LS_ERROR) << "H264DecoderNvdec::Configure: InitCuda failed.";
      return false;
    }
  }

  // 真正的 decoder 延后到第一次 Decode 时创建
  return true;
}

int32_t H264DecoderNvdec::Release() {
  nv_decoder_.reset();

  if (own_context_ && cu_context_) {
    cuCtxDestroy(cu_context_);
    cu_context_ = nullptr;
    own_context_ = false;
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t H264DecoderNvdec::RegisterDecodeCompleteCallback(
    DecodedImageCallback* callback) {
  decoded_image_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

bool H264DecoderNvdec::EnsureDecoderCreated() {
  if (nv_decoder_) {
    return true;
  }

  if (!cu_context_) {
    if (!InitCuda()) {
      return false;
    }
  }

  // 最小配置：host memory 输出，H.264，低延迟
  // 对应 NvDecoder ctor：
  // NvDecoder(CUcontext cuContext, bool bUseDeviceFrame, cudaVideoCodec eCodec,
  //           bool bLowLatency=false, bool bDeviceFramePitched=false,
  //           const Rect* pCropRect=nullptr, const Dim* pResizeDim=nullptr,
  //           bool extract_user_SEI_Message=false, int maxWidth=0,
  //           int maxHeight=0, unsigned int clkRate=1000,
  //           bool force_zero_latency=false,
  //           unsigned int initial_dec_surfaces=0, CUstream custream=nullptr);
  nv_decoder_ = std::make_unique<NvDecoder>(
      cu_context_,
      /*bUseDeviceFrame=*/false,
      cudaVideoCodec_H264,
      /*bLowLatency=*/true);

  return true;
}

int32_t H264DecoderNvdec::Decode(const EncodedImage& input_image,
                                 bool /*missing_frames*/,
                                 int64_t /*render_time_ms*/) {
  if (!decoded_image_callback_) {
    RTC_LOG(LS_WARNING)
        << "H264DecoderNvdec::Decode: callback not set via "
           "RegisterDecodeCompleteCallback.";
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  if (!input_image.data() || input_image.size() == 0) {
    // WebRTC 基本不会用空数据 flush，这里直接忽略
    return WEBRTC_VIDEO_CODEC_OK;
  }

  if (!EnsureDecoderCreated()) {
    RTC_LOG(LS_ERROR) << "H264DecoderNvdec::Decode: decoder create failed.";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // 传给 NVDEC 解码
  // timestamp 可以用 RTP/ntp 时间，这里简单传 ntp_time_ms_
  int64_t timestamp_nvdec = static_cast<int64_t>(input_image.ntp_time_ms_);
  if (timestamp_nvdec < 0) {
    timestamp_nvdec = 0;
  }

  int num_decoded = nv_decoder_->Decode(
      input_image.data(),
      static_cast<int>(input_image.size()),
      /*nFlags=*/0,
      /*nTimestamp=*/timestamp_nvdec);

  if (num_decoded < 0) {
    RTC_LOG(LS_ERROR) << "NvDecoder::Decode returned error: " << num_decoded;
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (num_decoded == 0) {
    // 当前没有可显示帧（内部可能缓存用于 reorder）
    return WEBRTC_VIDEO_CODEC_OK;
  }

  // 每次 Decode 可能产出多帧，全部取出并回调
  for (int i = 0; i < num_decoded; ++i) {
    int64_t frame_ts_nvdec = 0;
    uint8_t* nv12_frame = nv_decoder_->GetFrame(&frame_ts_nvdec);
    if (!nv12_frame) {
      // 理论上不应该发生，保守处理
      continue;
    }

    const int width = nv_decoder_->GetWidth();   // 已按 NV12 对齐
    const int height = nv_decoder_->GetHeight(); // Luma 高度
    const int pitch = nv_decoder_->GetDeviceFramePitch();  // host buffer stride

    // 创建 I420 输出
    rtc::scoped_refptr<I420Buffer> i420_buffer =
        I420Buffer::Create(width, height);

    // NV12 layout:
    //  - Y  平面:  height 行, pitch stride
    //  - UV 平面:  height/2 行, pitch stride, interleaved
    const uint8_t* src_y = nv12_frame;
    const uint8_t* src_uv = nv12_frame + pitch * height;

    int ret = libyuv::NV12ToI420(
        src_y, pitch,
        src_uv, pitch,
        i420_buffer->MutableDataY(), i420_buffer->StrideY(),
        i420_buffer->MutableDataU(), i420_buffer->StrideU(),
        i420_buffer->MutableDataV(), i420_buffer->StrideV(),
        width, height);

    if (ret != 0) {
      RTC_LOG(LS_ERROR) << "libyuv::NV12ToI420 failed, ret=" << ret;
      continue;
    }
    // 构造 WebRTC VideoFrame
    VideoFrame frame = VideoFrame::Builder()
                           .set_video_frame_buffer(i420_buffer)
                           .set_timestamp_rtp(input_image.RtpTimestamp())
                           .set_id(input_image.VideoFrameTrackingId().value_or(0))
                           .build();

    decoded_image_callback_->Decoded(frame);
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

}  // namespace webrtc

#endif  // WEBRTC_USE_H264