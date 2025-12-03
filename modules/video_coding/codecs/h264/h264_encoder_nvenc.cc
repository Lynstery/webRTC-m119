#include "nvEncodeAPI.h"
#ifdef WEBRTC_USE_H264

#include "modules/video_coding/codecs/h264/h264_encoder_nvenc.h"

#include <algorithm>
#include <limits>
#include <string>

#include "absl/strings/match.h"
#include "absl/types/optional.h"
#include "api/video/video_codec_constants.h"
#include "common_video/h264/h264_common.h"
#include "common_video/libyuv/include/webrtc_libyuv.h"
#include "modules/video_coding/utility/simulcast_rate_allocator.h"
#include "modules/video_coding/utility/simulcast_utility.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"
#include "system_wrappers/include/metrics.h"
#include "third_party/libyuv/include/libyuv/convert.h"

namespace webrtc {

namespace {

constexpr bool kNvencEncoderDetailedLogging = false;

// QP scaling thresholds.
constexpr int kLowH264QpThreshold = 24;
constexpr int kHighH264QpThreshold = 37;

// Used by histograms. Values of entries should not be changed.
enum NvencEncoderEvent {
  kNvencEncoderEventInit = 0,
  kNvencEncoderEventError = 1,
  kNvencEncoderEventMax = 16,
};

}  // namespace

// ---------- LayerConfig ----------

void H264EncoderNvenc::LayerConfig::SetStreamState(bool send_stream) {
  if (send_stream && !sending) {
    // 第一次启用该层，或者重新启用，需要请求 IDR
    key_frame_request = true;
  }
  sending = send_stream;
}

// ---------- ctor / dtor ----------

H264EncoderNvenc::H264EncoderNvenc(const cricket::VideoCodec& codec)
    : packetization_mode_(H264PacketizationMode::SingleNalUnit),
      max_payload_size_(0),
      number_of_cores_(0),
      encoded_image_callback_(nullptr),
      has_reported_init_(false),
      has_reported_error_(false),
      tl0sync_limit_(0),
      force_next_idr_(false) {
  RTC_CHECK(absl::EqualsIgnoreCase(codec.name, cricket::kH264CodecName));
  std::string packetization_mode_string;
  if (codec.GetParam(cricket::kH264FmtpPacketizationMode,
                     &packetization_mode_string) &&
      packetization_mode_string == "1") {
    packetization_mode_ = H264PacketizationMode::NonInterleaved;
  }

  encoded_images_.reserve(kMaxSimulcastStreams);
  configurations_.reserve(kMaxSimulcastStreams);
  svc_controllers_.reserve(kMaxSimulcastStreams);
  scalability_modes_.resize(kMaxSimulcastStreams);

  memset(&nv_init_params_, 0, sizeof(nv_init_params_));
  memset(&nv_encode_config_, 0, sizeof(nv_encode_config_));
}

H264EncoderNvenc::~H264EncoderNvenc() {
  Release();
}

// ---------- Init / Release ----------

int32_t H264EncoderNvenc::InitEncode(const VideoCodec* inst,
                                     const VideoEncoder::Settings& settings) {
  ReportInit();

  if (!inst || inst->codecType != kVideoCodecH264) {
    ReportError();
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (inst->maxFramerate == 0) {
    ReportError();
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (inst->width < 1 || inst->height < 1) {
    ReportError();
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  int32_t release_ret = Release();
  if (release_ret != WEBRTC_VIDEO_CODEC_OK) {
    ReportError();
    return release_ret;
  }

  int number_of_streams = SimulcastUtility::NumberOfSimulcastStreams(*inst);
  bool doing_simulcast = (number_of_streams > 1);

  if (doing_simulcast &&
      !SimulcastUtility::ValidSimulcastParameters(*inst, number_of_streams)) {
    return WEBRTC_VIDEO_CODEC_ERR_SIMULCAST_PARAMETERS_NOT_SUPPORTED;
  }

  if (number_of_streams != 1) {
    RTC_LOG(LS_WARNING)
        << "H264EncoderNvenc currently only supports single stream.";
  }

  encoded_images_.clear();
  configurations_.clear();
  svc_controllers_.clear();
  scalability_modes_.clear();

  encoded_images_.resize(1);
  configurations_.resize(1);
  scalability_modes_.resize(1);
  svc_controllers_.resize(1);

  max_payload_size_ = settings.max_payload_size;
  number_of_cores_ = settings.number_of_cores;
  encoder_thread_limit_ = settings.encoder_thread_limit;
  codec_ = *inst;

  // 确保 simulcastStream[0] 的分辨率被填好
  if (codec_.numberOfSimulcastStreams == 0) {
    codec_.simulcastStream[0].width = codec_.width;
    codec_.simulcastStream[0].height = codec_.height;
  }

  LayerConfig& cfg = configurations_[0];
  cfg.simulcast_idx = 0;
  cfg.sending = true;
  cfg.width = codec_.simulcastStream[0].width;
  cfg.height = codec_.simulcastStream[0].height;
  cfg.max_frame_rate = static_cast<float>(codec_.maxFramerate);
  cfg.frame_dropping_on = codec_.GetFrameDropEnabled();
  cfg.key_frame_interval = codec_.H264()->keyFrameInterval;
  cfg.num_temporal_layers =
      std::max(codec_.H264()->numberOfTemporalLayers,
               codec_.simulcastStream[0].numberOfTemporalLayers);

  // Codec_settings uses kbits/second; NVENC uses bits/second.
  cfg.max_bps = codec_.maxBitrate * 1000;
  cfg.target_bps = codec_.startBitrate * 1000;

  if (!InitCudaContext()) {
    RTC_LOG(LS_ERROR) << "Failed to init CUDA context for NVENC.";
    Release();
    ReportError();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  if (!InitNvencEncoder()) {
    RTC_LOG(LS_ERROR) << "Failed to init NvEncoderCuda.";
    Release();
    ReportError();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // 初始化 EncodedImage 默认 buffer（大小先按未压缩 I420 分配）
  const size_t new_capacity =
      CalcBufferSize(VideoType::kI420, cfg.width, cfg.height);
  encoded_images_[0].SetEncodedData(EncodedImageBuffer::Create(new_capacity));
  encoded_images_[0]._encodedWidth = cfg.width;
  encoded_images_[0]._encodedHeight = cfg.height;
  encoded_images_[0].set_size(0);

  SimulcastRateAllocator init_allocator(codec_);
  VideoBitrateAllocation allocation =
      init_allocator.Allocate(VideoBitrateAllocationParameters(
          DataRate::KilobitsPerSec(codec_.startBitrate), codec_.maxFramerate));
  SetRates(RateControlParameters(allocation, codec_.maxFramerate));

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t H264EncoderNvenc::Release() {
  nvenc_initialized_ = false;

  if (nv_encoder_) {
    nv_encoder_->DestroyEncoder();
    nv_encoder_.reset();
  }

  if (cu_context_) {
    cuCtxDestroy(cu_context_);
    cu_context_ = nullptr;
  }

  encoded_images_.clear();
  configurations_.clear();
  svc_controllers_.clear();
  scalability_modes_.clear();

  return WEBRTC_VIDEO_CODEC_OK;
}

// ---------- Callbacks / SetRates ----------

int32_t H264EncoderNvenc::RegisterEncodeCompleteCallback(
    EncodedImageCallback* callback) {
  encoded_image_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

void H264EncoderNvenc::SetRates(const RateControlParameters& parameters) {
  if (!nvenc_initialized_) {
    RTC_LOG(LS_WARNING) << "SetRates() while NVENC not initialized.";
    return;
  }

  if (parameters.framerate_fps < 1.0f) {
    RTC_LOG(LS_WARNING) << "Invalid frame rate: " << parameters.framerate_fps;
    return;
  }

  if (parameters.bitrate.get_sum_bps() == 0) {
    // encoder 暂停
    for (auto& cfg : configurations_) {
      cfg.SetStreamState(false);
    }
    return;
  }

  codec_.maxFramerate = static_cast<uint32_t>(parameters.framerate_fps);
  LayerConfig& cfg = configurations_[0];

  cfg.target_bps = parameters.bitrate.GetSpatialLayerSum(0);
  cfg.max_frame_rate = parameters.framerate_fps;

  if (cfg.target_bps == 0) {
    cfg.SetStreamState(false);
    return;
  }

  cfg.SetStreamState(true);
  if (!ReconfigureNvencBitrateFramerate(cfg.target_bps, cfg.max_frame_rate)) {
    RTC_LOG(LS_WARNING) << "Failed to reconfigure NVENC bitrate/framerate.";
  }
}

// ---------- Encode ----------

int32_t H264EncoderNvenc::Encode(
    const VideoFrame& input_frame,
    const std::vector<VideoFrameType>* frame_types) {
  if (!nvenc_initialized_) {
    ReportError();
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (!encoded_image_callback_) {
    RTC_LOG(LS_WARNING)
        << "InitEncode() has been called, but a callback function "
           "has not been set with RegisterEncodeCompleteCallback().";
    ReportError();
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  rtc::scoped_refptr<I420BufferInterface> frame_buffer =
      input_frame.video_frame_buffer()->ToI420();
  if (!frame_buffer) {
    RTC_LOG(LS_ERROR) << "Failed to convert buffer to I420.";
    return WEBRTC_VIDEO_CODEC_ENCODER_FAILURE;
  }

  LayerConfig& cfg = configurations_[0];
  if (!cfg.sending) {
    return WEBRTC_VIDEO_CODEC_OK;
  }

  RTC_DCHECK_EQ(cfg.width, frame_buffer->width());
  RTC_DCHECK_EQ(cfg.height, frame_buffer->height());

  // 是否需要 key frame
  bool send_key_frame = cfg.key_frame_request || force_next_idr_;
  if (!send_key_frame && frame_types && !frame_types->empty()) {
    if ((*frame_types)[0] == VideoFrameType::kVideoFrameKey) {
      send_key_frame = true;
    }
  }
  if (frame_types && !frame_types->empty() &&
      (*frame_types)[0] == VideoFrameType::kEmptyFrame) {
    return WEBRTC_VIDEO_CODEC_OK;
  }

  cfg.key_frame_request = false;
  force_next_idr_ = false;

  // 1) 拿 NVENC 输入缓冲
  const NvEncInputFrame* nv_input = nv_encoder_->GetNextInputFrame();
  RTC_DCHECK(nv_input);
  CUdeviceptr device_ptr =
      reinterpret_cast<CUdeviceptr>(nv_input->inputPtr);

  const int width = cfg.width;
  const int height = cfg.height;

  // 2) I420 -> NV12（CPU 侧）
  const int y_size = width * height;
  const int uv_size = width * height / 2;
  std::vector<uint8_t> nv12(y_size + uv_size);

  int ret = ConvertFromI420(input_frame, VideoType::kNV12, width,
                            nv12.data());
  if (ret < 0) {
    RTC_LOG(LS_ERROR) << "ConvertFromI420 to NV12 failed.";
    return WEBRTC_VIDEO_CODEC_ENCODER_FAILURE;
  }

  // 3) Host → Device 拷贝
  NvEncoderCuda::CopyToDeviceFrame(
      cu_context_,
      nv12.data(),            // pSrcFrame
      width,                  // nSrcPitch (NV12 Y plane pitch)
      device_ptr,             // pDstFrame
      nv_input->pitch,        // dstPitch
      width, height,
      CU_MEMORYTYPE_HOST,
      nv_input->bufferFormat,
      nv_input->chromaOffsets,
      nv_input->numChromaPlanes,
      false /*bUnAlignedDeviceCopy*/);

  // 4) 调用 EncodeFrame
  std::vector<NvEncOutputFrame> vPacket;

  NV_ENC_PIC_PARAMS pic_params = {};
  NV_ENC_PIC_PARAMS* pic_params_ptr = nullptr;
  if (send_key_frame) {
    pic_params.version = NV_ENC_PIC_PARAMS_VER;
    pic_params.encodePicFlags = NV_ENC_PIC_FLAG_FORCEIDR;
    // 把 WebRTC RTP 时间戳传给 NVENC（可选）
    pic_params.inputTimeStamp = input_frame.timestamp();
    pic_params_ptr = &pic_params;
  }

  nv_encoder_->EncodeFrame(vPacket, pic_params_ptr);

  if (vPacket.empty()) {
    // 可能内部缓冲，没有立即输出
    return WEBRTC_VIDEO_CODEC_OK;
  }

  const NvEncOutputFrame& nv_frame = vPacket[0];
  EncodedImage& out = encoded_images_[0];

  out._encodedWidth = cfg.width;
  out._encodedHeight = cfg.height;
  out.SetTimestamp(input_frame.timestamp());
  out.ntp_time_ms_ = input_frame.ntp_time_ms();
  out.SetColorSpace(input_frame.color_space());
  out._frameType =
      (nv_frame.pictureType == NV_ENC_PIC_TYPE_IDR)
          ? VideoFrameType::kVideoFrameKey
          : VideoFrameType::kVideoFrameDelta;
  out.SetSimulcastIndex(cfg.simulcast_idx);

  CopyBitstreamToEncodedImage(nv_frame, &out);

  if (out.size() == 0) {
    return WEBRTC_VIDEO_CODEC_OK;
  }

  // 解析 QP
  h264_bitstream_parser_.ParseBitstream(out);
  out.qp_ = h264_bitstream_parser_.GetLastSliceQp().value_or(-1);

  CodecSpecificInfo codec_specific;
  codec_specific.codecType = kVideoCodecH264;
  codec_specific.codecSpecific.H264.packetization_mode = packetization_mode_;
  codec_specific.codecSpecific.H264.temporal_idx = kNoTemporalIdx;
  codec_specific.codecSpecific.H264.idr_frame =
      (nv_frame.pictureType == NV_ENC_PIC_TYPE_IDR);
  codec_specific.codecSpecific.H264.base_layer_sync = false;
  codec_specific.scalability_mode = absl::nullopt;

  encoded_image_callback_->OnEncodedImage(out, &codec_specific);

  return WEBRTC_VIDEO_CODEC_OK;
}

// ---------- EncoderInfo ----------

VideoEncoder::EncoderInfo H264EncoderNvenc::GetEncoderInfo() const {
  EncoderInfo info;
  info.supports_native_handle = false;
  info.implementation_name = "NVENC (CUDA)";
  info.scaling_settings =
      VideoEncoder::ScalingSettings(kLowH264QpThreshold, kHighH264QpThreshold);
  info.is_hardware_accelerated = true;
  info.supports_simulcast = false;  // 当前实现只支持单路
  info.preferred_pixel_formats = {VideoFrameBuffer::Type::kI420};
  return info;
}

// ---------- NVENC / CUDA helpers ----------

bool H264EncoderNvenc::InitCudaContext() {
  CUresult cu_res = cuInit(0);
  if (cu_res != CUDA_SUCCESS) {
    RTC_LOG(LS_ERROR) << "cuInit failed: " << cu_res;
    return false;
  }

  CUdevice cu_device;
  cu_res = cuDeviceGet(&cu_device, cuda_device_index_);
  if (cu_res != CUDA_SUCCESS) {
    RTC_LOG(LS_ERROR) << "cuDeviceGet failed: " << cu_res;
    return false;
  }

  cu_res = cuCtxCreate(&cu_context_, 0, cu_device);
  if (cu_res != CUDA_SUCCESS) {
    RTC_LOG(LS_ERROR) << "cuCtxCreate failed: " << cu_res;
    cu_context_ = nullptr;
    return false;
  }

  return true;
}

bool H264EncoderNvenc::InitNvencEncoder() {
  if (!cu_context_) {
    RTC_LOG(LS_ERROR) << "CUDA context not initialized.";
    return false;
  }

  LayerConfig& cfg = configurations_[0];

  const NV_ENC_BUFFER_FORMAT buffer_format = NV_ENC_BUFFER_FORMAT_NV12;

  nv_encoder_ = std::make_unique<NvEncoderCuda>(
      cu_context_, cfg.width, cfg.height, buffer_format,
      3 /*nExtraOutputDelay*/,
      false /*bMotionEstimationOnly*/,
      false /*bOPInVideoMemory*/,
      true /*bUseIVFContainer*/);

  if (!nv_encoder_) {
    RTC_LOG(LS_ERROR) << "Failed to allocate NvEncoderCuda.";
    return false;
  }

  memset(&nv_encode_config_, 0, sizeof(nv_encode_config_));
  memset(&nv_init_params_, 0, sizeof(nv_init_params_));

  nv_init_params_.version = NV_ENC_INITIALIZE_PARAMS_VER;
  nv_init_params_.encodeConfig = &nv_encode_config_;
  nv_encode_config_.version = NV_ENC_CONFIG_VER;

  nv_encoder_->CreateDefaultEncoderParams(
      &nv_init_params_,
      NV_ENC_CODEC_H264_GUID,
      NV_ENC_PRESET_P3_GUID,
      NV_ENC_TUNING_INFO_ULTRA_LOW_LATENCY);

  nv_init_params_.encodeWidth = cfg.width;
  nv_init_params_.encodeHeight = cfg.height;
  nv_init_params_.frameRateNum =
      static_cast<uint32_t>(cfg.max_frame_rate * 1000);
  nv_init_params_.frameRateDen = 1000;
  nv_init_params_.enablePTD = 1;
  nv_init_params_.maxEncodeWidth = cfg.width;
  nv_init_params_.maxEncodeHeight = cfg.height;

  nv_encode_config_.rcParams.averageBitRate = cfg.target_bps;
  nv_encode_config_.rcParams.maxBitRate =
      cfg.max_bps ? cfg.max_bps : cfg.target_bps;
  nv_encode_config_.rcParams.rateControlMode = NV_ENC_PARAMS_RC_CBR;
  nv_encode_config_.gopLength = cfg.key_frame_interval > 0
                                    ? cfg.key_frame_interval
                                    : NVENC_INFINITE_GOPLENGTH;
  nv_encode_config_.frameIntervalP = 1;  // IPPP...

  nv_init_params_.encodeConfig = &nv_encode_config_;

  nv_encoder_->CreateEncoder(&nv_init_params_);

  nvenc_initialized_ = true;
  return true;
}

bool H264EncoderNvenc::ReconfigureNvencBitrateFramerate(uint32_t bitrate_bps,
                                                        float framerate_fps) {
  if (!nvenc_initialized_ || !nv_encoder_) {
    return false;
  }

  NV_ENC_RECONFIGURE_PARAMS reconf = {NV_ENC_RECONFIGURE_PARAMS_VER};
  NV_ENC_CONFIG new_config = nv_encode_config_;
  reconf.reInitEncodeParams = nv_init_params_;
  reconf.reInitEncodeParams.encodeConfig = &new_config;

  reconf.resetEncoder = 0;
  reconf.forceIDR = 0;

  reconf.reInitEncodeParams.frameRateNum =
      static_cast<uint32_t>(framerate_fps * 1000);
  reconf.reInitEncodeParams.frameRateDen = 1000;

  new_config.rcParams.averageBitRate = bitrate_bps;
  new_config.rcParams.maxBitRate = bitrate_bps;

  bool ok = nv_encoder_->Reconfigure(&reconf);
  if (ok) {
    nv_encode_config_ = new_config;
    nv_init_params_ = reconf.reInitEncodeParams;
  }
  return ok;
}

void H264EncoderNvenc::CopyBitstreamToEncodedImage(
    const NvEncOutputFrame& nv_frame,
    EncodedImage* encoded_image) {
  RTC_DCHECK(encoded_image);
  const size_t size = nv_frame.frame.size();
  auto buffer = EncodedImageBuffer::Create(size);
  if (size > 0) {
    memcpy(buffer->data(), nv_frame.frame.data(), size);
  }
  encoded_image->SetEncodedData(buffer);
  encoded_image->set_size(size);
}

// ---------- Helpers / Metrics ----------

int H264EncoderNvenc::NumberOfThreads(int width,
                                      int height,
                                      int number_of_cores) const {
  // NVENC 是硬件编码，引擎自己 pipeline，WebRTC 层用 1 就够了。
  return 1;
}

void H264EncoderNvenc::ReportInit() {
  if (has_reported_init_)
    return;
  RTC_HISTOGRAM_ENUMERATION("WebRTC.Video.H264EncoderNvenc.Event",
                            kNvencEncoderEventInit, kNvencEncoderEventMax);
  has_reported_init_ = true;
}

void H264EncoderNvenc::ReportError() {
  if (has_reported_error_)
    return;
  RTC_HISTOGRAM_ENUMERATION("WebRTC.Video.H264EncoderNvenc.Event",
                            kNvencEncoderEventError, kNvencEncoderEventMax);
  has_reported_error_ = true;
}

}  // namespace webrtc

#endif  // WEBRTC_USE_H264