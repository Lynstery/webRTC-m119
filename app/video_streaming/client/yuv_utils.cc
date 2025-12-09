#include "app/video_streaming/client/yuv_utils.h"
#include "third_party/libyuv/include/libyuv.h"

rtc::scoped_refptr<webrtc::I420Buffer> ReadI420FrameFromFile(
    const std::string& path, int width, int height) {

  const int cw = (width + 1) / 2;
  const int ch = (height + 1) / 2;

  const size_t y_size = width * height;
  const size_t u_size = cw * ch;
  const size_t v_size = cw * ch;
  const size_t frame_size = y_size + u_size + v_size;

  FILE* fp = fopen(path.c_str(), "rb");
  if (!fp) {
    printf("ReadI420FrameFromFile: Missing file %s\n", path.c_str());
    return nullptr;
  }

  std::unique_ptr<uint8_t[]> buf(new uint8_t[frame_size]);
  size_t read_bytes = fread(buf.get(), 1, frame_size, fp);
  fclose(fp);

  if (read_bytes != frame_size) {
    printf("ReadI420FrameFromFile: Corrupted file %s (read=%zu, expect=%zu)\n",
           path.c_str(), read_bytes, frame_size);
    return nullptr;
  }

  auto i420 = webrtc::I420Buffer::Create(width, height);

  const uint8_t* srcY = buf.get();
  const uint8_t* srcU = srcY + y_size;
  const uint8_t* srcV = srcU + u_size;

  libyuv::I420Copy(
      srcY, width,
      srcU, cw,
      srcV, cw,
      i420->MutableDataY(), i420->StrideY(),
      i420->MutableDataU(), i420->StrideU(),
      i420->MutableDataV(), i420->StrideV(),
      width, height);

  return i420;
}


bool WriteI420FrameToFile(
    const webrtc::I420BufferInterface* buffer,
    const std::string& path) {

  if (!buffer) return false;

  const int w = buffer->width();
  const int h = buffer->height();
  const int cw = (w + 1) / 2;
  const int ch = (h + 1) / 2;

  FILE* fp = fopen(path.c_str(), "wb");
  if (!fp) {
    printf("WriteI420FrameToFile: Failed to open file %s\n", path.c_str());
    return false;
  }

  // Write Y
  for (int i = 0; i < h; ++i)
    fwrite(buffer->DataY() + i * buffer->StrideY(), 1, w, fp);

  // Write U
  for (int i = 0; i < ch; ++i)
    fwrite(buffer->DataU() + i * buffer->StrideU(), 1, cw, fp);

  // Write V
  for (int i = 0; i < ch; ++i)
    fwrite(buffer->DataV() + i * buffer->StrideV(), 1, cw, fp);

  fclose(fp);
  return true;
}