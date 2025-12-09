#include <cstdio>
#include "api/video/i420_buffer.h"
#include "api/video/video_frame_buffer.h"

rtc::scoped_refptr<webrtc::I420Buffer> ReadI420FrameFromFile(const std::string& path, int width, int height);

bool WriteI420FrameToFile( const webrtc::I420BufferInterface* buffer, const std::string& path);
