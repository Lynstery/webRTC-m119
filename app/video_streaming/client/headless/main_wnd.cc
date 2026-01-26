    
#include "app/video_streaming/client/headless/main_wnd.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cstdint>
#include <utility>

#include "api/video/i420_buffer.h"
#include "api/video/video_frame_buffer.h"
#include "api/video/video_rotation.h"
#include "api/video/video_source_interface.h"
#include "rtc_base/logging.h"
#include "third_party/libyuv/include/libyuv/convert.h"
#include "api/task_queue/default_task_queue_factory.h"
#include "app/video_streaming/client/yuv_utils.h"
#include "absl/flags/flag.h"
#include "app/video_streaming/client/flag_defs.h"
#include "app/video_streaming/client/image_seq_video_track_source.h"
#include "rtc_tools/frame_analyzer/video_quality_analysis.h"
#include "rtc_base/trace_event.h"
#include "absl/strings/str_format.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#define MKDIR(x) mkdir(x, 0755)


#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "app/video_streaming/client/stb_image_write.h"


bool CreateDirectoryRecursive(const std::string& path) {
  if (path.empty()) return false;

  struct stat st;
  if (stat(path.c_str(), &st) == 0) {
    return S_ISDIR(st.st_mode);  // already exists
  }

  // Recursively create parent directories
  size_t pos = path.find_last_of("/\\");
  if (pos != std::string::npos) {
    if (!CreateDirectoryRecursive(path.substr(0, pos))) {
      return false;
    }
  }
  // Try to create this directory
  if (MKDIR(path.c_str()) != 0) {
    if (errno == EEXIST) return true;
    return false;
  }
  return true;
}

HeadlessMainWnd::HeadlessMainWnd(const char* server,
                       int port,
                       bool autoconnect,
                       bool autocall)
    : is_window_(false),
      callback_(NULL),
      server_(server),
      autoconnect_(autoconnect),
      autocall_(autocall){
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%i", port);
  port_ = buffer;
  port_int_ = port;
  main_thread_ = rtc::Thread::Current();
}

HeadlessMainWnd::~HeadlessMainWnd() {
}

void HeadlessMainWnd::RegisterObserver(MainWndCallback* callback) {
  callback_ = callback;
}

bool HeadlessMainWnd::IsWindow() {
  return is_window_;
}

void HeadlessMainWnd::MessageBox(const char* caption,
                                  const char* text,
                                  bool is_error) {
  RTC_LOG(LS_INFO) << "UI Message:" << caption << ": " << text;
}

MainWindow::UI HeadlessMainWnd::current_ui() {
  return current_ui_; 
}


HeadlessMainWnd::VideoRenderer::VideoRenderer(
    std::string save_path,
    webrtc::VideoTrackInterface* track_to_render)
    : save_path_(std::move(save_path)),
      rendered_track_(track_to_render){
    
    auto video_path = absl::GetFlag(FLAGS_video);
    auto start_index = absl::GetFlag(FLAGS_start_index);
    auto end_index = absl::GetFlag(FLAGS_end_index);
    filename_mapper_ = std::make_unique<FileNameMapper>(video_path, start_index, end_index);

    task_queue_factory_ = webrtc::CreateDefaultTaskQueueFactory();
    auto queue_base = task_queue_factory_->CreateTaskQueue(
        "FrameSaverQueue",
        webrtc::TaskQueueFactory::Priority::NORMAL);
    io_queue_ = std::make_unique<rtc::TaskQueue>(std::move(queue_base));

    rendered_track_->AddOrUpdateSink(this, rtc::VideoSinkWants());

    if (!CreateDirectoryRecursive(save_path_)) {
      RTC_LOG(LS_ERROR) << "Failed to create directory: " << save_path_;
    }
}

HeadlessMainWnd::VideoRenderer::~VideoRenderer() {
    if (rendered_track_) {
        rendered_track_->RemoveSink(this);
    }
}

void HeadlessMainWnd::VideoRenderer::OnFrame(
    const webrtc::VideoFrame& video_frame) {

  rtc::scoped_refptr<webrtc::I420BufferInterface> buffer(
      video_frame.video_frame_buffer()->ToI420());

  if (video_frame.rotation() != webrtc::kVideoRotation_0) {
    buffer = webrtc::I420Buffer::Rotate(*buffer, video_frame.rotation());
  }

  const int w = buffer->width();
  const int h = buffer->height();

  int frame_id = video_frame.id();

  if (last_rendered_frame_id_ > frame_id) {
      RTC_LOG(LS_WARNING) << "Rendered: Frame IDs out of order: last=" << last_rendered_frame_id_ << ", current=" << frame_id;
      return;
  }

  last_rendered_frame_id_ = frame_id;
  
  TRACE_EVENT_INSTANT1("video-expr", "Frame:Rendered", "json", absl::StrFormat(
      R"({"tracking_id": %d })",
      frame_id));

  // ---- Deep copy I420 into a new buffer ----
  rtc::scoped_refptr<webrtc::I420Buffer> copy =
      webrtc::I420Buffer::Create(w, h);

  libyuv::I420Copy(
      buffer->DataY(), buffer->StrideY(),
      buffer->DataU(), buffer->StrideU(),
      buffer->DataV(), buffer->StrideV(),
      copy->MutableDataY(), copy->StrideY(),
      copy->MutableDataU(), copy->StrideU(),
      copy->MutableDataV(), copy->StrideV(),
      w, h);

  // ---- Pass deep-copied buffer to async task ----
  io_queue_->PostTask([this, copy, frame_id]() {
    
    // save received frame to file
    char fname[256];
    snprintf(fname, sizeof(fname),"%s/%06d.yuv", save_path_.c_str(), frame_id);
    std::string save_filename = fname;

    if (!WriteI420FrameToFile(copy.get(), save_filename)) {
      RTC_LOG(LS_ERROR) << "Failed to save I420 frame " << frame_id;
    } else{
      // RTC_LOG(LS_INFO) << "Saved I420 frame id=" << frame_id << " to " << save_filename;
    }

    // original frame for quality calculation
    auto ref_filepath = filename_mapper_->GetFilePath(frame_id);
    auto ref_i420 = ReadI420FrameFromFile(ref_filepath, copy->width(), copy->height());
    int width = copy->width();
    int height = copy->height();
    double psnr = 0.0;
    double ssim = 0.0;
    // calculate PSNR/SSIM
    if (ref_i420) {
        psnr = webrtc::test::Psnr(ref_i420, copy);
        ssim = webrtc::test::Ssim(ref_i420, copy);
            
    } 
    
    TRACE_EVENT_INSTANT1("video-expr", "Frame:Quality", "json", absl::StrFormat(
        R"({"tracking_id": %d, "width": %u, "height": %u, "psnr": %.2f, "ssim": %.4f, "ref_filepath": "%s", "recv_filepath": "%s"})",
        frame_id, width, height, psnr, ssim, ref_filepath.c_str(), save_filename.c_str()));

  });
}

void HeadlessMainWnd::StartLocalRenderer(webrtc::VideoTrackInterface* local_video) {}
void HeadlessMainWnd::StopLocalRenderer() {}

void HeadlessMainWnd::StartRemoteRenderer(webrtc::VideoTrackInterface* remote_video) {
  video_renderer_.reset(new VideoRenderer( "received_frames", remote_video));
}
void HeadlessMainWnd::StopRemoteRenderer() {
  video_renderer_.reset();
}

void HeadlessMainWnd::QueueUIThreadCallback(int msg_id, void* data) {
  main_thread_->PostTask([cb = callback_, msg_id, data]() {
      cb->UIThreadCallback(msg_id, data);
  });
}

void HeadlessMainWnd::SetUIThread(rtc::Thread* thread) {
  main_thread_ = thread;
}

bool HeadlessMainWnd::Create() {
  is_window_ = true;
  SwitchToConnectUI();
  return true;
}

bool HeadlessMainWnd::Destroy() {
  is_window_ = false;
  return true;
}

void HeadlessMainWnd::SwitchToConnectUI() {
  current_ui_ = CONNECT_TO_SERVER;
  if (autoconnect_) callback_->StartLogin(server_, port_int_);
}

void HeadlessMainWnd::SwitchToPeerList(const Peers& peers) {
  current_ui_ = LIST_PEERS;
  if (autocall_ && !peers.empty()) {
    int peer_id = peers.begin()->first;
    callback_->ConnectToPeer(peer_id);
  }
}

void HeadlessMainWnd::SwitchToStreamingUI() {
  current_ui_ = STREAMING;
}


