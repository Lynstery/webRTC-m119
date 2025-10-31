#include "absl/flags/flag.h"
#include "app/video_streaming/client/defaults.h"
#include "app/video_streaming/client/flag_defs.h"

ABSL_FLAG(bool,
          autoconnect,
          true,
          "Connect to the server without user intervention.");

ABSL_FLAG(std::string, server, "localhost", "The server to connect to.");

ABSL_FLAG(int,
          port,
          kDefaultServerPort,
          "The port on which the server is listening.");

ABSL_FLAG(bool,
          autocall,
          false,
          "Call the first available other client on the server without user "
          "intervention. Note: this flag should only be set to true on one "
          "of the two clients.");

ABSL_FLAG(std::string,
          force_fieldtrials,
          "",
          "Field trials control experimental features...");

ABSL_FLAG(std::string, video, kDefaultVideoStream, "The video stream to use.");

ABSL_FLAG(float,
          fps,
          kDefaultFps,
          "The frames per second for the video stream.");

ABSL_FLAG(int,
          width,
          kDefaultWidth,
          "width to which the video frames are scaled. 0 means no scaling.");

ABSL_FLAG(int,
          height,
          kDefaultHeight,
          "height to which the video frames are scaled. 0 means no scaling.");
ABSL_FLAG(int,
          start_index,
          kDefaultStartIndex,
          "The starting index of the video sequence.");
ABSL_FLAG(int,
          end_index,
          kDefaultEndIndex,
          "The ending index of the video sequence. 0 means infinite sequence.");

ABSL_FLAG(int,
          video_source_threads,
          kDefaultVideoSourceThreads,
          "The number of threads to use for video source processing.");

