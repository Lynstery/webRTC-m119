#ifndef EXAMPLES_PEERCONNECTION_CLIENT_FLAG_DEFS_H_
#define EXAMPLES_PEERCONNECTION_CLIENT_FLAG_DEFS_H_

#include <string>
#include "absl/flags/flag.h"
#include "absl/flags/declare.h"

ABSL_DECLARE_FLAG(bool, autoconnect);
ABSL_DECLARE_FLAG(std::string, server);
ABSL_DECLARE_FLAG(int, port);
ABSL_DECLARE_FLAG(bool, autocall);
ABSL_DECLARE_FLAG(std::string, video);
ABSL_DECLARE_FLAG(std::string, force_fieldtrials);
ABSL_DECLARE_FLAG(float, fps);
ABSL_DECLARE_FLAG(int, width);
ABSL_DECLARE_FLAG(int, height);
ABSL_DECLARE_FLAG(int, start_index);
ABSL_DECLARE_FLAG(int, end_index);
ABSL_DECLARE_FLAG(int, video_source_threads);
ABSL_DECLARE_FLAG(std::string, trace_file);



#endif  // EXAMPLES_PEERCONNECTION_CLIENT_FLAG_DEFS_H_