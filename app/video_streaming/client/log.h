#include <stdio.h>
#include "absl/flags/parse.h"
#include "api/scoped_refptr.h"
#include "rtc_base/thread.h"
#include "system_wrappers/include/field_trial.h"
#include "test/field_trial.h"
#include <string>
#include <vector>
#include <mutex>
#include <chrono>
#include <ctime>
#include <mutex>
#include <vector>
#include <string>
#include <cstdio>
#include <utility>
#include "rtc_base/logging.h"
#include "absl/strings/ascii.h"
#include "absl/strings/match.h"

static std::vector<std::string> BLOCK_KEYWORDS = {
    "openssl", "audio", "AEC3", "AGC", "agc_", "Conn[", "boringssl", "adm_helpers", "Audio", "voice", "port_allocator", "Port[", "Net[", "p2p_transport_channel", "Skip interface",
    "stun_request", "stun_port"
};

class CustomLogSink : public rtc::LogSink {
 public:
  explicit CustomLogSink(){
    blocked_keywords_ = BLOCK_KEYWORDS;
  }

  void OnLogMessage(const std::string& message) override {
    if (ShouldBlock(message)) return;

    std::lock_guard<std::mutex> lock(mu_);
    PrintWithTimestamp(message);
  }

  void OnLogMessage(absl::string_view message) override {
    std::string msg(message);
    if (ShouldBlock(msg)) return;

    std::lock_guard<std::mutex> lock(mu_);
    PrintWithTimestamp(msg);
  }

 private:
  bool ShouldBlock(const std::string& msg) {
    for (const auto& kw : blocked_keywords_) {
      if (absl::StrContains(msg, kw)) return true;
    }
    return false;
  }

  static std::string CurrentThreadLabel() {
    if (auto* t = rtc::Thread::Current()) {
      std::string name = t->name();   
      if (!name.empty()) return name;
    }
    std::ostringstream os;
    os << rtc::CurrentThreadId();
    return os.str();
  }

  void PrintWithTimestamp(const std::string& msg) {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto t = system_clock::to_time_t(now);
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::tm tm_buf;
    localtime_r(&t, &tm_buf);

    char ts[64];
    std::snprintf(ts, sizeof(ts),
                  "%04d-%02d-%02d %02d:%02d:%02d.%03d",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
                  static_cast<int>(ms.count()));
    const std::string thr = CurrentThreadLabel();
    fprintf(stderr, "[%s] [%s] %s\n", ts, thr.c_str(), msg.c_str());
  }

  std::vector<std::string> blocked_keywords_;
  std::mutex mu_;
};
