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


class CustomLogSink : public rtc::LogSink {
 public:
  explicit CustomLogSink(std::vector<std::string> blocked_keywords)
      : blocked_keywords_(std::move(blocked_keywords)) {
    for (auto& kw : blocked_keywords_) {
      absl::AsciiStrToLower(&kw);
    }
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
    std::string lower = absl::AsciiStrToLower(msg);
    for (const auto& kw : blocked_keywords_) {
      if (absl::StrContains(lower, kw)) return true;
    }
    return false;
  }

  static std::string CurrentThreadLabel() {
    // 线程名优先；没有就用线程ID
    if (auto* t = rtc::Thread::Current()) {
      std::string name = t->name();   // 拷贝，避免临时生命周期问题
      if (!name.empty()) return name;
    }
    // 未注册为 rtc::Thread 或没有名字：退化到线程ID
    std::ostringstream os;
    os << rtc::CurrentThreadId();
    return os.str();
  }

  void PrintWithTimestamp(const std::string& msg) {
    // 获取当前系统时间
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
    // 输出格式: [时间戳] 原始日志内容
    fprintf(stderr, "[%s] [%s] %s\n", ts, thr.c_str(), msg.c_str());
  }

  std::vector<std::string> blocked_keywords_;
  std::mutex mu_;
};
