#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace scan_planner
{
struct TimingWindow
{
  const char *name;
  double budget_ms;
  std::array<double, 100> samples{};
  size_t count = 0;
};

class CallbackTiming
{
  using Clock = std::chrono::steady_clock;
  TimingWindow &window_;
  Clock::time_point start_{};

public:
  explicit CallbackTiming(TimingWindow &window) : window_(window)
  {
    static const bool enabled = [] {
      const char *value = std::getenv("LEGBOT_NAV_TIMINGS");
      return value && std::strcmp(value, "1") == 0;
    }();
    if (enabled) start_ = Clock::now();
  }

  ~CallbackTiming()
  {
    if (start_ == Clock::time_point{}) return;
    auto &samples = window_.samples;
    samples[window_.count++] = std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    if (window_.count < samples.size()) return;
    std::sort(samples.begin(), samples.end());
    auto overdue = samples.end() - std::upper_bound(samples.begin(), samples.end(), window_.budget_ms);
    std::fprintf(stderr, "[nav_timing] %s n=100 p95_ms=%.2f max_ms=%.2f budget_ms=%.1f over_budget=%td\n",
                 window_.name, samples[94], samples.back(), window_.budget_ms, overdue);
    window_.count = 0;
  }
};
}
