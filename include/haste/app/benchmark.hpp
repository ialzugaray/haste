// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
#include <chrono>
#include <sstream>

namespace haste {
namespace app {
namespace benchmark {

class TimingBenchmark {
 public:
  using Time = uint64_t;

  auto registerState(const Time& time) {
    timing.state_sum += time;
    ++counters.state;
  }

  auto registerRegular(const Time& time) {
    timing.regular_sum += time;
    ++counters.regular;
  }
  auto averageTimeRegular() const -> Time { return counters.regular ? timing.regular_sum / counters.regular : 0; }
  auto averageTimeState() const -> Time { return counters.state ? timing.state_sum / counters.state : 0; }
  auto averageTime() const -> Time {
    return (counters.state + counters.regular)
        ? (timing.state_sum + timing.regular_sum) / (counters.state + counters.regular)
        : 0;
  }
  auto percentageRegular() const -> double {
    return (counters.state + counters.regular) ? 100.0 * double(counters.regular) / (counters.state + counters.regular)
                                               : 0;
  }
  auto percentageState() const -> double {
    return (counters.state + counters.regular) ? 100.0 * double(counters.state) / (counters.state + counters.regular)
                                               : 0;
  }

  auto fullReport() -> std::string {
    std::ostringstream is;
    is << "\nBenchmark results: \n"
       << "Regular Events: \t" << percentageRegular() << "%  \t | \t " << averageTimeRegular() << " ns/event \n"
       << "State Events:   \t" << percentageState() << "%  \t | \t " << averageTimeState() << " ns/event \n"
       << "---------------------------------- \n"
       << "Total:          \t" << 100.0 << "% \t | \t " << averageTime() << " ns/event \n";

    return is.str();
  }

 private:
  struct Timings {
    Time regular_sum = 0;
    Time state_sum = 0;
  } timing;

  struct Counters {
    size_t regular = 0;
    size_t state = 0;
  } counters;
};

class Stopwatch {
 public:
  using Clock = std::chrono::high_resolution_clock;
  using Time = uint64_t;
  Stopwatch() : start_point_(Clock::now()) {}
  auto tic() -> void { start_point_ = Clock::now(); }
  auto toc() const -> Time {
    //    std::atomic_thread_fence(std::memory_order_relaxed);
    auto counted_time = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start_point_).count();
    //    std::atomic_thread_fence(std::memory_order_relaxed);
    return static_cast<Time>(counted_time);
  }

 protected:
  Clock::time_point start_point_;
};

}// namespace benchmark
}// namespace app
}// namespace haste