#ifndef SIRE_TIMER_HPP_
#define SIRE_TIMER_HPP_

#include <thread>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>

namespace sire::core {
class Timer {
  using Clock = std::chrono::high_resolution_clock;
  using Duration = std::chrono::duration<double>;
  using TimePoint = std::chrono::time_point<Clock, Duration>;

 public:
  auto pauseIfTooFast() -> void {
    if (target_realtime_rate_ <= 0) return;  // Run at full speed.
    const double sim_passed = sim_time_;
    const TimePoint desired_realtime =
        begin_time_ + Duration(sim_passed / target_realtime_rate_);
    if (desired_realtime > Clock::now())
      std::this_thread::sleep_until(desired_realtime);
  }
  auto updateSimTime(double dt) -> void { sim_time_ += dt; };
  auto init() -> void { begin_time_ = Clock::now(); }
  auto reset() -> void {
    sim_time_ = 0.0;
    begin_time_ = Clock::now();
  }
  auto setRealtimeRate(double rate) -> void { target_realtime_rate_ = rate; }
  auto realtimeRate() -> double { return target_realtime_rate_; }

 private:
  double target_realtime_rate_{1.0};
  double sim_time_{0.0};
  TimePoint begin_time_;
};
}  // namespace sire::core
#endif