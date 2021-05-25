// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

namespace haste {
HasteCorrelationTracker::HasteCorrelationTracker(const Time &t, const Location &x, const Location &y,
                                                 const Orientation &theta)
    : CorrelationTracker(t, x, y, theta) {}

auto HasteCorrelationTracker::trackerName() const -> std::string { return "HasteCorrelationTracker"; };

auto HasteCorrelationTracker::appendEventToWindow(const EventTuple &newest_event) -> EventTuple {
  const auto oldest_event = event_window_.appendEvent(newest_event);

  // TODO: subs with an actual rolling vector efficiency
  samples_stack_.leftCols<kEventWindowSize - 1>() = samples_stack_.rightCols<kEventWindowSize - 1>();

  const auto &[et, ex, ey] = newest_event;
  for (size_t i = 0; i < kNumHypotheses; ++i) {
    const auto &hypothesis = hypotheses_[i];
    const auto &[xp, yp] = patchLocation(ex, ey, hypothesis);
    const auto &sampled_value = Interpolator::bilinearSample(template_, xp, yp);
    samples_stack_(i, kEventWindowSize - 1) = sampled_value;
  }

  event_counter_++;
  return oldest_event;
}

auto HasteCorrelationTracker::initializeHypotheses() -> void {
  for (size_t i = 0; i < kNumHypotheses; ++i) {
    const auto &hypothesis = hypotheses_[i];
    const auto &[xp_vec, yp_vec] = patchLocation(event_window_.ex_vec(), event_window_.ey_vec(), hypothesis);
    const auto &sampled_values = Interpolator ::bilinearSampleVector(template_, xp_vec, yp_vec);
    samples_stack_.row(i) = sampled_values;
  }

  hypotheses_score_ = (samples_stack_.matrix() * weights_.matrix()).array();
};

auto HasteCorrelationTracker::updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event)
    -> void {
  // Ignore newest and oldest event and proceed from scratch
  hypotheses_score_ = (samples_stack_.matrix() * weights_.matrix()).array();
};

}// namespace haste