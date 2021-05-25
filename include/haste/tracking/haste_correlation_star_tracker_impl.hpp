// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

namespace haste {

HasteCorrelationStarTracker::HasteCorrelationStarTracker(const Time &t, const Location &x, const Location &y,
                                                         const Orientation &theta)
    : HypothesisPatchTracker(t, x, y, theta) {}

auto HasteCorrelationStarTracker::trackerName() const -> std::string { return "HasteCorrelationStarTracker"; };

auto HasteCorrelationStarTracker::updateTemplate() -> void { updateTemplateWithMiddleEvent(kWeight_); };

auto HasteCorrelationStarTracker::eventWindowToModel(const EventWindow &event_window,
                                                     const Hypothesis &hypothesis) const -> Patch {
  return eventWindowToModelUnitary(event_window, hypothesis, kWeight_);
};

auto HasteCorrelationStarTracker::initializeHypotheses() -> void {
  template_normalized_ =
      template_ / template_.sum();// TODO: this step could be avoided by accounting a running sum for template.sum();

  for (size_t i = 0; i < kNumHypotheses; ++i) {
    const auto &hypothesis = hypotheses_[i];
    auto &hypothesis_score = hypotheses_score_[i];
    const auto &[xp_vec, yp_vec] = patchLocation(event_window_.ex_vec(), event_window_.ey_vec(), hypothesis);
    auto sampled_value_vec = Interpolator::bilinearSampleVector(template_normalized_, xp_vec, yp_vec);

    hypothesis_score = (kWeight_ * sampled_value_vec).sum();
  }
};

auto HasteCorrelationStarTracker::updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event)
    -> void {
  const auto &[t_old, x_old, y_old] = oldest_event;
  const auto &[t_new, x_new, y_new] = newest_event;

  for (size_t i = 0; i < kNumHypotheses; ++i) {
    const auto &hypothesis = hypotheses_[i];
    auto &hypothesis_score = hypotheses_score_[i];

    const auto &[xp_old, yp_old] = patchLocation(x_old, y_old, hypothesis);
    const auto &[xp_new, yp_new] = patchLocation(x_new, y_new, hypothesis);

    auto sampled_value_old = Interpolator::bilinearSample(template_normalized_, xp_old, yp_old);
    auto sampled_value_new = Interpolator::bilinearSample(template_normalized_, xp_new, yp_new);

    hypothesis_score += kWeight_ * (sampled_value_new - sampled_value_old);
  }
}

}// namespace haste