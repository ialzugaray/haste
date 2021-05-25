// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

namespace haste {

CorrelationTracker::CorrelationTracker(const Time &t, const Location &x, const Location &y,
                                                   const Orientation &theta)
    : HypothesisPatchTracker(t, x, y, theta) {
  setGaussianWeight_();
}

auto CorrelationTracker::trackerName() const -> std::string { return "CorrelationTracker"; };

auto CorrelationTracker::updateTemplate() -> void {
  updateTemplateWithMiddleEvent(weights_[EventWindow::kMiddleEventIdx]);
};

auto CorrelationTracker::eventWindowToModel(const EventWindow &event_window, const Hypothesis &hypothesis) const
    -> Patch {
  return eventWindowToModelVector(event_window, hypothesis, weights_);
};

auto CorrelationTracker::initializeHypotheses() -> void {
  for (size_t i = 0; i < kNumHypotheses; ++i) { hypotheses_score_[i] = getHypothesisScore_(hypotheses_[i]); }
};

auto CorrelationTracker::updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event)
    -> void {
  // Ignore newest and oldest event and proceed from scratch
  for (size_t i = 0; i < kNumHypotheses; ++i) { hypotheses_score_[i] = getHypothesisScore_(hypotheses_[i]); }
};

auto CorrelationTracker::getHypothesisScore_(const Hypothesis &hypothesis) const -> Scalar {
  const auto &[xp_vec, yp_vec] = patchLocation(event_window_.ex_vec(), event_window_.ey_vec(), hypothesis);
  auto sampled_value_vec = Interpolator::bilinearSampleVector(template_, xp_vec, yp_vec);
  return (weights_ * sampled_value_vec).sum();
}

auto CorrelationTracker::setGaussianWeight_() -> void {
  constexpr auto kEventWindowSizeHalf = (kEventWindowSize - 1) / 2;
  constexpr auto sigma = (kEventWindowSize / 6.0);
  constexpr auto sigma2 = sigma * sigma;
  constexpr auto sigma2_inv = 1.0 / sigma2;
  weights_ =
      Eigen::exp(-0.5 * sigma2_inv
                 * Eigen::square(Eigen::Array<Weight, -1, 1>::LinSpaced(kEventWindowSize, 0, kEventWindowSize - 1)
                                 - kEventWindowSizeHalf))
          .array();
  weights_ = weights_ / weights_.sum();
}
}// namespace haste