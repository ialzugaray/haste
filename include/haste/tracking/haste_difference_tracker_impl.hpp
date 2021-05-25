// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
namespace haste {

HasteDifferenceTracker::HasteDifferenceTracker(const Time &t, const Location &x, const Location &y,
                                                           const Orientation &theta)
    : HypothesisPatchTracker(t, x, y, theta) {}

auto HasteDifferenceTracker::trackerName() const -> std::string { return "HasteDifferenceTracker"; };

auto HasteDifferenceTracker::updateTemplate() -> void {
  updateTemplateWithMiddleEvent(kWeight_);
};

auto HasteDifferenceTracker::eventWindowToModel(const EventWindow &event_window,
                                                      const Hypothesis &hypothesis) const -> Patch {
  return eventWindowToModelUnitary(event_window, hypothesis, kWeight_);
}

auto HasteDifferenceTracker::initializeHypotheses() -> void {
  for (size_t i = 0; i < kNumHypotheses; ++i) {
    hypotheses_score_[i] = -getDifferencePatch_(hypotheses_[i]).square().sum();
  }
}

auto HasteDifferenceTracker::updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event)
    -> void {
  const auto &[t_old, x_old, y_old] = oldest_event;
  const auto &[t_new, x_new, y_new] = newest_event;

  for (size_t i = 0; i < kNumHypotheses; ++i) {
    hypotheses_score_[i] = -getDifferencePatch_(hypotheses_[i]).square().sum();
  }
}

auto HasteDifferenceTracker::getDifferencePatch_(const Hypothesis &hypothesis) -> Patch {
  const auto hypothesis_model = eventWindowToModel(event_window_, hypothesis);
  Patch difference_patch = template_ / template_.sum() - hypothesis_model / (kEventWindowSize * kWeight_);
  return difference_patch;
}

}// namespace haste