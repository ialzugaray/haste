// Copyright (c) 2020 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

namespace haste {

HypothesisPatchTracker::HypothesisPatchTracker(const Time &t, const Location &x, const Location &y,
                                               const Orientation &theta) {
  hypotheses_[kNullHypothesisIdx] = Hypothesis{t, x, y, theta};
}

auto HypothesisPatchTracker::isEventInRange(const Location &ex, const Location &ey) const -> bool {
  //      auto dx = ex - x_;
  //      auto dy = ey - y_;
  //      auto xp = +dx * ctheta_ + dy * stheta_ + kPatchSizeHalf;
  //      auto yp = -dx * stheta_ + dy * ctheta_ + kPatchSizeHalf;
  // d2 rule
  Location dx = ex - x();
  Location dy = ey - y();
  constexpr auto d2_thresh = kPatchSizeHalf * kPatchSizeHalf;
  return (dx * dx + dy * dy) < d2_thresh;
}

auto HypothesisPatchTracker::patchLocation(const Location &ex, const Location &ey, const Hypothesis &state) const
    -> std::pair<Location, Location> {
  const auto dx = ex - state.x();
  const auto dy = ey - state.y();
  const auto ctheta = state.ctheta();
  const auto stheta = state.stheta();

  auto xp = +dx * ctheta + dy * stheta + kPatchSizeHalf;// Patch coordinates
  auto yp = -dx * stheta + dy * ctheta + kPatchSizeHalf;
  return {xp, yp};
}

template<int N>
auto HypothesisPatchTracker::patchLocation(const LocationVector<N> &ex_vec, const LocationVector<N> &ey_vec,
                                           const Hypothesis &state) const
    -> std::pair<LocationVector<N>, LocationVector<N>> {
  const LocationVector<N> dx = ex_vec - state.x();
  const LocationVector<N> dy = ey_vec - state.y();
  const auto ctheta = state.ctheta();
  const auto stheta = state.stheta();

  LocationVector<N> xp_vec = dx * ctheta + dy * stheta + kPatchSizeHalf;// Patch coordinates
  LocationVector<N> yp_vec = -dx * stheta + dy * ctheta + kPatchSizeHalf;

  // TODO: verify the return type does not harm Eigen alignment
  return {xp_vec, yp_vec};
}

auto HypothesisPatchTracker::updateTemplateWithMiddleEvent(const Weight &weight) -> void {
  const auto &[et, ex, ey] = event_window_.middleEvent();
  const auto [xp, yp] = patchLocation(ex, ey, state());
  Interpolator::bilinearIncrementVector(template_, xp, yp, weight * kTemplateUpdateFactor);
}

auto HypothesisPatchTracker::eventWindowToModelUnitary(const EventWindow &event_window, const Hypothesis &hypothesis,
                                                       const Weight &weight) const -> Patch {
  Patch model = Patch::Zero();
  EventWindowLocationVector ex_vec = event_window.ex_vec();
  EventWindowLocationVector ey_vec = event_window.ey_vec();
  const auto [xp_vec, yp_vec] = patchLocation(ex_vec, ey_vec, hypothesis);

  for (size_t i = 0; i < kEventWindowSize; ++i) {// TODO(ialzugaray): "vectorizable" loop with Eigen binaryExpr
    const Location &xp = xp_vec[i];
    const Location &yp = yp_vec[i];
    Interpolator::bilinearIncrementVector(model, xp, yp, weight);
  }
  return model;
}

auto HypothesisPatchTracker::eventWindowToModelVector(const EventWindow &event_window, const Hypothesis &hypothesis,
                                                      const EventWindowVector<Weight> &weights) const -> Patch {
  Patch model = Patch::Zero();
  EventWindowLocationVector ex_vec = event_window.ex_vec();
  EventWindowLocationVector ey_vec = event_window.ey_vec();

  auto [xp_vec, yp_vec] = patchLocation(ex_vec, ey_vec, hypothesis);

  for (size_t i = 0; i < kEventWindowSize; ++i) {// TODO(ialzugaray): "vectorizable" loop with Eigen binaryExpr
    const Location &xp = xp_vec[i];
    const Location &yp = yp_vec[i];
    const Weight &weight = weights[i];
    Interpolator::bilinearIncrementVector(model, xp, yp, weight);
  }
  return model;
}

auto HypothesisPatchTracker::initializeTracker() -> void {
  status_ = TrackerStatus::kRunning;
  const auto &[et, ex, ey] = event_window_.middleEvent();
  Hypothesis initial_hypothesis{et, x(), y(), theta()};
  template_ = eventWindowToModel(event_window_, initial_hypothesis);
  transitionToHypothesis(initial_hypothesis);
}

auto HypothesisPatchTracker::appendEventToWindow(const EventTuple &newest_event) -> EventTuple {
  const auto oldest_event = event_window_.appendEvent(newest_event);
  event_counter_++;
  return oldest_event;
}

auto HypothesisPatchTracker::updateHypothesesTimeFromMiddleEvent() {
  auto [et_mid, ex_mid, ey_mid] = event_window_.middleEvent();
  for (auto &hypothesis : hypotheses_) { hypothesis.t() = et_mid; }
}
auto HypothesisPatchTracker::pushEvent(const Time &et, const Location &ex, const Location &ey) -> EventUpdate {
  if (//(et <= t()) ||
      (!isEventInRange(ex, ey))) {
    return kOutOfRange;
  }

  EventTuple newest_event{et, ex, ey};
  const auto oldest_event = appendEventToWindow(newest_event);

  if (status_ == kUninitialized) {
    if (event_counter_ >= kEventWindowSize) {// TODO: verify >= range
      initializeTracker();
      return kStateEvent;
    } else {
      return kInitializingEvent;
    }
  }

  updateHypothesesTimeFromMiddleEvent();// TODO: Not relevant until change of state;

  updateHypothesesScore(oldest_event, newest_event);

  auto best_hypothesis_idx = getBestHypothesisIdx();
  EventUpdate ret;
  if (best_hypothesis_idx == kNullHypothesisIdx) {
    ret = EventUpdate::kRegularEvent;
  } else {
    ret = EventUpdate::kStateEvent;
    transitionToHypothesis(hypotheses_[best_hypothesis_idx]);
  }

  updateTemplate();
  return ret;
}

auto HypothesisPatchTracker::getBestHypothesisIdx() const -> size_t {
  const auto &null_hypothesis_score = hypotheses_score_[kNullHypothesisIdx];
  size_t best_hypothesis_idx;
  auto best_hypothesis_score = hypotheses_score_.maxCoeff(&best_hypothesis_idx);
  // TODO: Compute normalized versions only if null < best
  auto worst_hypothesis_score = hypotheses_score_.minCoeff();
  const auto best_hypothesis_score_normalized =
      (best_hypothesis_score - worst_hypothesis_score) / (best_hypothesis_score - worst_hypothesis_score);
  const auto null_hypothesis_score_normalized =
      (null_hypothesis_score - worst_hypothesis_score) / (best_hypothesis_score - worst_hypothesis_score);

  if ((null_hypothesis_score < best_hypothesis_score)
      && ((best_hypothesis_score_normalized - null_hypothesis_score_normalized) > kHysteresisFactor)) {
    return best_hypothesis_idx;
  } else {
    return kNullHypothesisIdx;
  }
}

auto HypothesisPatchTracker::transitionToHypothesis(const Hypothesis &hypothesis) -> void {
  hypotheses_ = HypothesesGenerator::GenerateCenteredHypotheses(hypothesis);// Renew hypotheses
  initializeHypotheses();
}

}// namespace haste
