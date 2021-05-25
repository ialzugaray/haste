// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

#include "haste/tracking/hypothesis_tracker.hpp"

namespace haste {
class CorrelationTracker : public HypothesisPatchTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CorrelationTracker(const Time &t, const Location &x, const Location &y, const Orientation &theta);

  auto trackerName() const -> std::string override;
  auto updateTemplate() -> void override;
  auto eventWindowToModel(const EventWindow &event_window, const Hypothesis &hypothesis) const -> Patch override;
  auto initializeHypotheses() -> void override;
  auto updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event) -> void override;

 protected:
  auto getHypothesisScore_(const Hypothesis &hypothesis) const -> Scalar;
  auto setGaussianWeight_() -> void;
  EventWindowVector<Weight> weights_;
};
}// namespace haste

#include "correlation_tracker_impl.hpp"