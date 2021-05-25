// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

#include "haste/tracking/correlation_tracker.hpp"
namespace haste {
class HasteCorrelationTracker : public CorrelationTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HasteCorrelationTracker(const Time &t, const Location &x, const Location &y, const Orientation &theta);
  auto trackerName() const -> std::string override;
  auto appendEventToWindow(const EventTuple &newest_event) -> EventTuple override;
  auto initializeHypotheses() -> void override;
  auto updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event) -> void override;

 protected:
  EventWindowStack<Scalar, kNumHypotheses> samples_stack_;
};
}// namespace haste

#include "haste/tracking/haste_correlation_tracker_impl.hpp"