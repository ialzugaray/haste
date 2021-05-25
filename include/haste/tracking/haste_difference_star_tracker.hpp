// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
#include "haste_difference_tracker.hpp"

namespace haste {
class HasteDifferenceStarTracker : public HasteDifferenceTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HasteDifferenceStarTracker(const Time &t, const Location &x, const Location &y, const Orientation &theta);

  auto trackerName() const -> std::string override;
  auto initializeHypotheses() -> void override;
  auto updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event) -> void override;

 protected:
  auto updateDifferences_(Eigen::Ref<Patch> difference_patch, const EventTuple &event, const Hypothesis &hypothesis,
                          Scalar &score, const Scalar &increment) -> void;
  std::array<Patch, kNumHypotheses> difference_patches_;
};
}// namespace haste

#include "haste_difference_star_tracker_impl.hpp"