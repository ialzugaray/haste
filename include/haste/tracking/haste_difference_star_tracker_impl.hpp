// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

namespace haste {

HasteDifferenceStarTracker::HasteDifferenceStarTracker(const Time &t, const Location &x, const Location &y,
                                                       const Orientation &theta)
    : HasteDifferenceTracker(t, x, y, theta) {}

auto HasteDifferenceStarTracker::trackerName() const -> std::string { return "HasteDifferenceStarTracker"; };

auto HasteDifferenceStarTracker::initializeHypotheses() -> void {
  for (size_t i = 0; i < kNumHypotheses; ++i) {
    difference_patches_[i] = getDifferencePatch_(hypotheses_[i]);
    hypotheses_score_[i] = -difference_patches_[i].square().sum();
  }
};

auto HasteDifferenceStarTracker::updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event)
    -> void {
  const auto &[t_old, x_old, y_old] = oldest_event;
  const auto &[t_new, x_new, y_new] = newest_event;

  for (size_t i = 0; i < kNumHypotheses; ++i) {
    const auto &hypothesis = hypotheses_[i];
    auto &hypothesis_score = hypotheses_score_[i];
    auto &difference_patch = difference_patches_[i];
    updateDifferences_(difference_patch, newest_event, hypothesis, hypothesis_score, -1.0);
    updateDifferences_(difference_patch, oldest_event, hypothesis, hypothesis_score, +1.0);
  }
}

auto HasteDifferenceStarTracker::updateDifferences_(Eigen::Ref<Patch> difference_patch, const EventTuple &event,
                                                    const Hypothesis &hypothesis, Scalar &score,
                                                    const Scalar &increment) -> void {
  const auto &[et, ex, ey] = event;
  const auto [xp, yp] = patchLocation(ex, ey, hypothesis);

  if (xp >= 0 && yp >= 0 && xp < (kPatchSize - 1) && yp < (kPatchSize - 1)) {
    auto interp_kernel = Interpolator::bilinearKernel(xp, yp);

    Eigen::Ref<Eigen::Array<Scalar, 2, 2>> difference_patch_block = difference_patch.block<2, 2>((int) xp, (int) yp);

    // Remove the contribution of this interpolated locations
    score += difference_patch_block.square().sum();
    // Update the values of the difference patch in the interpolated locations
    difference_patch_block += interp_kernel * increment * kWeight_;
    // Re-apply the contribution of this interpolated locations
    score -= difference_patch_block.square().sum();
  }
}

}// namespace haste
