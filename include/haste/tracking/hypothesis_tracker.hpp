// Copyright (c) 2020 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

#include <glog/logging.h>

#include "haste/core/event_window.hpp"
#include "haste/core/hypotheses_manager.hpp"
#include "haste/core/interpolator.hpp"
#include "haste/core/patch.hpp"

namespace haste {

//template<typename Scalar_>
class HypothesisPatchTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Note that Scalar can be easily set to double but this way we spare a significant amount of computation.
  // TODO: Known issue: using Scalar=float, this extends to Hypothesis<float> and the tracker state, so that the high
  //  resolution in event timestamps with the large raw numbers (as in microseconds in Unix time) cannot be captured.
  //  As a results, all the resulting states in the tracking would have the same timestamp.
  //  Please rebase the timestamp of the event stream with respect to the first event or set Scalar = double
  //  but then you should a expect a poor(er) computational performance (between 10% and 25% worse).
  using Scalar = float; // TODO: it could be templated.
  using HypothesesGenerator = CenteredHypothesesGenerator<IncrementalHypothesesGenerator_TXYR_8neigh_2rot<Scalar>>;

  static constexpr auto kNullHypothesisIdx = HypothesesGenerator::kNullHypothesisIdx;
  static constexpr auto kNumHypotheses = HypothesesGenerator::kNumHypotheses;
  using Hypothesis = typename HypothesesGenerator::Hypothesis;

  using Time = typename Hypothesis::Time;
  using Location = typename Hypothesis::Location;
  using Orientation = typename Hypothesis::Orientation;
  using Weight = Scalar;

  static constexpr Scalar kTextureFactor = 0.2;
  static constexpr Scalar kTemplateUpdateFactor = 0.1;
  static constexpr size_t kPatchSize = 31;
  static_assert(kPatchSize % 2 == 1, "Patch kSize parameter must be an odd");
  static constexpr size_t kPatchSizeHalf = (kPatchSize - 1) / 2;
  static constexpr size_t kEventWindowSize = 1 + 2 * size_t(kTextureFactor * kPatchSize * kPatchSize / 2);

  using Hypotheses = typename HypothesesGenerator::Hypotheses;
  using HypothesesScore = Eigen::Array<Scalar, kNumHypotheses, 1>;
  static constexpr auto kHysteresisFactor = 0.05;

  using EventWindow = FixedSizeLocationEventWindowType<EventTraitType<Scalar>, kEventWindowSize>;
  using EventWindowLocationVector = typename EventWindow::LocationVector;
  using EventTuple = typename EventWindow::EventTuple;

  enum EventUpdate { kOutOfRange, kInitializingEvent, kRegularEvent, kStateEvent };
  enum TrackerStatus { kUninitialized, kRunning };

  using Interpolator = InterpolatorType<Scalar, Location>;
  using Patch = Eigen::Array<Scalar, kPatchSize, kPatchSize>;
  using PatchVec = Eigen::Array<Scalar, kPatchSize * kPatchSize, 1>;
  using PatchConstMap = Eigen::Map<const Patch>;
  using PatchMap = Eigen::Map<Patch>;

  template<size_t N>
  using PatchStack = Eigen::Array<Scalar, kPatchSize * kPatchSize, N>;

  template<typename T>
  using EventWindowVector = typename EventWindow::template EventWindowVector<T>;
  template<typename T, size_t N>
  using EventWindowStack = Eigen::Array<T, N, kEventWindowSize>;
  template<size_t N>
  using LocationVector = Eigen::Array<Location, N, 1>;

  HypothesisPatchTracker(const Time &t, const Location &x, const Location &y, const Orientation &theta);

  template<int N>
  auto patchLocation(const LocationVector<N> &ex_vec, const LocationVector<N> &ey_vec, const Hypothesis &state) const
      -> std::pair<LocationVector<N>, LocationVector<N>>;

  auto isEventInRange(const Location &ex, const Location &ey) const -> bool;
  auto patchLocation(const Location &ex, const Location &ey, const Hypothesis &state) const
      -> std::pair<Location, Location>;
  auto updateTemplateWithMiddleEvent(const Weight &weight) -> void;
  auto eventWindowToModelUnitary(const EventWindow &event_window, const Hypothesis &hypothesis,
                                 const Weight &weight = 1.0) const -> Patch;
  auto eventWindowToModelVector(const EventWindow &event_window, const Hypothesis &hypothesis,
                                const EventWindowVector<Weight> &weights) const -> Patch;
  auto initializeTracker() -> void;
  auto updateHypothesesTimeFromMiddleEvent();
  auto pushEvent(const Time &et, const Location &ex, const Location &ey) -> EventUpdate;
  auto getBestHypothesisIdx() const -> size_t;
  auto transitionToHypothesis(const Hypothesis &hypothesis) -> void;

  virtual auto appendEventToWindow(const EventTuple &newest_event) -> EventTuple;
  virtual auto trackerName() const -> std::string = 0;
  virtual auto updateTemplate() -> void = 0;
  virtual auto initializeHypotheses() -> void = 0;
  virtual auto updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event) -> void = 0;
  virtual auto eventWindowToModel(const EventWindow &event_window, const Hypothesis &hypothesis) const -> Patch = 0;

  auto state() const -> const Hypothesis & { return hypotheses_[kNullHypothesisIdx]; }
  auto t() const -> const Time & { return state().t(); }
  auto x() const -> const Location & { return state().x(); }
  auto y() const -> const Location & { return state().y(); }
  auto theta() const -> const Orientation & { return state().theta(); }
  auto event_window() const -> EventWindow { return event_window_; }
  auto tracker_template() const -> const Patch & { return template_; }
  auto event_counter() const -> const size_t & { return event_counter_; }
  auto status() const -> const TrackerStatus & { return status_; }

 protected:
  TrackerStatus status_ = kUninitialized;
  Hypotheses hypotheses_;
  HypothesesScore hypotheses_score_;

  Patch template_;
  EventWindow event_window_;

  // Counters
  size_t event_counter_ = 0;
  size_t state_counter_ = 0;
};

}// namespace haste

#include "hypothesis_tracker_impl.hpp"