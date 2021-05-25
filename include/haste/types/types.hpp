// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
#include <cmath>

template<typename Scalar_, size_t kSize_>
class Hypothesis {
 public:
  using Scalar = Scalar_;
  static constexpr auto kSize = kSize_;

  Hypothesis() = delete;

  template<typename... Args>
  constexpr Hypothesis(Args... args) : data_{args...} {}
  constexpr Hypothesis(const Hypothesis<Scalar, kSize>& state) : data_{state.data_} {}

 protected:
  using Data_ = std::array<Scalar, kSize>;
  Data_ data_;
};

template<typename Scalar_>
class HypothesisXYR : public Hypothesis<Scalar_, 5> {
  using Base = Hypothesis<Scalar_, 5>;
  using Base::data_;

 public:
  using typename Base::Scalar;

  constexpr HypothesisXYR(const Scalar& x, const Scalar& y, const Scalar& theta)
      : Base{x, y, theta, std::cos(theta), std::sin(theta)} {}

  constexpr HypothesisXYR(const HypothesisXYR& other) : Base{other.data_} {}

  constexpr auto operator+(const HypothesisXYR& other) const -> HypothesisXYR {
    return {this->x() + other.x(), this->y() + other.y(), this->theta() + other.theta()};
  }

  auto x() const -> const Scalar& { return data_[0]; }
  auto y() const -> const Scalar& { return data_[1]; }
  auto theta() const -> const Scalar& { return data_[2]; }
  auto ctheta() const -> const Scalar& { return data_[3]; }
  auto stheta() const -> const Scalar& { return data_[4]; }
};

template<typename Hypothesis_, typename Time_ = double>
class HypothesisState {
 public:
  using Time = Time_;
  using Hypothesis = Hypothesis_;
  using Scalar = typename Hypothesis::Scalar;

  template<typename... Args>
  constexpr HypothesisState(const Time& t, Args... state_args) : t_{t},
                                                                 hypothesis_{state_args...} {};
  constexpr HypothesisState(const HypothesisState& other) : t_{other.t_}, hypothesis_{other.hypothesis_} {};
  constexpr auto operator+(const Hypothesis& other_state) -> HypothesisState<Hypothesis, Time> {
    return {this->t_, this->hypothesis_ + other_state};
  }

  auto t() const -> const Time& { return t_; }
  auto hypothesis() const -> const Hypothesis& { return hypothesis_; }

 protected:
  Time t_;
  Hypothesis hypothesis_;
};

template<typename Scalar_>
struct HypothesesGenerator_XYR_8neigh_2rot {
  using Hypothesis = HypothesisXYR<Scalar_>;
  using Scalar = typename Hypothesis::Scalar;

  static constexpr Scalar kDeltaX = 1.0f;
  static constexpr Scalar kDeltaY = 1.0f;
  static constexpr Scalar kDeltaTheta = 4.0 * M_PI / 180.0;

  // clang-format off
  static constexpr size_t kNullHypothesisIdx = 0;
  static constexpr std::array kIncrementalHypotheses{
      Hypothesis{+0.0, +0.0, +0.0},
      Hypothesis{+kDeltaX, +0.0, +0.0},
      Hypothesis{-kDeltaX, +0.0, +0.0},
      Hypothesis{+0.0, +kDeltaY, +0.0},
      Hypothesis{+0.0, -kDeltaY, +0.0},
      Hypothesis{+kDeltaX, +kDeltaY, +0.0},
      Hypothesis{-kDeltaX, +kDeltaY, +0.0},
      Hypothesis{-kDeltaX, -kDeltaY, +0.0},
      Hypothesis{+kDeltaX, -kDeltaY, +0.0},
      Hypothesis{+0.0, +0.0, +kDeltaTheta},
      Hypothesis{+0.0, +0.0, -kDeltaTheta}};
  static constexpr auto kNumHypotheses = kIncrementalHypotheses.size();
  // clang-format on

  static auto getCenteredHypotheses(const Hypothesis& center_hypothesis) -> std::array<Hypothesis,kNumHypotheses> {
    std::array<Hypothesis,kNumHypotheses> hypotheses;
    for (size_t i = 0; i<kNumHypotheses ;++i) { hypotheses[i] =  kIncrementalHypotheses[i] + center_hypothesis; }
    return hypotheses;
  }

};

//
//
//template <typename Event, typename State_>
//class EventStateManager{
// public:
//  using Event = Event;
//  using State = State_;
//
//  EventStateManager(const State& state) : state_{state}{}
//
//  virtual auto AddEvent(const Event& event)-> void;
//  auto state() const -> const State& {return state_;}
// protected:
//  State state_;
//};
//
//template <typename Event, typename HypothesisGenerator_>
//class IncrementalHypothesisTracker : public EventStateManager<Event,HypothesisState<typename HypothesisGenerator_::Hypothesis>>{
//  using Base = EventStateManager<Event,HypothesisGenerator_>;
//  using Base::Event;
//  using Base::State
//
//  IncrementalHypothesisTracker(const State& state) : state_{state}{}
//  virtual auto AddEvent(const Event& event)-> void;
//  auto state() const -> const State& {return state_;}
// protected:
//  State state_;
//};
//


////template<typename Scalar_>
//class HypothesisPatchTracker {
// public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//  using Scalar = double;
//  using HypothesesGenerator = CenteredHypothesesGenerator<IncrementalHypothesesGenerator_TXYR_8neigh_2rot<Scalar>>;
//  static constexpr auto kNullHypothesisIdx = HypothesesGenerator::kNullHypothesisIdx;
//  static constexpr auto kNumHypotheses = HypothesesGenerator::kNumHypotheses;
//  using Hypothesis = typename HypothesesGenerator::Hypothesis;
//
//  using Time = typename Hypothesis::Time;
//  using Location = typename Hypothesis::Location;
//  using Orientation = typename Hypothesis::Orientation;
//  using Weight = Scalar;
//
//  static constexpr size_t kPatchSize = 31;
//  static_assert(kPatchSize % 2 == 1, "Patch kSize parameter must be an odd");
//  static constexpr size_t kPatchSizeHalf = (kPatchSize - 1) / 2;
//  static constexpr Scalar kTextureFactor = 0.2;
//  static constexpr size_t kEventWindowSize = 1 + 2 * size_t(kTextureFactor * kPatchSize * kPatchSize / 2);
//  static constexpr Scalar kTemplateUpdateFactor = 0.1;
//
//  using Hypotheses = typename HypothesesGenerator::Hypotheses;
//  using HypothesesScore = Eigen::Array<Scalar, kNumHypotheses, 1>;
//  static constexpr auto kHysteresisFactor = 0.05;
//
//  using EventWindow = FixedSizeLocationEventWindowType<Scalar, kEventWindowSize>;
//  using EventWindowLocationVector = typename EventWindow::LocationVector;
//  using EventTuple = typename EventWindow::EventTuple;
//
//  enum EventUpdate { kOutOfRange, kInitializingEvent, kRegularEvent, kStateEvent };
//  enum TrackerStatus { kUninitialized, kRunning };
//
//  //    using Patch = PatchType<Scalar, Location, kPatchSize>;
//  using Patch = Eigen::Array<Scalar, kPatchSize, kPatchSize>;
//  using PatchVec = Eigen::Array<Scalar, kPatchSize * kPatchSize, 1>;
//  using PatchConstMap = Eigen::Map<const Patch>;
//  using PatchMap = Eigen::Map<Patch>;
//
//  template<size_t N>
//  using PatchStack = Eigen::Array<Scalar, kPatchSize * kPatchSize, N>;
//
//  template<typename LocationEvent_>
//  using EventWindowVector = typename EventWindow::template EventWindowVector<LocationEvent_>;
//  template<typename LocationEvent_, size_t N>
//  using EventWindowStack = Eigen::Array<LocationEvent_, N, kEventWindowSize>;
//
//  HypothesisPatchTracker(const Time &t, const Location &x, const Location &y, const Orientation &theta) {
//    hypotheses_[kNullHypothesisIdx] = Hypothesis{t, x, y, theta};
//  }
//  const Hypothesis &state() const { return hypotheses_[kNullHypothesisIdx]; }
//  const Time &t() const { return state().t(); }
//  const Location &x() const { return state().x(); }
//  const Location &y() const { return state().y(); }
//  const Orientation &theta() const { return state().theta(); }
//  EventWindow event_window() const { return event_window_; }
//  Patch tracker_template() const { return template_; }
//
//  auto isEventInRange(const Location &ex, const Location &ey) const -> bool {
//    //      auto dx = ex - x_;
//    //      auto dy = ey - y_;
//    //      auto xp = +dx * ctheta_ + dy * stheta_ + kPatchSizeHalf;
//    //      auto yp = -dx * stheta_ + dy * ctheta_ + kPatchSizeHalf;
//    // d2 rule
//    Location dx = ex - x();
//    Location dy = ey - y();
//    constexpr auto d2_thresh = kPatchSizeHalf * kPatchSizeHalf;
//    return (dx * dx + dy * dy) < d2_thresh;
//  }
//
//  auto PatchLocation(const Location &ex, const Location &ey, const Hypothesis &state) const
//  -> std::pair<Location, Location> {
//    const auto dx = ex - state.x();
//    const auto dy = ey - state.y();
//    const auto ctheta = state.ctheta();
//    const auto stheta = state.stheta();
//
//    auto xp = +dx * ctheta + dy * stheta + kPatchSizeHalf;// Patch coordinates
//    auto yp = -dx * stheta + dy * ctheta + kPatchSizeHalf;
//    return {xp, yp};
//  }
//
//  template<size_t N>
//  using LocationVector = Eigen::Array<Location, N, 1>;
//
//  template<int N>
//  auto PatchLocation(const LocationVector<N> &ex_vec, const LocationVector<N> &ey_vec, const Hypothesis &state) const
//  -> std::pair<LocationVector<N>, LocationVector<N>> {
//    const LocationVector<N> dx = ex_vec - state.x();
//    const LocationVector<N> dy = ey_vec - state.y();
//    const auto ctheta = state.ctheta();
//    const auto stheta = state.stheta();
//
//    LocationVector<N> xp_vec = dx * ctheta + dy * stheta + kPatchSizeHalf;// Patch coordinates
//    LocationVector<N> yp_vec = -dx * stheta + dy * ctheta + kPatchSizeHalf;
//
//    // TODO: verify the return type does not harm Eigen alignment
//    return {xp_vec, yp_vec};
//  }
//
//  auto UpdateTemplateWithMiddleEvent(const Weight &weight) -> void {
//    const auto &[et, ex, ey] = event_window_.middleEvent();
//    const auto &[xp, yp] = PatchLocation(ex, ey, state());
//    BilinearIncrementArray2DVector(template_, xp, yp, weight * kTemplateUpdateFactor);
//  }
//
//  Patch EventWindowToModelUnitary(const EventWindow &event_window, const Hypothesis &hypothesis,
//                                  const Weight &weight = 1.0) const {
//    Patch model = Patch::Zero();
//    EventWindowLocationVector ex_vec = event_window.ex_vec();
//    EventWindowLocationVector ey_vec = event_window.ey_vec();
//    const auto &[xp_vec, yp_vec] = PatchLocation(ex_vec, ey_vec, hypothesis);
//
//    for (size_t i = 0; i < kEventWindowSize; ++i) {// TODO(ialzugaray): "vectorizable" loop with Eigen binaryExpr
//      const Location &xp = xp_vec[i];
//      const Location &yp = yp_vec[i];
//      BilinearIncrementArray2DVector(model, xp, yp, weight);
//    }
//    return model;
//  }
//
//  Patch EventWindowToModelVector(const EventWindow &event_window, const Hypothesis &hypothesis,
//                                 const EventWindowVector<Weight> &weights) const {
//    Patch model = Patch::Zero();
//    EventWindowLocationVector ex_vec = event_window.ex_vec();
//    EventWindowLocationVector ey_vec = event_window.ey_vec();
//
//    auto [xp_vec, yp_vec] = PatchLocation(ex_vec, ey_vec, hypothesis);
//
//    for (size_t i = 0; i < kEventWindowSize; ++i) {// TODO(ialzugaray): "vectorizable" loop with Eigen binaryExpr
//      const Location &xp = xp_vec[i];
//      const Location &yp = yp_vec[i];
//      const Weight &weight = weights[i];
//      BilinearIncrementArray2DVector(model, xp, yp, weight);
//    }
//    return model;
//  }
//
//  auto initializeTracker() -> void {
//    status_ = TrackerStatus::kRunning;
//    const auto &[et, ex, ey] = event_window_.middleEvent();
//    Hypothesis initial_hypothesis{et, x(), y(), theta()};
//    template_ = eventWindowToModel(event_window_, initial_hypothesis);
//    transitionToHypothesis(initial_hypothesis);
//  }
//
//  virtual auto appendEventToWindow(const EventTuple &newest_event) -> EventTuple {
//    const auto oldest_event = event_window_.appendEvent(newest_event);
//    event_counter_++;
//    return oldest_event;
//  }
//
//  auto updateHypothesesTimeFromMiddleEvent() {
//    auto [et_mid, ex_mid, ey_mid] = event_window_.middleEvent();
//    for (auto &hypothesis : hypotheses_) { hypothesis.t() = et_mid; }
//  }
//  auto pushEvent(const Time &et, const Location &ex, const Location &ey) -> EventUpdate {
//    if ((et <= t()) || (!isEventInRange(ex, ey))) { return kOutOfRange; }
//
//    EventTuple newest_event{et, ex, ey};
//    const auto oldest_event = appendEventToWindow(newest_event);
//
//    if (status_ == kUninitialized) {
//      if (event_counter_ >= kEventWindowSize) { // TODO: verify >= range
//        initializeTracker();
//        return kStateEvent;
//      } else {
//        return kInitializingEvent;
//      }
//    }
//
//    updateHypothesesTimeFromMiddleEvent();// TODO: Not relevant until change of state;
//
//    updateHypothesesScore(oldest_event, newest_event);
//
//    auto best_hypothesis_idx = getBestHypothesisIdx();
//    EventUpdate ret;
//    if (best_hypothesis_idx == kNullHypothesisIdx) {
//      ret = EventUpdate::kRegularEvent;
//    } else {
//      ret = EventUpdate::kStateEvent;
//      transitionToHypothesis(hypotheses_[best_hypothesis_idx]);
//    }
//
//    updateTemplate();
//    return ret;
//  }
//
//  auto getBestHypothesisIdx() -> size_t {
//    const auto &null_hypothesis_score = hypotheses_score_[kNullHypothesisIdx];
//    size_t best_hypothesis_idx;
//    auto best_hypothesis_score = hypotheses_score_.maxCoeff(&best_hypothesis_idx);
//    // TODO: Compute normalized versions only if null < best
//    auto worst_hypothesis_score = hypotheses_score_.minCoeff();
//    const auto best_hypothesis_score_normalized =
//        (best_hypothesis_score - worst_hypothesis_score) / (best_hypothesis_score - worst_hypothesis_score);
//    const auto null_hypothesis_score_normalized =
//        (null_hypothesis_score - worst_hypothesis_score) / (best_hypothesis_score - worst_hypothesis_score);
//
//    if ((null_hypothesis_score < best_hypothesis_score)
//        && ((best_hypothesis_score_normalized - null_hypothesis_score_normalized) > kHysteresisFactor)) {
//      return best_hypothesis_idx;
//    } else {
//      return kNullHypothesisIdx;
//    }
//  }
//
//  auto transitionToHypothesis(const Hypothesis &hypothesis) -> void {
//    hypotheses_ = HypothesesGenerator::GenerateCenteredHypotheses(hypothesis);// Renew hypotheses
//    initializeHypotheses();
//  }
//
//  virtual auto trackerName() const -> std::string = 0;
//  virtual auto updateTemplate() -> void = 0;
//  virtual auto initializeHypotheses() -> void = 0;
//  virtual auto updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event) -> void = 0;
//  virtual auto eventWindowToModel(const EventWindow &event_window, const Hypothesis &hypothesis) const -> Patch = 0;
//
//  // protected:
//  TrackerStatus status_ = kUninitialized;
//  Hypotheses hypotheses_;
//  HypothesesScore hypotheses_score_;
//
//  Patch template_;
//  EventWindow event_window_;
//
//  // Counters
//  size_t event_counter_ = 0;
//  size_t state_counter_ = 0;
//};
//
//class CorrelationTracker : public HypothesisPatchTracker {
// public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//  EventWindowVector<Weight> weights_;
//
//  CorrelationTracker(const Time &t, const Location &x, const Location &y, const Orientation &theta)
//      : HypothesisPatchTracker(t, x, y, theta) {
//    SetGaussianWeight();
//  }
//
//  virtual auto trackerName() const -> std::string override { return "CorrelationTracker"; };
//
//  virtual auto updateTemplate() -> void override {
//    UpdateTemplateWithMiddleEvent(weights_[EventWindow::kMiddleEventIdx]);
//  };
//
//  virtual auto eventWindowToModel(const EventWindow &event_window, const Hypothesis &hypothesis) const
//  -> Patch override {
//    return EventWindowToModelVector(event_window, hypothesis, weights_);
//  };
//
//  virtual auto initializeHypotheses() -> void override {
//    for (size_t i = 0; i < kNumHypotheses; ++i) { hypotheses_score_[i] = GetHypothesisScore(hypotheses_[i]); }
//  };
//
//  virtual auto updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event) -> void override {
//    // Ignore newest and oldest event and proceed from scratch
//    for (size_t i = 0; i < kNumHypotheses; ++i) { hypotheses_score_[i] = GetHypothesisScore(hypotheses_[i]); }
//  };
//
//  auto GetHypothesisScore(const Hypothesis &hypothesis) const -> Scalar {
//    const auto &[xp_vec, yp_vec] = PatchLocation(event_window_.ex_vec(), event_window_.ey_vec(), hypothesis);
//    auto sampled_value_vec = BilinearSampleArray2DVector(template_, xp_vec, yp_vec);
//    return (weights_ * sampled_value_vec).sum();
//  }
//
//  auto SetGaussianWeight() -> void {
//    constexpr auto kEventWindowSizeHalf = (kEventWindowSize - 1) / 2;
//    constexpr auto sigma = (kEventWindowSize / 6.0);
//    constexpr auto sigma2 = sigma * sigma;
//    constexpr auto sigma2_inv = 1.0 / sigma2;
//    weights_ =
//        Eigen::exp(-0.5 * sigma2_inv
//                       * Eigen::square(Eigen::Array<Weight, -1, 1>::LinSpaced(kEventWindowSize, 0, kEventWindowSize - 1)
//                                           - kEventWindowSizeHalf))
//            .array();
//    weights_ = weights_ / weights_.sum();
//  }
//};
//
