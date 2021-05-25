// Copyright (c) 2020 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

#include <Eigen/Core>
#include <haste/types/event.hpp>

namespace haste {
template<typename Event_, size_t Size_>
class FixedSizeLocationEventWindowType {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr size_t kSize = Size_;
  static_assert(kSize % 2 == 1, "EventWindow kSize parameter must be an odd");

  using Event = Event_;
  using Time = typename Event::Time;
  using Location = typename Event::Location;
  using EventTuple = std::tuple<Time, Location, Location>;

  template<typename T>
  using EventWindowVector = Eigen::Array<T, kSize, 1>;
  using TimeVector = EventWindowVector<Time>;
  using LocationVector = EventWindowVector<Location>;

  static constexpr size_t kOldestEventIdx = 0;
  static constexpr size_t kNewestEventIdx = kSize - 1;
  static constexpr size_t kMiddleEventIdx = (kSize - 1) / 2;

  FixedSizeLocationEventWindowType();

  // TODO(ialzugaray): template to direct access without getEvent
  auto getEvent(const size_t &idx) const -> EventTuple;
  auto oldestEvent() const -> EventTuple;
  auto newestEvent() const -> EventTuple;
  auto middleEvent() const -> EventTuple;

  auto setEvent(const Time &et, const Location &ex, const Location &ey, const size_t &idx) -> void;
  auto setEvent(const EventTuple &event, const size_t &idx) -> void;
  auto appendEvent(const Time &et, const Location &ex, const Location &ey) -> EventTuple;
  auto appendEvent(const EventTuple &new_event) -> EventTuple;

  // TODO(ialzugaray): convert to const ref getters. Maybe transpose the underlying data depending on use
  auto et_vec() const -> TimeVector { return et_vec_; };
  auto ex_vec() const -> LocationVector { return ex_vec_; };
  auto ey_vec() const -> LocationVector { return ey_vec_; };

 protected:
  TimeVector et_vec_;
  LocationVector ex_vec_;
  LocationVector ey_vec_;
};

}// namespace haste

#include "event_window_impl.hpp"
