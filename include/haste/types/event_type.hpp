// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
#include <tuple>
namespace haste {

template<typename Location_, typename Time_ = double>
struct TEvent {
  using Location = Location_;
  using Time = Time_;
  using Polarity = bool;

  using EventTuple = std::tuple<Time, Location, Location, Polarity>;
  using EventTupleRef = std::tuple<Time&, Location&, Location&, Polarity&>;
  Time t;
  Location x, y;
  Polarity p;

  auto operator=(TEvent& event) -> EventTupleRef {
    return std::forward_as_tuple(event.t, event.x, event.y, event.p); }
  auto operator=(const TEvent& event) const -> EventTuple {
    return {event.t, event.x, event.y, event.p};
  }
};


}// namespace haste