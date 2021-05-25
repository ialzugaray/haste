// Copyright (c) 2020 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

#include <tuple>

namespace haste {

template <typename L, typename T = double> struct EventTraitType {
  using Time = T;
  using Location = L;
  using Polarity = bool;
  using TimeLocationTuple = std::tuple<Time, Location, Location>;
  using TimeLocationPolarityTuple =
      std::tuple<Time, Location, Location, Polarity>;
};

} // namespace haste
