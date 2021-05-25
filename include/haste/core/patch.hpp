// Copyright (c) 2020 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

#include <Eigen/Core>

#include "interpolator.hpp"

namespace haste {

template<typename Value_, typename Location_, size_t Size_>
class PatchType {// Either the Template or the Model of a feature
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Value = Value_;
  using Location = Location_;
  using Interpolator = InterpolatorType<Value, Location>;

  static constexpr size_t kSize = Size_;
  static_assert(kSize % 2 == 1, "Patch kSize parameter must be an odd");
  static constexpr size_t kSizeHalf = (kSize - 1) / 2;

  // TODO: consider creating using ValueArray = Eigen::Array type.
  using Data = Eigen::Array<Value, kSize, kSize>;
  using Vector = Eigen::Array<Value, kSize * kSize, 1>;// TODO: vector clashes in name. Probably better LinearArray?
  using VectorConstMap = Eigen::Map<const Vector>;
  using VectorMap = Eigen::Map<Vector>;

  PatchType();

  static auto isWithinPatch(const Location &xp, const Location &yp) -> bool;
  auto incrementLocationByValue(const Location &xp, const Location &yp, const Value &increment) -> bool;
  auto operator()(const Location &xp, const Location &yp) -> Value;
  auto block(const Location &xp, const Location &yp) -> Eigen::Ref<Eigen::Array<Value, 2, 2>>;

  template<size_t N>
  auto operator()(const Eigen::Array<Location, N, 1> &xp, const Eigen::Array<Location, N, 1> &yp)
      -> Eigen::Array<Value, N, 1>;

  auto data() const -> const Data&  { return data_; };
  auto data() -> Data& { return data_; };// TODO: probably this must be hidden.

 private:
  Data data_;
};
}// namespace haste

#include "patch_impl.hpp"
