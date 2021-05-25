// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
#include <Eigen/Eigen>

namespace haste {
template<typename Value_, typename Location_>
class InterpolatorType {
 public:
  using Value = Value_;
  using Location = Location_;

  // TODO: Consider Rows and Cols to become size_t parameters.
  // TODO: Consider using derived types directly from Eigen to deduct Value / Location / Sizes.
  template<int kRows, int kCols>
  using ValueArray = Eigen::Array<Value, kRows, kCols>;

  template<int kRows, int kCols>
  using LocationArray = Eigen::Array<Location, kRows, kCols>;

  template<int kRows, int kCols>
  static inline auto bilinearIncrementVector(ValueArray<kRows, kCols> &mat, const Location &x, const Location &y,
                                             const Value &w) -> bool;

  template<int kRows, int kCols>
  static inline auto bilinearSample(const ValueArray<kRows, kCols> &mat, const Location &x, const Location &y) -> Value;

  template<int kRows, int kCols, int kSamples>
  static inline auto bilinearSampleVector(const ValueArray<kRows, kCols> &mat, const LocationArray<kSamples, 1> &x_vec,
                                          const LocationArray<kSamples, 1> &y_vec) -> ValueArray<kSamples, 1>;

  static inline auto bilinearKernel(const Location &x, const Location &y) -> ValueArray<2, 2>;

  template<int kRows, int kCols>
  static auto bilinearBlock(const ValueArray<kRows, kCols>& mat, const Location &xp, const Location &yp) -> Eigen::Ref<ValueArray<2, 2>>;

};
}// namespace haste

#include "interpolator_impl.hpp"
