// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

namespace haste {

template<typename V, typename L, size_t S>
PatchType<V, L, S>::PatchType() {
  data_.setZero();
}

template<typename V, typename L, size_t S>
auto PatchType<V, L, S>::isWithinPatch(const Location &xp, const Location &yp) -> bool {
  return ((xp >= 0) && (xp < (kSize - 1)) && (yp >= 0) && (yp < (kSize - 1)));
  // TODO(ialzugaray): -1 in upper limit required for the current
}
template<typename V, typename L, size_t S>
auto PatchType<V, L, S>::incrementLocationByValue(const Location &xp, const Location &yp, const Value &increment)
    -> bool {
  return Interpolator::bilinearIncrementVector(data_, xp, yp, increment);
}

template<typename V, typename L, size_t S>
auto PatchType<V, L, S>::operator()(const Location &xp, const Location &yp) -> Value {
  return Interpolator::bilinearSample(data_, xp, yp);
}

template<typename V, typename L, size_t S>
auto PatchType<V, L, S>::block(const Location &xp, const Location &yp) -> Eigen::Ref<Eigen::Array<Value, 2, 2>> {
  return Interpolator::bilinearBlock(data_, xp, yp);
}

template<typename V, typename L, size_t S>
template<size_t N>
auto PatchType<V, L, S>::operator()(const Eigen::Array<Location, N, 1> &xp, const Eigen::Array<Location, N, 1> &yp)
-> Eigen::Array<Value, N, 1> {
  return Interpolator::bilinearSampleVector(data_, xp, yp);
}

}// namespace haste
