// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.
#pragma once


#include <Eigen/Eigen>

namespace haste {

template<typename Scalar_>
struct PinholeRadTanCamera {
 public:
  using Scalar = Scalar_;
  using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
  using Matrix22 = Eigen::Matrix<Scalar, 2, 2>;

  inline auto distortPointNormalized(Vector2 &p, Matrix22 *jacobian) const -> void;
  inline auto undistortPointNormalized(const Vector2 &p_distorted) const -> Vector2;
  inline auto undistortPoint(const Vector2 &p_distorted) const -> Vector2;

  struct UndistortionMap {
   public:
    UndistortionMap(const PinholeRadTanCamera &camera);
    inline auto operator()(const int &x, const int &y) const -> std::pair<Scalar, Scalar>;
   private:
    Eigen::Array<Vector2, -1, -1> data_;
  };

  inline auto createUndistortionMap() const -> UndistortionMap;

  size_t width, height;     ///< Sensor size.
  Scalar fx, fy, cx, cy;    ///< Intrinsics.
  Scalar k1, k2, p1, p2, k3;/// << Rad-tan distortion.

 protected:
  static constexpr auto kNumItersUndistortion_ = 50;  ///< Max number of iterations until convergence.
  static constexpr auto kMaxErrorUndistortion_ = 1e-3;///< Max error in iterative distortion-undistortion.

};

}// namespace haste


#include "camera_impl.hpp"