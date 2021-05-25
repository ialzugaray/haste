// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.


#include <glog/logging.h>

namespace haste {

template<typename Scalar>
auto PinholeRadTanCamera<Scalar>::distortPointNormalized(Vector2 &p, Matrix22 *jacobian) const -> void {
  auto &x = p.x();
  auto &y = p.y();
  const auto mx2_u = x * x;
  const auto my2_u = y * y;
  const auto mxy_u = x * y;
  const auto rho2_u = mx2_u + my2_u;
  const auto rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

  if (jacobian) {
    const auto duf_du = 1.0 + rad_dist_u + 2.0 * k1 * mx2_u + 4.0 * k2 * rho2_u * mx2_u + 2.0 * p1 * y + 6.0 * p2 * x;
    const auto duf_dv = 2.0 * k1 * mxy_u + 4.0 * k2 * rho2_u * mxy_u + 2.0 * p1 * x + 2.0 * p2 * y;
    const auto dvf_du = duf_dv;
    const auto dvf_dv = 1.0 + rad_dist_u + 2.0 * k1 * my2_u + 4.0 * k2 * rho2_u * my2_u + 2.0 * p2 * x + 6.0 * p1 * y;
    *jacobian << duf_du, duf_dv, dvf_du, dvf_dv;
  }

  x += x * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u);
  y += y * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

template<typename Scalar>
auto PinholeRadTanCamera<Scalar>::undistortPointNormalized(const Vector2 &p_distorted) const -> Vector2 {
  Vector2 p_undistorted = Vector2::Zero();

  for (int i = 0; i < kNumItersUndistortion_; ++i) {
    Matrix22 J;
    auto p_distorted_backwards = p_undistorted;
    distortPointNormalized(p_distorted_backwards, &J);
    Vector2 error(p_distorted - p_distorted_backwards);

    p_undistorted += (J.transpose() * J).inverse() * J.transpose() * error;
    if (error.dot(error) <= kMaxErrorUndistortion_ * kMaxErrorUndistortion_) { return p_undistorted; }
  }
  LOG(FATAL) << "Undistortion did not converge after max num of iterations.";
  return {};
}

template<typename Scalar>
auto PinholeRadTanCamera<Scalar>::undistortPoint(const Vector2 &p_distorted) const -> Vector2 {
  Vector2 p_undistorted_normalized =
      undistortPointNormalized({(p_distorted.x() - cx) / fx, (p_distorted.y() - cy) / fy});
  return {p_undistorted_normalized.x() * fx + cx, p_undistorted_normalized.y() * fy + cy};
}

template<typename Scalar>
PinholeRadTanCamera<Scalar>::UndistortionMap::UndistortionMap(const PinholeRadTanCamera &camera) {
  data_.resize(camera.width, camera.height);
  for (size_t x = 0; x < camera.width; x++) {
    for (size_t y = 0; y < camera.height; y++) { data_(x, y) = camera.undistortPoint({x, y}); }
  }
}

template<typename Scalar>
auto PinholeRadTanCamera<Scalar>::UndistortionMap::operator()(const int &x, const int &y) const
    -> std::pair<Scalar, Scalar> {
  const Vector2 &p = data_(x, y);
  return {p[0], p[1]};
}
template<typename Scalar>
auto PinholeRadTanCamera<Scalar>::createUndistortionMap() const -> UndistortionMap {
  return UndistortionMap(*this);
}

}// namespace haste
