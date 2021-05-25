// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
#include <string>
#include <Eigen/Eigen>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

namespace haste {

/// Auxilliar function to split strings into tokens given a delimiter.
auto splitString(const std::string& s, char delim) -> std::vector<std::string> {
  std::vector<std::string> ret;
  std::istringstream iss(s);
  std::string item;
  while (std::getline(iss, item, delim)) { ret.emplace_back() = item; }
  return ret;
}

template<typename Derived>
cv::Mat EigenArrayToCv(const Eigen::ArrayBase<Derived>& array) {
  Eigen::Matrix<typename Derived::Scalar, -1, -1> mat = array.matrix();
  cv::Mat mat_cv;
  cv::eigen2cv(mat, mat_cv);
  mat_cv.convertTo(mat_cv, CV_32F);
  cv::cvtColor(mat_cv, mat_cv, cv::COLOR_GRAY2RGB);
  return mat_cv;
}

template<typename Derived>
void ImshowEigenArray(const std::string& winname, const Eigen::ArrayBase<Derived>& array) {
  Eigen::Matrix<typename Derived::Scalar, -1, -1> matrix = array.matrix();
  cv::Mat matrix_cv;
  cv::eigen2cv(matrix, matrix_cv);
  cv::namedWindow(winname, cv::WINDOW_KEEPRATIO);
  cv::imshow(winname, matrix_cv);
}

template<typename Derived>
void ImshowEigenArrayNormalized(const std::string& winname, const Eigen::ArrayBase<Derived>& array) {
  Eigen::Matrix<typename Derived::Scalar, -1, -1> matrix =
      ((array - array.minCoeff()) / (array.maxCoeff() - array.minCoeff())).matrix();
  cv::Mat matrix_cv;
  cv::eigen2cv(matrix, matrix_cv);
  cv::namedWindow(winname, cv::WINDOW_KEEPRATIO);
  cv::imshow(winname, matrix_cv);
}

}// namespace haste