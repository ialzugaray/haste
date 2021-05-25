// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

namespace haste {
template<typename Value, typename Location>
template<int kRows, int kCols>
auto InterpolatorType<Value, Location>::bilinearIncrementVector(ValueArray<kRows, kCols> &mat, const Location &x,
                                                                const Location &y, const Value &w) -> bool {

  static_assert(kRows != -1 || kCols != -1, "Sampled array must be fixed size");
  // TODO (ialzugaray): this range prevents using elements just
  if (x >= 0 && y >= 0 && x < (kRows - 1) && y < (kCols - 1)) {
    DCHECK_GE(x, 0) << "int downcasting not expected for negative values";
    DCHECK_GE(y, 0) << "int downcasting not expected for negative values";

    const int ix = (int) x;
    const int iy = (int) y;
    const Location dx = x - ix;
    const Location dy = y - iy;
    const Location dxdy = dx * dy;

    mat(ix + 1, iy + 1) += dxdy * w;
    mat(ix, iy + 1) += (dy - dxdy) * w;
    mat(ix + 1, iy) += (dx - dxdy) * w;
    mat(ix, iy) += (1 - dx - dy + dxdy) * w;
    return true;
  } else {
    return false;
  }
}

template<typename Value, typename Location>
template<int kRows, int kCols>
auto InterpolatorType<Value, Location>::bilinearSample(const ValueArray<kRows, kCols> &mat, const Location &x,
                                                       const Location &y) -> Value {
  static_assert(std::is_same<Value, float>::value || std::is_same<Value, double>::value,
                "Arrays must be composed of doubles or float");
  static_assert(kRows != -1 || kCols != -1, "Sampled array must be fixed size");

  const int ix = (int) x;
  const int iy = (int) y;
  const Location dx = x - ix;
  const Location dy = y - iy;
  const Location dxdy = dx * dy;

  if (x >= 0 && y >= 0 && x < (kRows - 1)
      && y < (kCols - 1)) {// TODO (ialzugaray): this range prevents using elements just
    // in kPatchSize because we are using +1 later on
    // clang-format off
    return  dxdy * mat(ix + 1, iy + 1) +
            (dy - dxdy) * mat(ix, iy + 1) +
            (dx - dxdy) * mat(ix + 1, iy) +
            (1 - dx - dy + dxdy) * mat(ix, iy);
    // clang-format on

  } else {
    return Value{0};
  }
}

template<typename Value, typename Location>
template<int kRows, int kCols, int kSamples>
auto InterpolatorType<Value, Location>::bilinearSampleVector(const ValueArray<kRows, kCols> &mat,
                                                             const LocationArray<kSamples, 1> &x_vec,
                                                             const LocationArray<kSamples, 1> &y_vec)
    -> ValueArray<kSamples, 1> {
  static_assert(std::is_same<Value, float>::value || std::is_same<Value, double>::value,
                "Arrays must be composed of doubles or float");
  static_assert(kRows != -1 || kCols != -1, "Sampled array must be fixed size");

  using ValueVector = Eigen::Array<Value, kSamples, 1>;
  using LocationVector = Eigen::Array<Location, kSamples, 1>;
  using LinearArray = Eigen::Array<Value, kRows * kCols, 1>;
  using LinearArrayConstMap = Eigen::Map<const LinearArray>;

  // TODO(ialzugaray): is this better than flooring?
  // TODO(ialzugaray): Verify that this is better than using the stack
  const LocationVector xfloor_vec = x_vec.template cast<int>().template cast<Location>();
  const LocationVector yfloor_vec = y_vec.template cast<int>().template cast<Location>();

  const LocationVector xfract_vec = x_vec - xfloor_vec;
  const LocationVector yfract_vec = y_vec - yfloor_vec;

  const LinearArrayConstMap v{mat.data()};

  // clang-format off
    //TODO: Could we do this with previous slicing?
    //TODO: verify that kRows is actually the offset of each col;
    return (x_vec >= 0 && y_vec >= 0 && x_vec < (kRows - 1) && y_vec < (kCols - 1)) // TODO (ialzugaray): this range prevents using elements just in kPatchSize because we are using +1 later on
        .select(  v(xfloor_vec + kRows * yfloor_vec) * ((1 - xfract_vec) * (1 - yfract_vec)) +
                      v(xfloor_vec + kRows * (yfloor_vec  + 1)) * ((1 - xfract_vec) * yfract_vec) +    // TODO: double check these two factors
                      v((xfloor_vec + 1) + kRows * yfloor_vec) * (xfract_vec * (1 - yfract_vec)) + // TODO: double check these two factors
                      v((xfloor_vec + 1) + kRows * (yfloor_vec  + 1)) * xfract_vec * yfract_vec,
                  0);
  // clang-format on
}

template<typename Value, typename Location>
auto InterpolatorType<Value, Location>::bilinearKernel(const Location &x, const Location &y) -> ValueArray<2, 2> {
//  Value dx = x - std::floor(x);
//  Value dy = y - std::floor(y);
//  Value dxdy = dx * dy;

  const int ix = (int) x;
  const int iy = (int) y;
  const Location dx = x - ix;
  const Location dy = y - iy;
  const Location dxdy = dx * dy;


  ValueArray<2, 2> kernel;// TODO assumes column major

  kernel(0, 0) = (1 - dx - dy + dxdy);
  kernel(1, 0) = (dx - dxdy);
  kernel(0, 1) = (dy - dxdy);
  kernel(1, 1) = dxdy;

  return kernel;
}

template<typename Value, typename Location>
template<int kRows, int kCols>
auto InterpolatorType<Value, Location>::bilinearBlock(const ValueArray<kRows, kCols> &mat, const Location &xp,
                                                      const Location &yp) -> Eigen::Ref<ValueArray<2, 2>> {
  // TODO: Guard if out of range;
  //if (xp >= 0 && yp >= 0 && xp < (kSize - 1) && yp < (kSize - 1)) {
  return mat.block<2, 2>((int) xp, (int) yp);// TODO: check this conversion to int
  /*} else {
    return Eigen::Array<Value, 2, 2>::Zero();
  }
   */
}

}// namespace haste