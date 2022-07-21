#ifndef ROTATION_DELTA_COST_FUNCTION_H
#define ROTATION_DELTA_COST_FUNCTION_H

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace AVP
{
    
namespace mapping
{
    
class RotationDeltaCostFunctor2D {
 public:
  // 静态成员函数, 返回CostFunction
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const double target_angle) {
    return new ceres::AutoDiffCostFunction<
        RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
        new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
  }

  // 旋转量残差的计算
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }

 private:
  explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                      const double target_angle)
      : scaling_factor_(scaling_factor), angle_(target_angle) {}

  RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
  RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) =
      delete;

  const double scaling_factor_;
  const double angle_;
};

} // namespace mapping


} // namespace AVP


#endif