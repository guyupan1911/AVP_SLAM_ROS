#include "spa_cost_function.h"

namespace AVP
{
    
namespace mapping
{
    
template <typename T>
static std::array<T, 3> ComputeUnscaledError(
    const Eigen::Vector3d& relative_pose, const T* const start,
    const T* const end) {
  const T cos_theta_i = cos(start[2]);
  const T sin_theta_i = sin(start[2]);
  const T delta_x = end[0] - start[0]; // t2 -t1
  const T delta_y = end[1] - start[1];
  const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y, // R.inverse * (t2 -t1)
                  -sin_theta_i * delta_x + cos_theta_i * delta_y,
                  end[2] - start[2]};
  return {{T(relative_pose[0]) - h[0],
           T(relative_pose[1]) - h[1],
           normalize(T(relative_pose[2]) - h[2])
           }};
}

template <typename T>
std::array<T, 3> ScaleError(const std::array<T, 3>& error,
                            double translation_weight, double rotation_weight) {
  return {{
      error[0] * translation_weight,
      error[1] * translation_weight,
      error[2] * rotation_weight
  }};
}

ceres::CostFunction* CreateAutoDiffSpaCostFunction(
    const Constraint& observed_relative_pose) {
  return new ceres::AutoDiffCostFunction<SpaCostFunction2D, 3 /* residuals */,
                                         3 /* start pose variables */,
                                         3 /* end pose variables */>(
      new SpaCostFunction2D(observed_relative_pose));
}

} // namespace mapping


} // namespace AVP
