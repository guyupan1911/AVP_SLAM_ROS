#include "ceres_scan_matcher_2d.h"

namespace AVP
{
    
namespace mapping
{
    
void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                               const Eigen::Vector3d& initial_pose_estimate,
                               const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
                               const GridMap& grid,
                               Eigen::Vector3d* const pose_estimate,
                               ceres::Solver::Summary* const summary) const {
  double ceres_pose_estimate[3] = {initial_pose_estimate[0],
                                   initial_pose_estimate[1],
                                   initial_pose_estimate[2]};
  ceres::Problem problem;
  ceres::Solver::Options ceres_solver_options_;
  ceres_solver_options_.max_num_iterations = 50;

  // 地图部分的残差
    problem.AddResidualBlock(
        CreateOccupiedSpaceCostFunction2D(
            1. /
                std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, grid),
        nullptr /* loss function */, ceres_pose_estimate);
    
//   // 平移的残差

  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          8, target_translation), // 平移的目标值, 没有使用校准后的平移
      nullptr /* loss function */, ceres_pose_estimate);      // 平移的初值

//   // 旋转的残差, 固定了角度不变
//   problem.AddResidualBlock(
//       RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
//           1, ceres_pose_estimate[2]), // 角度的目标值
//       nullptr /* loss function */, ceres_pose_estimate);       // 角度的初值

  // 根据配置进行求解
  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = Eigen::Vector3d
      {ceres_pose_estimate[0], ceres_pose_estimate[1], ceres_pose_estimate[2]};
}

} // namespace mapping


} // namespace AVP
