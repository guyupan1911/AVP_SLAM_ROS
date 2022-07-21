#ifndef CERES_SCAN_MATCHER_H
#define CERES_SCAN_MATCHER_H

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "gridmap.h"
#include "occupied_space_cost_function.h"
#include "rotation_delta_cost_functor_2d.h"
#include "translation_delta_cost_functor_2d.h"

namespace AVP
{
    
namespace mapping
{
    
// Align scans with an existing map using Ceres.
class CeresScanMatcher2D {
 public:


  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const Eigen::Vector2d& target_translation,
             const Eigen::Vector3d& initial_pose_estimate,
             const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const GridMap& grid,
             Eigen::Vector3d* pose_estimate,
             ceres::Solver::Summary* summary) const;

};

} // namespace mapping


} // namespace AVP


#endif