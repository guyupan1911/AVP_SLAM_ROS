#ifndef OCCUPIED_SPACE_COST_FUNCTION_H
#define OCCUPIED_SPACE_COST_FUNCTION_H

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "gridmap.h"

namespace AVP
{
    
namespace mapping
{
    
ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
    const GridMap& grid);


} // namespace mapping


} // namespace AVP


#endif