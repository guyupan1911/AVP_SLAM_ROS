#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>



namespace AVP
{
namespace mapping
{
    
void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Vector3d& P_w_vehicle);

void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Matrix4d& T);

pcl::PointCloud<pcl::PointXYZ> TransformPointCloudAndReturn(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Vector3d& P_w_tracking);

} // namespace mapping
} // namespace AVP


#endif