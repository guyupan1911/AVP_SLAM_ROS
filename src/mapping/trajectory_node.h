#ifndef TRAJECTORY_NODE_H
#define TRAJECTORY_NODE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace AVP
{
namespace mapping
{

struct TrajectoryNode
{
    struct Data
    {
        pcl::PointCloud<pcl::PointXYZ> filtered_semantic_data;
        Eigen::Vector3d local_pose;
    };

    std::shared_ptr<const Data> constant_data;
    Eigen::Vector3d global_pose;
};

} // namespace mapping
} // namespace AVP


#endif