#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <deque>
#include <mutex>
#include <Eigen/Core>
#include <boost/bind.hpp>
#include "pose_extrapolator.h"
#include "global_trajectory_builder.h"
#include "pose_graph.h"
#include "local_trajectory_builder.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_node");
    
    ros::start();

    std::shared_ptr<AVP::mapping::LocalTrajectoryBuilder> local_trajectory_builder(
        new AVP::mapping::LocalTrajectoryBuilder
    );
 
    AVP::mapping::GlobalTrajectoryBuilder global_trajectory_builder2D(
        new AVP::mapping::PoseGraph, local_trajectory_builder);
    
    ros::spin();
    

    return 0;
}