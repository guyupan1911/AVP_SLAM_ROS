#ifndef GLOBAL_TRAJECTORY_BUILDER_H
#define GLOBAL_TRAJECTORY_BUILDER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <map>
#include <mutex>
#include "pose_graph.h"
#include "local_trajectory_builder.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl_conversions/pcl_conversions.h>

namespace AVP
{
namespace mapping
{
    
class GlobalTrajectoryBuilder
{
public:
    GlobalTrajectoryBuilder(PoseGraph* const pose_graph, std::shared_ptr<LocalTrajectoryBuilder> local_trajectory_builder);

    void odometry_callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void semantic_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

    void AddSensorData(const Eigen::Vector3d& odometery_pose);
    void AddSensorData(const pcl::PointCloud<pcl::PointXYZ>& semantic_scan);

    void PubSemanticMap(const ros::TimerEvent& timer_event);

    PoseGraph* const pose_graph_;
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber semantic_scan_sub_;
    ros::Publisher semantic_map_pub_;
    ros::Publisher markers_pub_;
    std::vector<ros::Timer> timers_;

    std::shared_ptr<LocalTrajectoryBuilder> local_trajectory_builder_;

    std::set<unsigned int> accumulated_submap_id_;

};



} // namespace mapping

} // namespace AVP


#endif