#ifndef LOCAL_TRAJECTORY_BUILDER_H
#define LOCAL_TRAJECTORY_BUILDER_H

#include "pose_extrapolator.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <memory>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <unistd.h>
#include "transform.h"
#include "motion_filter.h"
#include "trajectory_node.h"
#include "submap.h"
#include "real_time_correlative_scan_matcher.h"
#include "ceres_scan_matcher_2d.h"
namespace AVP
{
namespace mapping
{
    
class LocalTrajectoryBuilder
{
public:

    LocalTrajectoryBuilder();

    struct InsertionResult 
    {
        std::shared_ptr<const TrajectoryNode::Data> constant_data; 
        std::vector<std::shared_ptr<const Submap>> insertion_submaps; 
    };

    struct MatchingResult 
    {
        Eigen::Vector3d local_pose;
        pcl::PointCloud<pcl::PointXYZ> semantic_data; 
        std::unique_ptr<const InsertionResult> insertion_result;
    };

    std::unique_ptr<MatchingResult> AddSemanticScan(const pcl::PointCloud<pcl::PointXYZ>& semantic_scan);
    void AddOdometryData(const Eigen::Vector3d& odometry_pose);

    std::unique_ptr<MatchingResult> AddAccumulatedSemantics(const pcl::PointCloud<pcl::PointXYZ>& accumulated_semantics, Eigen::Vector3d& global_pose);

    std::shared_ptr<Eigen::Vector3d> ScanMatch(const Eigen::Vector3d& predict_pose, 
                                               const pcl::PointCloud<pcl::PointXYZ>& downSampled_cloud);

    std::unique_ptr<InsertionResult> InsertIntoSubmap(  
                                   const pcl::PointCloud<pcl::PointXYZ>& accumulated_semantic,
                                   const pcl::PointCloud<pcl::PointXYZ>& filtered_accumulated_semantic,
                                   const Eigen::Vector3d& pose_estimated);

private:
    bool use_real_time_correlative_scan_match = false;

    ros::NodeHandle node_handle_;
    ros::Publisher accumulated_semantic_scan_pub_;
    ros::Publisher estimated_path_pub_;
    ros::Publisher noise_path_pub_;

    nav_msgs::Path path_estimated_;
    nav_msgs::Path path_noise_;


    MotionFilter motion_filter_;
    std::unique_ptr<PoseExtrapolator> extrapolator_;
    int num_accumulated = 0;
    pcl::PointCloud<pcl::PointXYZ> accumulated_semantic_data_;
    ActiveSubmaps active_submaps_;

    RealTimeCorrelativeScanMatcher real_time_correlative_scan_matcher_;
    CeresScanMatcher2D ceres_scan_matcher_;

};

} // namespace mapping

} // namespace AVP


#endif