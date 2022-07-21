#include "local_trajectory_builder.h"
namespace AVP
{
namespace mapping
{

LocalTrajectoryBuilder::LocalTrajectoryBuilder()
{
    accumulated_semantic_scan_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("accumulated_semantic_scan",0);
    estimated_path_pub_ = node_handle_.advertise<nav_msgs::Path>("path_estimated",0);
    noise_path_pub_ = node_handle_.advertise<nav_msgs::Path>("path_noise",0);
    path_estimated_.header.frame_id = "world";
    path_noise_.header.frame_id = "world";
}

std::unique_ptr<LocalTrajectoryBuilder::MatchingResult> LocalTrajectoryBuilder::AddSemanticScan(
    const pcl::PointCloud<pcl::PointXYZ>& semantic_scan)
{
    if (extrapolator_==nullptr)
    {
        extrapolator_.reset(new PoseExtrapolator);
        extrapolator_->AddPose(Eigen::Vector3d::Zero());
        return nullptr;
    }
    
    if (num_accumulated == 0)
    {
        accumulated_semantic_data_.clear();
    }
    Eigen::Vector3d predict_pose = extrapolator_->PredictPose();
    
    pcl::PointCloud<pcl::PointXYZ> temp = semantic_scan;

    // transform to world frame
    TransformPointCloud(temp,predict_pose);
    accumulated_semantic_data_ += temp;
    num_accumulated++;

    if (num_accumulated > 0)
    {   
        pcl::PointCloud<pcl::PointXYZRGB> colored_accumulated_semantic_data;
        pcl::copyPointCloud(accumulated_semantic_data_, colored_accumulated_semantic_data);
        for(auto iter = colored_accumulated_semantic_data.begin(); iter != colored_accumulated_semantic_data.end(); iter++)
        {
            iter->r = 0; iter->g = 255; iter->b = 0;
        }

        sensor_msgs::PointCloud2 colored_accumulated_semantic_scan_pub_msg;
        pcl::toROSMsg(colored_accumulated_semantic_data, colored_accumulated_semantic_scan_pub_msg);
        colored_accumulated_semantic_scan_pub_msg.header.frame_id = "world";
        accumulated_semantic_scan_pub_.publish(colored_accumulated_semantic_scan_pub_msg);

        num_accumulated = 0;
        
        Eigen::Matrix4d T_w_vehicle;
        T_w_vehicle << cos(predict_pose[2]), -1*sin(predict_pose[2]), 0, predict_pose[0],
                    sin(predict_pose[2]), cos(predict_pose[2]), 0, predict_pose[1],
                    0,0,1,0,
                    0,0,0,1;
        Eigen::Matrix4d T_vehicle_w = T_w_vehicle.inverse();
        // transform to tracking frame
        TransformPointCloud(accumulated_semantic_data_, T_vehicle_w);

        return AddAccumulatedSemantics(accumulated_semantic_data_, predict_pose);
    }
    return nullptr;
}

void LocalTrajectoryBuilder::AddOdometryData(const Eigen::Vector3d& odometry_pose)
{
    if (extrapolator_==nullptr)
        return;

    extrapolator_->AddOdometry(odometry_pose);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = odometry_pose[0];
    pose_msg.pose.position.y = odometry_pose[1];
    pose_msg.pose.orientation.z = odometry_pose[2];
    path_noise_.poses.push_back(pose_msg);
}

std::unique_ptr<LocalTrajectoryBuilder::MatchingResult> LocalTrajectoryBuilder::AddAccumulatedSemantics(const pcl::PointCloud<pcl::PointXYZ>& accumulated_semantics,
                                                     Eigen::Vector3d& global_pose)
{
    pcl::PointCloud<pcl::PointXYZ> filtered_accumulated_semantics;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(accumulated_semantics.makeShared());
    filter.setLeafSize(0.1f, 0.1f, 0.1f);
    filter.filter(filtered_accumulated_semantics);

    std::shared_ptr<Eigen::Vector3d> pose_estimated = ScanMatch(global_pose, filtered_accumulated_semantics);
    if(pose_estimated == nullptr)
    {
        ROS_WARN("Scan Matching failed");
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ> accumulated_semantics_in_world = accumulated_semantics;
    TransformPointCloud(accumulated_semantics_in_world, *pose_estimated);

    std::unique_ptr<InsertionResult> insertion_result =  
        InsertIntoSubmap(accumulated_semantics_in_world, filtered_accumulated_semantics, *pose_estimated);

    return std::unique_ptr<LocalTrajectoryBuilder::MatchingResult>( new LocalTrajectoryBuilder::MatchingResult{
        global_pose, std::move(accumulated_semantics_in_world), std::move(insertion_result)
    });
}

std::shared_ptr<Eigen::Vector3d> LocalTrajectoryBuilder::ScanMatch(const Eigen::Vector3d& predict_pose, 
                                       const pcl::PointCloud<pcl::PointXYZ>& downSampled_cloud) // pointcloud in tracking frame
{
    if (active_submaps_.submaps().empty())
    {
        Eigen::Vector3d pose_estimated = predict_pose;
        return std::make_shared<Eigen::Vector3d>(pose_estimated);
    }
    
    std::shared_ptr<const Submap> matching_submap = active_submaps_.submaps().front();

    Eigen::Vector3d pose_estimated = predict_pose;

    if (use_real_time_correlative_scan_match)
    {
        real_time_correlative_scan_matcher_.Match(predict_pose, downSampled_cloud, 
                                                  *matching_submap->grid(), pose_estimated);
    }
    
    std::shared_ptr<Eigen::Vector3d> pose_observation(new Eigen::Vector3d);

    ceres::Solver::Summary summary;
    ceres_scan_matcher_.Match(pose_estimated.head<2>(), 
                              pose_estimated, downSampled_cloud, 
                              *matching_submap->grid(), pose_observation.get(),
                              &summary);

    // *pose_observation = pose_estimated;
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = (*pose_observation)[0];
    pose_msg.pose.position.y = (*pose_observation)[1];
    pose_msg.pose.orientation.z = (*pose_observation)[2];
    path_estimated_.poses.push_back(pose_msg);

    estimated_path_pub_.publish(path_estimated_);
    noise_path_pub_.publish(path_noise_);

    extrapolator_->AddPose(*pose_observation);


    // ROS_INFO("predict: %f, %f, %f estimated: %f, %f, %f", predict_pose[0], predict_pose[1], predict_pose[2],
    //                                 (*pose_observation)[0], (*pose_observation)[1], (*pose_observation)[2]);
    return pose_observation;
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult> LocalTrajectoryBuilder::InsertIntoSubmap(
                                   const pcl::PointCloud<pcl::PointXYZ>& accumulated_semantic,
                                   const pcl::PointCloud<pcl::PointXYZ>& filtered_accumulated_semantic,
                                   const Eigen::Vector3d& pose_estimated)
{
    if (motion_filter_.IsSimilar(pose_estimated))
    {
        return nullptr;
    }

    std::vector<std::shared_ptr<const Submap>> insertion_submaps = 
        active_submaps_.InsertSemanticData(accumulated_semantic, pose_estimated);

    return std::unique_ptr<InsertionResult>(new InsertionResult{
        std::make_shared<TrajectoryNode::Data>(TrajectoryNode::Data{
            filtered_accumulated_semantic,
            pose_estimated
        }),
        std::move(insertion_submaps)
    });
}

} // namespace mapping
} // namespace AVP
