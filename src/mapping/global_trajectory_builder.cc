#include "global_trajectory_builder.h"

namespace AVP
{
namespace mapping
{
    
    GlobalTrajectoryBuilder::GlobalTrajectoryBuilder(PoseGraph* const pose_graph, std::shared_ptr<LocalTrajectoryBuilder> local_trajectory_builder)
    :pose_graph_{pose_graph}, local_trajectory_builder_{local_trajectory_builder}
    {
        odometry_sub_ = node_handle_.subscribe("odometry_noised",0,&GlobalTrajectoryBuilder::odometry_callback,this);
        semantic_scan_sub_ = node_handle_.subscribe("scan_semantic_points",0,&GlobalTrajectoryBuilder::semantic_callback,this);
        semantic_map_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("semantic_map",0);
        markers_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("markers",0);

        timers_.push_back(node_handle_.createTimer(
            ros::Duration(5), &GlobalTrajectoryBuilder::PubSemanticMap,this));

    }

    void GlobalTrajectoryBuilder::AddSensorData(const Eigen::Vector3d& odometery_pose)
    {
        local_trajectory_builder_->AddOdometryData(odometery_pose);
    }

    void GlobalTrajectoryBuilder::AddSensorData(const pcl::PointCloud<pcl::PointXYZ>& semantic_scan)
    {
        std::unique_ptr<LocalTrajectoryBuilder::MatchingResult> matching_result = 
            local_trajectory_builder_->AddSemanticScan(semantic_scan);

        if (matching_result == nullptr)
            return;
        
        if (matching_result->insertion_result!=nullptr)
        {
            pose_graph_->AddNode(matching_result->insertion_result->constant_data,
                                 matching_result->insertion_result->insertion_submaps);
        }
    }

    void GlobalTrajectoryBuilder::odometry_callback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        Eigen::Vector3d odometry_pose;
        odometry_pose << msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z;
        AddSensorData(odometry_pose);
    }
    void GlobalTrajectoryBuilder::semantic_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {   
        pcl::PointCloud<pcl::PointXYZ> semantic_scan;
        
        pcl::fromROSMsg(*msg, semantic_scan);
        AddSensorData(semantic_scan);
    }

    void GlobalTrajectoryBuilder::PubSemanticMap(const ros::TimerEvent& timer_event)
    {
        // ROS_INFO("publish semantic map!!!");
        pose_graph_->RunOptimization();

        pcl::PointCloud<pcl::PointXYZRGB> accumulated_submap_;
        visualization_msgs::MarkerArray marker_array_;

        std::map<unsigned int, TrajectoryNode> all_submap_list = pose_graph_->getSubmapList();

        for(auto it = all_submap_list.begin(); it != all_submap_list.end(); it++)
        {
            pcl::PointCloud<pcl::PointXYZ> temp = it->second.constant_data->filtered_semantic_data;

            Eigen::Vector3d pose_temp = it->second.global_pose;

            TransformPointCloud(temp, pose_temp);

            pcl::PointCloud<pcl::PointXYZRGB> submap;
            pcl::copyPointCloud(temp, submap);

            for(int i = 0; i < submap.points.size(); ++i)
            {
                submap.points[i].r = 0;
                submap.points[i].g = 255;
                submap.points[i].b = 0;
            }

            accumulated_submap_ += submap;

            // visualization_msgs::Marker marker;
            // marker.type = visualization_msgs::Marker::SPHERE;
            // marker.action = visualization_msgs::Marker::ADD;
            // marker.id = it->first;
            // geometry_msgs::Pose point_pose;
            // point_pose.position.x = it->second.submap->local_pose()[0];
            // point_pose.position.y = it->second.submap->local_pose()[1];
            // point_pose.orientation.z = it->second.submap->local_pose()[2];

            // marker.pose = point_pose;
            // marker.scale.x = 0.5;
            // marker.scale.y = 0.5;
            // marker.scale.z = 0.5;

            // marker.color.r = 0.f;
            // marker.color.g = 1.f;
            // marker.color.b = 0.f;
            // marker.color.a = 1.f;

            // marker.header.frame_id = "world";

            // marker_array_.markers.push_back(marker);
        }

        pcl::PointCloud<pcl::PointXYZRGB> filtered_accumulated_submap;
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setInputCloud(accumulated_submap_.makeShared());
        filter.setLeafSize(0.1f, 0.1f, 0.1f);
        filter.filter(filtered_accumulated_submap);

        sensor_msgs::PointCloud2 semantic_map_msg;
        pcl::toROSMsg(accumulated_submap_, semantic_map_msg);
        semantic_map_msg.header.frame_id = "world";
        semantic_map_pub_.publish(semantic_map_msg);
        // markers_pub_.publish(marker_array_);
        // ROS_INFO("marker array size: %d", marker_array_.markers.size());
    }


} // namespace mapping
} // namespace AVP
