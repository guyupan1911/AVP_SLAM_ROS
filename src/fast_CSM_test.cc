#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <Eigen/Core>
#include "fast_correlative_scan_matcher.h"
#include "real_time_correlative_scan_matcher.h"
#include "submap.h"

using namespace AVP::mapping;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fast_CSM_test_node");
    ros::NodeHandle n;

    Eigen::Vector3d submap_pose, scan_pose;
    std::ifstream pose;
    pose.open("/home/catkin_ws/src/avp/data/pose.bin", std::ios::in | std::ios::binary);
    pose.read((char*)&submap_pose, sizeof(Eigen::Vector3d));
    pose.read((char*)&scan_pose, sizeof(Eigen::Vector3d));

    ROS_INFO("submap pose: %f %f %f", submap_pose[0], submap_pose[1], submap_pose[2]);
    ROS_INFO("scan pose: %f %f %f", scan_pose[0], scan_pose[1], scan_pose[2]);

    pcl::PointCloud<pcl::PointXYZ> submap, scan;
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/catkin_ws/src/avp/data/submap5.pcd", submap);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/catkin_ws/src/avp/data/scan443.pcd", scan);



    // do fast CSM
    ActiveSubmaps active_submap;
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    active_submap.InsertSemanticData(submap, origin);

    GridMap grid = *(active_submap.submaps().front()->grid());

    // for (size_t i = 0; i < grid.correspondence_cost_cells_.size(); i++)
    // {
    //     if(i % grid.limits().cell_limits().num_x_cells == 100)
    //     {
    //         grid.correspondence_cost_cells_[i] = 1;
    //         continue;
    //     }
    //     grid.correspondence_cost_cells_[i] = 32767;
    // }
    

    FastCorrelativeScanMatcher2D fast_csm(grid);

    RealTimeCorrelativeScanMatcher real_time_csm;

    float score;
    Eigen::Vector3d pose_estimated;

    // fast_csm.Match(origin, scan, 0.4, &score, &pose_estimated);

    score = real_time_csm.Match(scan_pose, scan, grid, pose_estimated);
    ROS_INFO("score: %f, estimated pose: %f %f %f", score, pose_estimated[0], pose_estimated[1], pose_estimated[2]);

    TransformPointCloud(scan, pose_estimated);

    pcl::PointCloud<pcl::PointXYZRGB> scan_color;
    pcl::copyPointCloud(scan,scan_color);
    for(auto& elem : scan_color)
    {
        elem.r = 0; elem.g = 255; elem.b = 0;
    }

    ros::Publisher submap_pub = n.advertise<sensor_msgs::PointCloud2>("submap",1);
    ros::Publisher scan_pub = n.advertise<sensor_msgs::PointCloud2>("scan",1);
    sensor_msgs::PointCloud2 submap_msg, scan_msg;
    pcl::toROSMsg(submap, submap_msg);
    pcl::toROSMsg(scan_color, scan_msg);

    submap_msg.header.frame_id = "world";
    scan_msg.header.frame_id = "world";

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        submap_pub.publish(submap_msg);
        scan_pub.publish(scan_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }

}

