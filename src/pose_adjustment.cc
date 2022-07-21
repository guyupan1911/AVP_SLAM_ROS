#include <ros/ros.h>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <ceres/ceres.h>
#include "optimization/spa_cost.h"
#include <fstream>

using namespace AVP;
using namespace mapping;

std::map<unsigned int, std::pair<Eigen::Vector3d, Eigen::Vector3d>> nodes;
std::map<unsigned int, std::pair<Eigen::Vector3d, Eigen::Vector3d>> submaps;

std::vector<constraint> constraints;
nav_msgs::Path path_true, path_noise, path_estimated;
bool loop_detected = false;

bool isSimilar(const geometry_msgs::PoseStamped& pose)
{
    static Eigen::Vector3d last_node_pose = Eigen::Vector3d::Zero();

    double dx = pose.pose.position.x - last_node_pose[0];
    double dy = pose.pose.position.y - last_node_pose[1];
    double dz = pose.pose.orientation.z - last_node_pose[2];

    if (std::sqrt(std::pow(dx,2)+std::pow(dy,2)) < 0.2 && sqrt(std::pow(dz,2))<0.2)
    {
        return true;
    }
    
    last_node_pose[0] = pose.pose.position.x;
    last_node_pose[1] = pose.pose.position.y;
    last_node_pose[2] = pose.pose.orientation.z;
    return false;
}

std::array<double, 3> FromPose(const Eigen::Vector3d& pose) 
{
  return {{pose[0], pose[1], normalize(pose[2])}};
}

ceres::Problem problem;
std::map<unsigned int, std::array<double, 3>> c_nodes;
std::map<unsigned int, std::array<double, 3>> c_submaps;


void callback(const geometry_msgs::PoseStampedConstPtr& pose_true, const geometry_msgs::PoseStampedConstPtr& pose_noise)
{
    static int num = 0;
    if (!isSimilar(*pose_true))
    {
        Eigen::Vector3d node_true, node_noise;
        node_true << pose_true->pose.position.x, pose_true->pose.position.y, pose_true->pose.orientation.z;
        // node_noise << pose_noise->pose.position.x, pose_noise->pose.position.y, pose_noise->pose.orientation.z;
        node_noise = node_true;
        node_noise[0] += 1;
        node_noise[1] += 1;

        nodes.insert(std::make_pair(nodes.size(), std::make_pair(node_true, node_noise)));
        c_nodes.insert(std::make_pair(nodes.rbegin()->first, FromPose(nodes.rbegin()->second.second)));
        if(num % 50 == 0)
        {
            submaps.insert(std::make_pair(submaps.size(), std::make_pair(node_true, node_noise)));
            c_submaps.insert(std::make_pair(submaps.rbegin()->first, FromPose(submaps.rbegin()->second.second)));
        }

        constraints.push_back(constraint{submaps.rbegin()->first, nodes.rbegin()->first, submaps.rbegin()->second.second, nodes.rbegin()->second.second, constraint::constraint_type::inter});
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CostFunction, 3, 3, 3>(new CostFunction(constraints.back().observerd_relative_pose, 0.2)),
            nullptr, c_submaps.find(constraints.back().submap_id_)->second.data(),
            c_nodes.find(constraints.back().node_id_)->second.data()
        );

        if (submaps.size() > 1)
        {
            constraints.push_back(constraint{(--submaps.rbegin())->first, nodes.rbegin()->first, (--submaps.rbegin())->second.second, nodes.rbegin()->second.second, constraint::constraint_type::inter});
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<CostFunction, 3, 3, 3>(new CostFunction(constraints.back().observerd_relative_pose, 0.2)),
                nullptr, c_submaps.find(constraints.back().submap_id_)->second.data(),
                c_nodes.find(constraints.back().node_id_)->second.data()
            );
        }
        


        if (num == 600)
        {
                // constraints.push_back(constraint{submaps.begin()->first, nodes.rbegin()->first, submaps.begin()->second.first, nodes.rbegin()->second.first, constraint::constraint_type::global});
                // problem.AddResidualBlock(
                //         new ceres::AutoDiffCostFunction<CostFunction,3,3,3>(new CostFunction(constraints.back().observerd_relative_pose, 0.01)), nullptr, 
                //         c_submaps.find(constraints.back().submap_id_)->second.data(),
                //         c_nodes.find(constraints.back().node_id_)->second.data()
                // );

                constraints.push_back(constraint{submaps.begin()->first, 300, submaps.begin()->second.first, nodes[300].first, constraint::constraint_type::global});
                problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<CostFunction,3,3,3>(new CostFunction(constraints.back().observerd_relative_pose, 0.01)), nullptr, 
                        c_submaps.find(constraints.back().submap_id_)->second.data(),
                        c_nodes.find(constraints.back().node_id_)->second.data()
                );


                problem.SetParameterBlockConstant(c_nodes.begin()->second.data());
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 50;
                ceres::Solver::Summary summary;

                ceres::Solve(options, &problem, &summary);
                // std::cout << summary.BriefReport() << "\n";

                for(auto& elem : c_nodes)
                {
                    geometry_msgs::PoseStamped pose;
                    pose.pose.position.x = elem.second[0];
                    pose.pose.position.y = elem.second[1];
                    pose.pose.orientation.z = elem.second[2];
                    
                    path_estimated.poses.push_back(pose);
                    ROS_INFO("path_estimated size: %d", path_estimated.poses.size());
                    std::cout << "node pose: " << elem.second[0] << " " << elem.second[1] << " " << elem.second[2] << std::endl;
                }                 
        }
        
        path_true.poses.push_back(*pose_true);
        path_noise.poses.push_back(*pose_noise);    
        num ++;
    }

    ROS_INFO("constraint size: %d, loop_detedted: %d num: %d", constraints.size(), loop_detected, num);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_adjustment_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_true_sub(nh, "/odometry_true", 0);
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_noise_sub(nh, "/odometry_noised", 0);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pose_true_sub, pose_noise_sub); //queue size=10
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Publisher path_true_pub = nh.advertise<nav_msgs::Path>("path_true",10);
    ros::Publisher path_noise_pub = nh.advertise<nav_msgs::Path>("path_noise",10);
    ros::Publisher path_estimated_pub = nh.advertise<nav_msgs::Path>("path_estimated",10);


    path_true.header.frame_id = "world";
    path_noise.header.frame_id = "world";
    path_estimated.header.frame_id = "world";

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        path_true_pub.publish(path_true);
        path_noise_pub.publish(path_noise);
        path_estimated_pub.publish(path_estimated);
        ros::spinOnce();
    }
    
    return 0;
}









