#ifndef POSE_GRAPH_H
#define POSE_GRAPH_H

#include "submap.h"
#include "trajectory_node.h"
#include "optimization_problem.h"
#include "constraint.h"
#include <set>
#include <map>
#include <mutex>
#include <ros/ros.h>
#include "fast_correlative_scan_matcher.h"
#include "real_time_correlative_scan_matcher.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include "io.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <fstream>
#include <mutex>

namespace AVP
{
namespace mapping
{

enum class SubmapState { kNoConstraintSearch, kFinished };

struct InternalSubmapData
{
    std::shared_ptr<const Submap> submap;
    SubmapState state = SubmapState::kNoConstraintSearch;

    std::set<unsigned int> node_ids;
};

struct PoseGraphData
{
    std::map<unsigned int, InternalSubmapData> submap_data;
    std::map<unsigned int, TrajectoryNode> trajectory_nodes;
    std::map<unsigned int, Eigen::Vector3d> global_submap_poses;
    std::vector<Constraint> constraints;
    std::map<unsigned int, SubmapSpec2D> global_submap_poses_2d;
};



class PoseGraph
{
    public:
    PoseGraph()
    {
        optimization_problem_.reset(new OptimizationProblem);
        path_estimated_after_spa_pub = node_handle_.advertise<nav_msgs::Path>("path_estimated_after_spa", 1);
        submap_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("submap_match",0);
        scan_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("scan_match",0);
        submap_grid_map_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("submap_grid_map",0);
    }

    void AddNode(std::shared_ptr<const TrajectoryNode::Data> constant_data, 
                                 const std::vector<std::shared_ptr<const Submap>>& insertion_submaps);

    void ComputeConstraintsForNode(const unsigned int node_id, std::vector<std::shared_ptr<const Submap>> insertion_submaps, const bool newly_finished_submap);
    
    std::vector<unsigned int> InitializeGlobalSubmapPoses(const std::vector<std::shared_ptr<const Submap>>& insertion_submaps);

    void DetectLoopAndComputeConstraint(unsigned int submap_id, unsigned int node_id);

    void RunOptimization();

    Eigen::Matrix4d ComputeLocalToGlobalTransform();

    std::map<unsigned int, TrajectoryNode> getSubmapList();



    PoseGraphData data_;
    private:
    std::mutex mMutex_data_;
    std::unique_ptr<OptimizationProblem> optimization_problem_;

    int num_nodes_since_last_loop_closure_ = 0;
    bool running_optimization = false;

    ros::NodeHandle node_handle_;
    ros::Publisher path_estimated_after_spa_pub;
    ros::Publisher submap_pub_;
    ros::Publisher scan_pub_;
    ros::Publisher submap_grid_map_pub_;

    std::map<unsigned int, unsigned int> submap_node_has_looped;
    std::mutex mMutex_loop;
};

} // namespace mapping
} // namespace AVP


#endif