#include "pose_graph.h"

namespace AVP
{
namespace mapping
{

void PoseGraph::AddNode(std::shared_ptr<const TrajectoryNode::Data> constant_data, 
                                        const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
{
    Eigen::Matrix4d T_global_local = ComputeLocalToGlobalTransform();
    Eigen::Vector4d P_local_node;
    P_local_node << constant_data->local_pose, 1;
    auto iter_node = data_.trajectory_nodes.insert(std::make_pair(data_.trajectory_nodes.size(), 
                                                 TrajectoryNode{constant_data, (T_global_local*P_local_node).head(3)}));
    
    if (data_.submap_data.size() == 0 || data_.submap_data.rbegin()->second.submap != insertion_submaps.back())
    {
        InternalSubmapData temp;
        auto ret = data_.submap_data.insert(std::make_pair(data_.submap_data.size(), temp));
        if (ret.second == true)
        {
            ret.first->second.submap = insertion_submaps.back();
        }
    }

    const bool newly_finished_submap = insertion_submaps.front()->insertion_finished();

    ComputeConstraintsForNode(iter_node.first->first, insertion_submaps, newly_finished_submap);
    // ROS_INFO("submap size: %d, node size: %d", data_.submap_data.size(), data_.trajectory_nodes.size());

}

void PoseGraph::DetectLoopAndComputeConstraint(unsigned int submap_id, unsigned int node_id)
{
    const TrajectoryNode::Data* constant_data;
    const Submap* submap;
    {
        if (!data_.submap_data.at(submap_id).submap->insertion_finished())
        {
            ROS_INFO("submap %d has not finished", submap_id);
            return;
        }

        if (submap_node_has_looped.count(submap_id))
        {
            return;
        }
        
        Eigen::Vector3d initial_relative_pose; // T_submap_node;
        initial_relative_pose = ComputeRelativePose(optimization_problem_->submap_data().at(submap_id).global_pose,
                                                    optimization_problem_->node_data().at(node_id).global_pose_2d);

        double distance = initial_relative_pose.head<2>().norm();
        if ( distance > 5)
        {
            return;
        }

        ROS_INFO("submap id: %d, node id: %d distance: %f: ", submap_id, node_id, distance);
        submap_node_has_looped.insert(std::make_pair(submap_id, node_id));
    }
    //     constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
    //     submap = static_cast<const Submap*>(data_.submap_data.at(submap_id).submap.get());
    //     Eigen::Vector3d pose_estimated = Eigen::Vector3d::Zero();
    //     Eigen::Vector3d initial_pose = constant_data->local_pose;
 
    // // FastCorrelativeScanMatcher2D* fast_correlative_scan_matcher = new FastCorrelativeScanMatcher2D(*submap->grid());
    // RealTimeCorrelativeScanMatcher real_csm;
    // float score = 0;
  
    // if(real_csm.Match(constant_data->local_pose,
    //                   constant_data->filtered_semantic_data,
    //                   *(submap->grid()),pose_estimated))
    // {
    //     running_optimization = true;
 
    //     // ROS_INFO("detect loop closure, submap_id: %d, node_id: %d, score: %f,\n predict node pose: %f %f %f  after fast-CSM: %f %f %f", submap_id, node_id, score,
    //     // initial_pose[0], initial_pose[1], initial_pose[2], pose_estimated[0], pose_estimated[1], pose_estimated[2]);
    //     Eigen::Vector3d constraint_transform = ComputeRelativePose(submap->local_pose(), pose_estimated);
    //     data_.constraints.push_back(Constraint{submap_id, node_id, 1.1e4, 1e5,
    //                                            constraint_transform, Constraint::Tag::Global});
    //     // RunOptimization();
    // }
    // // ROS_INFO("loop detect fail");
}

void PoseGraph::ComputeConstraintsForNode(const unsigned int node_id, std::vector<std::shared_ptr<const Submap>> insertion_submaps, const bool newly_finished_submap)
{
    if (running_optimization == true)
    {
        return;
    }
    

    std::vector<unsigned int> submap_ids;
    std::vector<unsigned int> finished_submap_ids;        // 处于完成状态的子图id的集合
    std::set<unsigned int> newly_finished_submap_node_ids;
    {
        submap_ids = InitializeGlobalSubmapPoses(insertion_submaps);

        Eigen::Vector3d node_local_pose = data_.trajectory_nodes.find(node_id)->second.constant_data->local_pose;
        optimization_problem_->AddTrajectoryNode(node_local_pose, node_local_pose);
    
        for (size_t i = 0; i < submap_ids.size(); i++)
        {
            data_.submap_data.find(submap_ids[i])->second.node_ids.emplace(node_id);
            Eigen::Vector3d relative_transform = ComputeRelativePose(insertion_submaps[i]->local_pose(), data_.trajectory_nodes.find(node_id)->second.constant_data->local_pose);
            data_.constraints.push_back(Constraint{submap_ids[i], node_id, 5e2, 1.6e3, relative_transform, Constraint::Tag::Non_Global});
        }

        for(auto& elem : data_.submap_data)
        {
            if (elem.second.state == SubmapState::kFinished)
            {
                finished_submap_ids.push_back(elem.first);
            }
        }

        if (newly_finished_submap)
        {
            data_.submap_data.find(submap_ids.front())->second.state = SubmapState::kFinished;
            newly_finished_submap_node_ids = data_.submap_data.find(submap_ids.front())->second.node_ids;
        }
    }

    for(auto& elem : finished_submap_ids)
    {
        DetectLoopAndComputeConstraint(elem, node_id);
    }

    // if (newly_finished_submap) 
    // {
    //     const unsigned int newly_finished_submap_id = submap_ids.front();

    //     for (const auto& node_id_data : optimization_problem_->node_data()) 
    //     {
    //         const unsigned int node_id = node_id_data.first;
    //         if (newly_finished_submap_node_ids.count(node_id) == 0) 
    //         {
    //             DetectLoopAndComputeConstraint(newly_finished_submap_id, node_id);
    //         }
    //     }
    // }

    // ROS_INFO("constraints size: %d", data_.constraints.size());

    // num_nodes_since_last_loop_closure_++;
    // if (num_nodes_since_last_loop_closure_>90)
    // {
    //     RunOptimization();
    // }
}

void PoseGraph::RunOptimization()
{

    const TrajectoryNode::Data* constant_data;
    const Submap* submap;
    for(auto elem : submap_node_has_looped)
    {
        constant_data = data_.trajectory_nodes.at(elem.second).constant_data.get();
        submap = static_cast<const Submap*>(data_.submap_data.at(elem.first).submap.get());
        Eigen::Vector3d pose_estimated = Eigen::Vector3d::Zero();
        Eigen::Vector3d initial_pose = constant_data->local_pose;
 
        RealTimeCorrelativeScanMatcher real_csm;
        float score = 0;
  
        if(real_csm.Match(constant_data->local_pose,
                        constant_data->filtered_semantic_data,
                        *(submap->grid()),pose_estimated) > 0.7)
        {
            running_optimization = true;
    
            Eigen::Vector3d constraint_transform = ComputeRelativePose(submap->local_pose(), pose_estimated);
            data_.constraints.push_back(Constraint{elem.first, elem.second, 1.1e4, 1e5,
                                                constraint_transform, Constraint::Tag::Global});
        }
    }



    if (optimization_problem_->submap_data().empty())
    {
        return;
    }

    optimization_problem_->Solve(data_.constraints);
    
    num_nodes_since_last_loop_closure_ = 0;

    const auto& submap_data = optimization_problem_->submap_data();
    const auto& node_data = optimization_problem_->node_data();

    nav_msgs::Path path_estimated_after_spa;

    for(const auto& elem : node_data)
    {
        data_.trajectory_nodes.at(elem.first).global_pose = elem.second.global_pose_2d;
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = elem.second.global_pose_2d[0];
        temp.pose.position.y = elem.second.global_pose_2d[1];
        temp.pose.orientation.z = elem.second.global_pose_2d[2];
        path_estimated_after_spa.poses.push_back(temp);
    }

    path_estimated_after_spa.header.frame_id = "world";
    path_estimated_after_spa_pub.publish(path_estimated_after_spa);
    data_.global_submap_poses_2d = submap_data;

    // update trajectory nodes
    Eigen::Vector3d local_to_global;
    for(const auto& node : node_data)
    {
        data_.trajectory_nodes.at(node.first).global_pose = node.second.global_pose_2d;
    }

    // Eigen::Matrix3d T_gloabl_local, T_global_node, T_local_node;
    
    // Eigen::Vector3d last_optimized_node_pose = std::prev(node_data.end())->second.global_pose_2d;
    // Eigen::Vector3d last_node_pose = std::prev(node_data.end())->second.local_pose_2d;

    // T_global_node << cos(last_optimized_node_pose[2]), -1*sin(last_optimized_node_pose[2]), last_optimized_node_pose[0],
    //                  sin(last_optimized_node_pose[2]), cos(last_optimized_node_pose[2]), last_optimized_node_pose[1],
    //                  0,0,1;

    // T_local_node << cos(last_node_pose[2]), -1*sin(last_node_pose[2]), last_node_pose[]

}


std::vector<unsigned int> PoseGraph::InitializeGlobalSubmapPoses(const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
{
    const auto& submap_data = optimization_problem_->submap_data();

    if (insertion_submaps.size() == 1)
    {
        if (submap_data.size()==0)
        {
            optimization_problem_->AddSubmap(insertion_submaps.front()->local_pose());
        }
        return {0};
    }
    
    unsigned int last_submap_id = std::prev(submap_data.end())->first;
    if (data_.submap_data.find(last_submap_id)->second.submap == insertion_submaps.front())
    {
        optimization_problem_->AddSubmap(insertion_submaps.back()->local_pose());
        return {last_submap_id, last_submap_id+1};
    }

    return {last_submap_id-1, last_submap_id};   
}


Eigen::Matrix4d PoseGraph::ComputeLocalToGlobalTransform()
{
    std::unique_lock<std::mutex> lock(mMutex_data_);

    if (data_.global_submap_poses.empty())
    {
        return Eigen::Matrix4d::Identity();
    }

    auto iter = data_.global_submap_poses.rbegin();
    
    Eigen::Matrix4d T_global_submap;
    T_global_submap << cos(iter->second[2]), -1*sin(iter->second[2]), 0, iter->second[0],
                       sin(iter->second[2]), cos(iter->second[2]), 0, iter->second[1],
                       0,0,1,0,
                       0,0,0,1;

    Eigen::Vector3d submap_local_pose = data_.submap_data.find(iter->first)->second.submap->local_pose();

    Eigen::Matrix4d T_local_submap;
    T_local_submap << cos(submap_local_pose[2]), -1*sin(submap_local_pose[2]), 0, submap_local_pose[0],
                       sin(submap_local_pose[2]), cos(submap_local_pose[2]), 0, submap_local_pose[1],
                       0,0,1,0,
                       0,0,0,1;  

    Eigen::Matrix4d ret = T_global_submap * T_local_submap.inverse();
    return ret;
}

std::map<unsigned int, TrajectoryNode> PoseGraph::getSubmapList()
{
    std::unique_lock<std::mutex> lock(mMutex_data_);
    return data_.trajectory_nodes;
}

} // namespace mapping
} // namespace AVP
