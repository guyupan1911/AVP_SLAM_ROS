#ifndef OPTIMIZATION_PROBLEM_H
#define OPTIMIZATION_PROBLEM_H

#include <Eigen/Core>
#include <map>
#include <vector>
#include <ceres/ceres.h>
#include "constraint.h"
#include "spa_cost_function.h"

namespace AVP
{
namespace mapping
{
    
struct NodeSpec2D 
{
    Eigen::Vector3d local_pose_2d;
    Eigen::Vector3d global_pose_2d;
};

struct SubmapSpec2D 
{
    Eigen::Vector3d global_pose;
};

class OptimizationProblem
{
    public:

        void AddSubmap(const Eigen::Vector3d& global_submap_pose)
        {
            submap_data_.insert(std::make_pair(submap_data_.size(), SubmapSpec2D{global_submap_pose}));
        }

        void AddTrajectoryNode(const Eigen::Vector3d& local_node_pose, const Eigen::Vector3d& global_node_pose)
        {
            node_data_.insert(std::make_pair(node_data_.size(), NodeSpec2D{local_node_pose, global_node_pose}));
        }

        const std::map<unsigned int, NodeSpec2D>& node_data() const
        {
            return node_data_;
        }

        const std::map<unsigned int, SubmapSpec2D>& submap_data() const
        {
            return submap_data_;
        }

        void Solve(std::vector<Constraint>& constraints);

    private:
        std::map<unsigned int, NodeSpec2D> node_data_;
        std::map<unsigned int, SubmapSpec2D> submap_data_;
};

} // namespace mapping
} // namespace AVP


#endif