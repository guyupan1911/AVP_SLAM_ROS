#include "optimization_problem.h"

namespace AVP
{
    
namespace mapping
{
    
std::array<double, 3> FromPose(const Eigen::Vector3d& pose) 
{
  return {{pose[0], pose[1], normalize(pose[2])}};
}

Eigen::Vector3d ToPose(const std::array<double, 3>& values) 
{
  return Eigen::Vector3d{values[0], values[1], values[2]};
}

void OptimizationProblem::Solve(std::vector<Constraint>& constraints)
{
    if (node_data_.empty())
    {
        return;
    }
    
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    std::map<unsigned int, std::array<double, 3>> C_submaps;
    std::map<unsigned int, std::array<double, 3>> C_nodes;
    bool first_submap = true;

    for(const auto& elem : submap_data_)
    {
        C_submaps.insert(std::make_pair(elem.first, FromPose(elem.second.global_pose)));
        problem.AddParameterBlock(C_submaps.at(elem.first).data(),3);
        if (first_submap)
        {
            problem.SetParameterBlockConstant(C_submaps.at(elem.first).data());
            first_submap = false;
        }
    }

    for(const auto& elem : node_data_)
    {
        C_nodes.insert(std::make_pair(elem.first, FromPose(elem.second.local_pose_2d)));
        problem.AddParameterBlock(C_nodes.at(elem.first).data(),3);
        if (first_submap)
        {
            problem.SetParameterBlockConstant(C_nodes.at(elem.first).data());
            first_submap = false;
        }
    }

    for (const Constraint& constraint : constraints) 
    {
        problem.AddResidualBlock(
            CreateAutoDiffSpaCostFunction(constraint),
            nullptr,
            C_submaps.at(constraint.submap_id).data(),
            C_nodes.at(constraint.node_id).data()
        );
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);

    for(const auto& C_submap_id_data : C_submaps)
    {
        submap_data_.at(C_submap_id_data.first).global_pose = ToPose(C_submap_id_data.second);
    }
    for(const auto& C_node_id_data : C_nodes)
    {
        node_data_.at(C_node_id_data.first).global_pose_2d = ToPose(C_node_id_data.second);
    }
}

} // namespace mapping


} // namespace AVP
