#include "constraint.h"

namespace AVP
{
namespace mapping
{
    
Eigen::Vector3d ComputeRelativePose(const Eigen::Vector3d& submap_global_pose, const Eigen::Vector3d& node_global_pose)
{
    double tx = node_global_pose[0] - submap_global_pose[0];
    double ty = node_global_pose[1] - submap_global_pose[1];
    double r = normalize(node_global_pose[2] - submap_global_pose[2]);

    double cz = cos(submap_global_pose[2]);
    double sz = sin(submap_global_pose[2]);

    return {cz*tx+sz*ty,
            -1*sz*tx+cz*ty,
            r};
}


} // namespace mapping
} // namespace AVP
