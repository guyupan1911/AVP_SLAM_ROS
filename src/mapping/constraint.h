#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <Eigen/Core>

namespace AVP
{
    
namespace mapping
{
    
template<typename T>
T normalize(const T& angle)
{
    T ret = angle;
    while (ret >= M_PI)
    {
        ret -= 2*M_PI;
    }
    
    while (ret < -1*M_PI)
    {
        ret += 2*M_PI;
    }
    
    return ret;
}

struct Constraint 
{
    unsigned int submap_id;  
    unsigned int node_id;    

    double translation_weight;
    double rotation_weight;

    Eigen::Vector3d relative_pose;

    enum Tag { Global, Non_Global } tag; // 局部约束或者全局约束
};
    
Eigen::Vector3d ComputeRelativePose(const Eigen::Vector3d& submap_global_pose, const Eigen::Vector3d& node_global_pose);

} // namespace mapping
} // namespace AVP


#endif