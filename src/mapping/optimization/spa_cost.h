#ifndef SPA_COST_H
#define SPA_COST_H

#include "ceres/ceres.h"
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

std::vector<double> calculate_relative_pose(std::array<double,3> p1, std::array<double,3> p2)
{
    std::vector<double> ret(3);
    ret[0] = cos(p1[2])*(p2[0]-p1[0])+sin(p1[2])*(p2[1]-p1[1]);
    ret[1] = -1*sin(p1[2])*(p2[0]-p1[0])+cos(p1[2])*(p2[1]-p1[1]);
    ret[2] = normalize(p2[2]-p1[2]);

    return ret;
}


template <typename T>
std::array<T, 3> ComputeUnscaledError(
    const Eigen::Vector3d& relative_pose, const T* const start,
    const T* const end) {
  // 旋转矩阵R
  const T cos_theta_i = cos(start[2]);
  const T sin_theta_i = sin(start[2]);
  const T delta_x = end[0] - start[0]; // t2 -t1
  const T delta_y = end[1] - start[1];
  const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y, // R.inverse * (t2 -t1)
                  -sin_theta_i * delta_x + cos_theta_i * delta_y,
                  end[2] - start[2]};
  return {{T(relative_pose[0]) - h[0],
           T(relative_pose[1]) - h[1],
           normalize(T(relative_pose[2]) - h[2])}};
}

struct CostFunction
{
    CostFunction(const Eigen::Vector3d& z, double cov):z_(z), cov_(cov){}

    template<typename T>
    bool operator()(const T* const start_pose, const T* const end_pose, T* e) const
    {
    std::array<T, 3> error = ComputeUnscaledError(z_,start_pose, end_pose);
    error[0] /= cov_;
    error[1] /= cov_;
    error[2] /= cov_;

    // c++11: std::copy 拷贝元素到e中
    std::copy(std::begin(error), std::end(error), e); 
    return true;
    }

    private:
        Eigen::Vector3d z_;
        double cov_;
};

class constraint
{;
public:
    enum class constraint_type{inter, global};

    constraint(unsigned int submap_id, unsigned int node_id, const Eigen::Vector3d& submap_pose, const Eigen::Vector3d& node_pose, constraint_type type)
    :submap_id_(submap_id), node_id_(node_id), submap_pose_(submap_pose), node_pose_(node_pose), type_(type)
    {
        observerd_relative_pose[0] = cos(submap_pose_[2])*(node_pose_[0]-submap_pose_[0])+sin(submap_pose_[2])*(node_pose_[1]-submap_pose_[1]);
        observerd_relative_pose[1] = -1*sin(submap_pose_[2])*(node_pose_[0]-submap_pose_[0])+cos(submap_pose_[2])*(node_pose_[1]-submap_pose_[1]);
        observerd_relative_pose[2] = normalize(node_pose_[2]-submap_pose_[2]);
    }

    unsigned int submap_id_, node_id_;
    Eigen::Vector3d submap_pose_, node_pose_;
    constraint_type type_;
    double cov_;
    Eigen::Vector3d observerd_relative_pose;
};



} // namespace mapping
} // namespace AVP


#endif