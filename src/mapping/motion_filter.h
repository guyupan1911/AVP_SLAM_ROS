#ifndef MOTION_FILTER_H
#define MOTION_FILTER_H

#include <Eigen/Core>
#include "ros/ros.h"

namespace AVP
{
namespace mapping
{
    
class MotionFilter
{
    public:
        bool IsSimilar(const Eigen::Vector3d& pose);
    private:
    Eigen::Vector3d last_pose_;
    unsigned int total_ = 0;
};

} // namespace mapping

} // namespace AVP


#endif