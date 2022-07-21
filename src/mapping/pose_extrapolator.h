#ifndef POSE_EXTRAPOLATOR_H
#define POSE_EXTRAPOLATOR_H

#include <Eigen/Core>
#include <deque>
#include <mutex>

namespace AVP
{
namespace mapping
{
    
class PoseExtrapolator
{
    public:
    void AddPose(const Eigen::Vector3d& pose);

    Eigen::Vector3d PredictPose();
    
    void AddOdometry(const Eigen::Vector3d& odometry_pose);
    
    Eigen::Vector3d last_odometry_pose_;

    private:
    std::mutex mMutexQueue;
    std::deque<Eigen::Vector3d> timed_pose_queue_;
    std::deque<Eigen::Vector3d> odometry_pose_queue_;
};

} // namespace mapping

} // namespace AVP


#endif