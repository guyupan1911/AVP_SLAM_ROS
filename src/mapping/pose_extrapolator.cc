#include "pose_extrapolator.h"

namespace AVP
{
namespace mapping
{

    void PoseExtrapolator::AddPose(const Eigen::Vector3d& pose)
    {   
        std::unique_lock<std::mutex> lock(mMutexQueue);
        timed_pose_queue_.push_back(pose);
        if (odometry_pose_queue_.empty())
        {
            last_odometry_pose_[0] = 0.f;
            last_odometry_pose_[1] = 0.f;
            last_odometry_pose_[2] = 0.f;
        }
        else
        {
            last_odometry_pose_[0] = odometry_pose_queue_.back()[0];
            last_odometry_pose_[1] = odometry_pose_queue_.back()[1];
            last_odometry_pose_[2] = odometry_pose_queue_.back()[2];
        }
    }

    void PoseExtrapolator::AddOdometry(const Eigen::Vector3d& odometry_pose)
    {   
        std::unique_lock<std::mutex> lock(mMutexQueue);   
        odometry_pose_queue_.push_back(odometry_pose);
    }

    Eigen::Vector3d PoseExtrapolator::PredictPose()
    { 
        std::unique_lock<std::mutex> lock(mMutexQueue);   

        Eigen::Vector3d current_odometry_pose = odometry_pose_queue_.back();
        Eigen::Vector3d last_stored_pose = timed_pose_queue_.back(); 

        Eigen::Vector3d ret;
        ret = last_stored_pose + (current_odometry_pose - last_odometry_pose_);

        return ret;
    }

} // namespace mapping
} // namespace AVP
