#ifndef ODOMETRY_SIMULATOR_
#define ODOMETRY_SIMULATOR_
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <ros/time.h>
#include <mutex>


namespace AVP{
namespace simulating{

// simulates vehicle dynamics, outputs poses
class OdometrySimulator{
public:
OdometrySimulator(){
    last_time = ros::Time::now();
    current_time = ros::Time::now();
    // initialize pose
    vehicle_pose_noised.header.stamp = ros::Time::now();
    vehicle_pose_noised.header.frame_id = "world";
    vehicle_pose_noised.pose.position.x = 0.f;
    vehicle_pose_noised.pose.position.y = 0.f;
    vehicle_pose_noised.pose.orientation.z = 0.f;


    vehicle_pose_true.header.stamp = ros::Time::now();
    vehicle_pose_true.header.frame_id = "world";
    vehicle_pose_true.pose.position.x = 0.f;
    vehicle_pose_true.pose.position.y = 0.f;
    vehicle_pose_true.pose.orientation.z = 0.f;

    ground_truth_path.reset(new nav_msgs::Path());
    ground_truth_path->header.frame_id = "world";
}

void SetSpeed(float& speed){
    std::unique_lock<std::mutex> lock(mMutex_speed_steeringAngle);
    vehicle_speed = speed;
}
void SetSteeringAngle(float& steeringAngle){
    std::unique_lock<std::mutex> lock(mMutex_speed_steeringAngle);
    steering_angle = steeringAngle;
}

geometry_msgs::PoseStamped getPoseNoised(){
    std::unique_lock<std::mutex> lock(mMutex_pose);
    return vehicle_pose_noised;
}

geometry_msgs::PoseStamped getPoseTrue()
{
    std::unique_lock<std::mutex> lock(mMutex_pose);
    return vehicle_pose_true;
}

nav_msgs::Path::ConstPtr getPathTrue()
{
    std::unique_lock<std::mutex> lock(mMutex_pose);
    return ground_truth_path;
}

void Update();

private:

void remapAngle(double& angle)
{
    while (true)
    {
        if (-1*M_PI < angle && angle <= M_PI)
        {
            return;
        }
        else if (angle <= (-1*M_PI))
        {
            angle += 2*M_PI;
        }
        else if (angle > M_PI)
        {
            angle -= 2*M_PI;
        }
    }
}

std::mutex mMutex_speed_steeringAngle;
float vehicle_speed = 0.f;
float steering_angle = 0.f;

const float L = 1.5;

std::mutex mMutex_pose;
geometry_msgs::PoseStamped vehicle_pose_noised;
geometry_msgs::PoseStamped vehicle_pose_true;
nav_msgs::PathPtr ground_truth_path;

ros::Time last_time, current_time;
};

} // name simulating
} // namespace AVP

#endif
              