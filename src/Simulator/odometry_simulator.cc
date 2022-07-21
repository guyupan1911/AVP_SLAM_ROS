#include "odometry_simulator.h"
#include <ros/console.h>
#include <random>
#include <chrono>

namespace AVP{
    namespace simulating{

        void OdometrySimulator::Update()
        {
            current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            //ROS_DEBUG("dt:%f Speed:%f SteeringAngle:%f", dt, vehicle_speed, steering_angle);
            last_time = current_time;
            
            //update pose true            
            float vx = vehicle_speed * cos(vehicle_pose_true.pose.orientation.z);
            float vy = vehicle_speed * sin(vehicle_pose_true.pose.orientation.z);
            float rtheta = vehicle_speed * tan(steering_angle) / L;

            vehicle_pose_true.pose.position.x += (vx * dt);
            vehicle_pose_true.pose.position.y += (vy * dt);
            vehicle_pose_true.pose.orientation.z += (rtheta * dt);
            remapAngle(vehicle_pose_true.pose.orientation.z);
            vehicle_pose_true.header.stamp = current_time;

            // update pose noised;
            // add gaussian noise to speed and steering angle;
            auto seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine gen(seed);
            std::normal_distribution<float> speed_noise(0,0.05);
            std::normal_distribution<float> steeringAngle_noise(0,0.1);

            float vehicle_speed_noised = vehicle_speed + speed_noise(gen);
            float steering_Angle_noised = steering_angle + steeringAngle_noise(gen);

            float vx_noised = vehicle_speed_noised * cos(vehicle_pose_noised.pose.orientation.z);
            float vy_noised = vehicle_speed_noised * sin(vehicle_pose_noised.pose.orientation.z);
            float rtheta_noised = vehicle_speed_noised * tan(steering_Angle_noised) / L;

            vehicle_pose_noised.pose.position.x += (vx_noised * dt)*(1+speed_noise(gen));
            vehicle_pose_noised.pose.position.y += (vy_noised * dt)*(1+steeringAngle_noise(gen));
            vehicle_pose_noised.pose.orientation.z += (rtheta_noised * dt);
            remapAngle(vehicle_pose_noised.pose.orientation.z);
            vehicle_pose_noised.header.stamp = current_time;

            ground_truth_path->poses.emplace_back(vehicle_pose_true);
            ground_truth_path->header.stamp = current_time;
        }
    }
}