#ifndef NODE_H
#define NODE_H

#include "odometry_simulator.h"
#include "hdMap.h"
#include <memory>
#include <ros/ros.h>
#include <avp/SaveMap.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

namespace AVP
{
    namespace simulating
    {

        class Node{

            public:
            Node();
            
            bool run_flag = true;
            void Run();

            private:

            void PublishOdometryPoseTure(const ros::TimerEvent& unused_timer_event);
            void PublishOdometryPoseNoise(const ros::TimerEvent& unused_timer_event);
            void PublishAllSemanticPoints(const ros::TimerEvent& unused_timer_event);
            void PublishScanSemanticPoints(const ros::TimerEvent& unused_timer_event);
            void PublishGroundTruthPath(const ros::TimerEvent& unused_timer_event);
            void PublishImagePath(const ros::TimerEvent& unused_timer_event);
            bool HandleSaveMap(avp::SaveMap::Request& request, avp::SaveMap::Response& response);



            void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);


            std::vector<ros::ServiceServer> service_server_;

            ros::NodeHandle node_handle_;
            image_transport::ImageTransport* it;

            image_transport::Publisher im_pub;
            ros::Publisher odometry_pose_true_publisher_;
            ros::Publisher odometry_pose_noise_publisher_;
            ros::Publisher all_semantic_points_publisher_;
            ros::Publisher scan_semantic_points_publisher_;
            ros::Publisher ground_truth_path_publisher_;

            ros::Subscriber vehicle_speed_subscriber_;

            std::unique_ptr<HDMAP> pHdMap_;
            std::unique_ptr<OdometrySimulator> pOdoSimulator_;

            std::vector<ros::Timer> timers_;

        };

    } // namespace simulating
} // namespace AVP


#endif