#include "simulating_node.h"
#include "unistd.h"

namespace AVP
{
    namespace simulating
    {

        Node::Node()
        {
            pHdMap_.reset(new HDMAP());
            pOdoSimulator_.reset(new OdometrySimulator());

            it = new image_transport::ImageTransport(node_handle_);

            im_pub = it->advertise("map/path",1);
            odometry_pose_true_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("odometry_true", 1000);
            odometry_pose_noise_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("odometry_noised", 1000);
            all_semantic_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("all_semantic_points",1000);
            scan_semantic_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("scan_semantic_points",1000);
            ground_truth_path_publisher_ = node_handle_.advertise<nav_msgs::Path>("ground_truth_path", 1000);

            vehicle_speed_subscriber_ = node_handle_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Node::cmdVelCallback, this);

            service_server_.push_back(node_handle_.advertiseService("SaveMap", &Node::HandleSaveMap, this));
            timers_.push_back(node_handle_.createTimer(ros::Duration(5),&Node::PublishAllSemanticPoints,this));
            timers_.push_back(node_handle_.createTimer(ros::Duration(0.05),&Node::PublishScanSemanticPoints,this));
            timers_.push_back(node_handle_.createTimer(ros::Duration(0.02),&Node::PublishOdometryPoseTure,this));
            timers_.push_back(node_handle_.createTimer(ros::Duration(0.02),&Node::PublishOdometryPoseNoise,this));
            timers_.push_back(node_handle_.createTimer(ros::Duration(0.02),&Node::PublishImagePath,this));
            timers_.push_back(node_handle_.createTimer(ros::Duration(0.02),&Node::PublishGroundTruthPath, this));
        }

        void Node::PublishOdometryPoseTure(const ros::TimerEvent& unused_timer_event)
        {
            odometry_pose_true_publisher_.publish(pOdoSimulator_->getPoseTrue());
        }
        
        void Node::PublishOdometryPoseNoise(const ros::TimerEvent& unused_timer_event)
        {
            odometry_pose_noise_publisher_.publish(pOdoSimulator_->getPoseNoised());
        }

        void Node::PublishAllSemanticPoints(const ros::TimerEvent& unused_timer_event)
        {
            all_semantic_points_publisher_.publish(pHdMap_->getAllSemanticPoints());
        }

        void Node::PublishScanSemanticPoints(const ros::TimerEvent& unused_timer_event)
        {
            scan_semantic_points_publisher_.publish(pHdMap_->getCurrentSemanticPoints(pOdoSimulator_->getPoseTrue()));
        }

        void Node::PublishGroundTruthPath(const ros::TimerEvent& unused_timer_event)
        {
            ground_truth_path_publisher_.publish(pOdoSimulator_->getPathTrue());
        }
        void Node::PublishImagePath(const ros::TimerEvent& unused_timer_event)
        {
            im_pub.publish(pHdMap_->drawPath(pOdoSimulator_->getPoseTrue(), pOdoSimulator_->getPoseNoised()));
        }
        
        bool Node::HandleSaveMap(avp::SaveMap::Request& request, avp::SaveMap::Response& response)
        {
            if (request.save == true)
            {
                pHdMap_->savePath();
                response.saved = true;
                run_flag = false;
                return true;
            }
            return false;
        }

        void Node::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
        {
            if (pOdoSimulator_ == nullptr)
            {
                //ROS_DEBUG("odometry simulator hasn't been initialized.");
                return;
            }
            
            float speed = msg->linear.x;
            float steeringAngle = msg->angular.z;
            ROS_DEBUG("steering_angle: %f", steeringAngle);
            ROS_DEBUG("speed: %f", speed);

            pOdoSimulator_->SetSpeed(speed);
            pOdoSimulator_->SetSteeringAngle(steeringAngle);
            return;
        }

        void Node::Run()
        {
            if(run_flag)
            {
                pOdoSimulator_->Update();
                return;
            }
            ros::shutdown();
        }

    } // namespace simulating
    
} // namespace AVP
