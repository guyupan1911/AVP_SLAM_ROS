#include "simulating_node.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator_node");
    AVP::simulating::Node node;

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        node.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;    
}