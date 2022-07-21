#include <ros/ros.h>
#include "avp/AddTwoInts.h"

bool add(avp::AddTwoInts::Request &req,
         avp::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  //ROS_DEBUG("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_DEBUG("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    //ROS_DEBUG("Ready to add two ints.");
    ros::spin();

    return 0;
}