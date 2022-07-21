#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <ros/time.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include "avp/SaveMap.h"

// Init variables
float speed(0.f); // Linear velocity (m/s)
float turn(0.f); // Angular velocity (rad/s)
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();
  if (ch == '\x03')
  {
	  return ch;
  }
  
  if (ch == '\033')
  {
    ch = getchar();
    if (ch == '[')
    {
      ch = getchar();
    }
  }
  
  
  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "teleop_twist_keyboard");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::ServiceClient savePath = nh.serviceClient<avp::SaveMap>("SaveMap");


  // Create Twist message
  geometry_msgs::Twist twist;

  while(true){

    // Get the pressed key
    key = getch();

    if (key == 'A')
    {
    	// move foward
		speed += 1.f;

		twist.linear.x = speed;
		twist.linear.y = 0.f;
		twist.linear.z = 0.f;
		twist.angular.x = 0.f;
		twist.angular.y = 0.f;
		twist.angular.z = turn;
		pub.publish(twist);
    }
	else if (key == 'B')
	{
		// move backward
		speed -= 1.f;
		
		twist.linear.x = speed;
		twist.linear.y = 0.f;
		twist.linear.z = 0.f;
		twist.angular.x = 0.f;
		twist.angular.y = 0.f;
		twist.angular.z = turn;
		pub.publish(twist);
	}
	else if (key == 'D')
	{
		//turn left
		turn += 0.25;
		
		twist.linear.x = speed;
		twist.linear.y = 0.f;
		twist.linear.z = 0.f;
		twist.angular.x = 0.f;
		twist.angular.y = 0.f;
		twist.angular.z = turn;
		pub.publish(twist);
	}
	else if (key == 'C')
	{
		// turn right
		turn -= 0.25;
		
		twist.linear.x = speed;
		twist.linear.y = 0.f;
		twist.linear.z = 0.f;
		twist.angular.x = 0.f;
		twist.angular.y = 0.f;
		twist.angular.z = turn;
		pub.publish(twist);
	}
	else if (key == '\x03')
	{	
		break;
	}
	else if (key == 's')
	{
		avp::SaveMap save;
		save.request.save = true;
		savePath.call(save);
	}
	
    ros::spinOnce();
  }

  return 0;
}