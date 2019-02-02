#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <sstream>
#include <iostream>
#include <geometry_msgs/Twist.h>

float linear = 0.0;
float angular = 0.0;

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
/* Logitech Joystick
Axes:
[0] - angular main (left & right)
[1] - linear (up & down)
[2] - twist (left & right)
[3] - bottom toggle (+ and -)
[4] - angular small (left & right)
[5] - linear small (up & down)

Buttons:
[0] - 1 (stick)
 ... (index = button number - 1)
buttons 7 & 9 are unreliable
*/	linear = msg->axes[1];
	angular = msg->axes[0];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joy", 1, chatterCallback);
  ros::Publisher joystick = n.advertise<geometry_msgs::Twist>("drive",1);
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
  	geometry_msgs::Twist joy;
  	joy.linear.x = linear;
	joy.angular.z= angular;
	joystick.publish(joy);
	ros::spinOnce();
	loop_rate.sleep();
  }
  return 0;
}

