#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread.hpp>
#include <sensor_msgs/Joy.h>

float linear = 0.0;
float angular = 0.0;
int linear_ = 1;
int angular_ = 2;

class TeleopRover {
	public:
		TeleopRover();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

		ros::NodeHandle nh_;

		int linear_, angular_, right_left_, forward_backward_, yaw_; 
		double l_scale_, a_scale_;
		ros::Publisher vel_pub_; 
		ros::Subscriber joy_sub_;
};

TeleopRover::TeleopRover():
	linear_(1),
	angular_(2)
{
	nh_.param("axis_linear", linear_, linear_);
	nh_.param("axis_angular", angular_, angular_);
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("drive", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRover::joyCallback, this);
}

void TeleopRover::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	twist.angular.z = a_scale_*joy->axes[angular_];
	twist.linear.x = l_scale_*joy->axes[linear_];
	ROS_INFO("%f", joy->axes[linear_]);
	ROS_INFO("%f", joy->axes[angular_]);
	vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_sender");
	TeleopRover drive_sender;
	
	ros::spin();
}
