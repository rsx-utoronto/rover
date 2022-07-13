#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
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
		// void publishDrive();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

		ros::NodeHandle nh_;

		int linear_, angular_, right_left_, forward_backward_, yaw_; 
		double l_scale_, a_scale_;
		ros::Publisher drive_pub_;
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

	// vel_pub_ = nh_.advertise<geometry_msgs::Twist>("drive", 1);
	drive_pub_ = nh_.advertise<std_msgs::String>("drive", 10);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRover::joyCallback, this);
}

void TeleopRover::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	// geometry_msgs::Twist twist;
	std_msgs::String drive_msg;
	drive_msg.data = "d";
	float lin_vel;
	//indexs for controller values
	int R2 = 5;
	int L2 = 2;
	int LS = 0;

	//Values from Controller
	double posThrottle = joy->axes[R2];
	double negThrottle = joy->axes[L2];
	float turnFactor = static_cast<float>(joy->axes[LS]);

	double dispVal = 0;

	//Encoding Values for Throttle
	if (posThrottle < 1 && negThrottle < 1){
		dispVal = 0;
		// twist.linear.x = 0;
		lin_vel =  0;

	} else if (posThrottle < 1){
		ROS_INFO("in Pos throttle");
		dispVal = 255 - (posThrottle+1)*127.5;
		// twist.linear.x = 255 - (posThrottle+1)*127.5;
		lin_vel = 120;
	} else if (negThrottle < 1){
		ROS_INFO("in neg throttle");
		dispVal = -1*(255 - (negThrottle+1)*127.5);
		// twist.linear.x = -1*(255 - (negThrottle+1)*127.5);
		lin_vel = -120;
	} else {
		dispVal = 0;
		lin_vel = 0;
	}

	drive_msg.data += "a";
	drive_msg.data += std::to_string(turnFactor);

	drive_msg.data += "l";
	drive_msg.data += std::to_string(lin_vel);

	ROS_INFO("Turn Factor %f", turnFactor);
	ROS_INFO("Motor Value %f", lin_vel);
	drive_pub_.publish(drive_msg);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_sender");
	TeleopRover drive_sender;
	ros::spin();
}
