#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread.hpp>
#include <sensor_msgs/Joy.h>


class TeleopRover {
	public:
		TeleopRover();
		void publishDrive();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

		ros::NodeHandle nh_;
		float MAX_ANGULAR_SPEED = 0.4;
		float MAX_LINEAR_SPEED = 0.6;
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

	drive_pub_ = nh_.advertise<geometry_msgs::Twist>("drive", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRover::joyCallback, this);
}

void TeleopRover::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;

	//indexs for controller values
	int R2 = 5;
	int L2 = 2;
	int LS = 0;

	//Values from Controller
	double posThrottle = joy->axes[R2];
	double negThrottle = joy->axes[L2];
	float turnFactor = static_cast<float>(joy->axes[LS]);
	double lin_vel;

	double dispVal = 0;

	//Encoding Values for Throttle
	if (posThrottle < 1 && negThrottle < 1){
		dispVal = 0;
		lin_vel =  0;

	} else if (posThrottle < 1){
		ROS_INFO("in Pos throttle");
		dispVal = 255 - (posThrottle+1)*127.5;
		lin_vel = 255 - (posThrottle+1)*127.5;
	} else if (negThrottle < 1){
		ROS_INFO("in neg throttle");
		dispVal = -1*(255 - (negThrottle+1)*127.5);
		lin_vel = -1*(255 - (negThrottle+1)*127.5);
	} else {
		dispVal = 0;
		lin_vel = 0;
	}

	twist.linear.x = lin_vel/255*MAX_LINEAR_SPEED;
	twist.angular.z = turnFactor/1*MAX_ANGULAR_SPEED;

	ROS_INFO("Turn Factor %f", turnFactor);
	ROS_INFO("Motor Value %f", lin_vel);
	drive_pub_.publish(twist);
}

void TeleopRover::publishDrive(){
	geometry_msgs::Twist twist;
	// if (cnt < 100000)
	// 	twist.linear.x = 0.3;
	// if ( cnt > 100000 && cnt < 200000)
	// 	twist.linear.x = 0.0;
	// if (cnt == 200000)
	// 	cnt = 0;
	twist.linear.x = 0.5;
	twist.angular.z = 0.0;

	drive_pub_.publish(twist);
	cnt ++;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_sender_falcon");
	TeleopRover drive_sender;
	// while (ros::ok()){
	// 	drive_sender.publishDrive();
	// }
	ros::spin();
}
