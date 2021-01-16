// A program that publishes mock (randomly - generated) velocity
// messages of type geometry_msgs::Twist

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
	// Initialize ROS system and become a node.
	ros::init(argc, argv, "publish_velocity");
	ros::NodeHandle nh;
	
	// Create a publisher object that sends messages
	// to "drive" topic of message type geometry_msgs::Twist
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
	"drive", 1000);
	
	// Seeds the random number generator
	srand(time(0));
	
	
	ros::Rate rate(2);
	while(ros::ok()){
		
		geometry_msgs::Twist msg;
		// linear.x will have a value between 0 and 1
		msg.linear.x = double(rand()) / double(RAND_MAX);
		// angular.z will have a value between -1 and 1
		msg.angular.z = 2*double(rand()) / double(RAND_MAX) - 1;
		
		// publishes 
		pub.publish(msg);
		
		ROS_INFO_STREAM("Sending mock velocity values: "
			<< " linear=" << msg.linear.x
			<< " angular=" << msg.angular.z);
		
		// Waits until another iteration
		rate.sleep();
	}
}