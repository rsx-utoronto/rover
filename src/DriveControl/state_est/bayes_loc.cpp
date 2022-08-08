#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "ar_track_alvar/Kalman.h"
#include "ar_track_alvar/Util.h"
#include "ar_track_alvar/Alvar.h"

using namespace message_filters;
class StateLoc {
	public:
		StateLoc();
	private:
		void BayesLoc(const nav_msgs::Odometry::ConstPtr& odom_msg);
		void camCallback(const sensor_msgs::Image);

		ros::NodeHandle nh_;
 
		ros::Publisher loc_pub_; 
		message_filters::Subscriber<nav_msgs::Odometry> odom_sub_; // maybe change to GPS topic ?
		message_filters::Subscriber<sensor_msgs::Image> cam_sub_; // need to check this topic name 
		message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_; // need to check this too
		TimeSynchronizer<Image, CameraInfo> sync(cam_sub_, info_sub_, 10);


};

void camCallback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info){

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bayes_loc");
	StateLoc bayes_loc;

	ros::Publisher loc_pub_; 
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub_; // maybe change to GPS topic ?
	message_filters::Subscriber<sensor_msgs::Image> cam_sub_; // need to check this topic name 
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_; // need to check this too
	message_filters::Subscriber<ar_track_alvar::AlvarMarkers> marker_sub_; // will need to subscribe to marker info
	TimeSynchronizer<Image, CameraInfo> sync(cam_sub_, info_sub_, 10);
	sync.RegisterCallback(boost::bind(&camCallback, _1, _2));
	
	ros::spin();
}









