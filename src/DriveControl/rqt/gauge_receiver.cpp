#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <stdio.h>

float vals [2]; //declare the variable for angle and velocity

void gaugesCallback(const geometry_msgs::Twist& msg){ // Callback function for publisher
        ROS_INFO("Turn: %f, Velocity: %f", msg.linear.x, msg.angular.z); //ros messages
        vals[0]=msg.linear.x;
        vals[1]=msg.angular.z;
}
        
int main(int argc, char**argv){
        ros::init(argc, argv, "gauges");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("drive", 1000, &gaugesCallback);
        
        // subscribe to drive_sender node in drive_sender.cpp
        ros::Publisher  vel_pub = n.advertise<std_msgs::Float64>("gauge_velocity_sender", 1000);
        ros::Publisher  ang_pub = n.advertise<std_msgs::Float64>("gauge_angle_sender", 1000);
        // make publishers for each of the two messages

        ros::Rate loop_rate(10);
        //update at 10Hz

        while (ros::ok()){
                std_msgs::Float64 velocity;
                std_msgs::Float64 angle;
                velocity.data=vals[0];
                velocity.data=velocity.data/2.55;
                //normalize velocity to +/- 100 so it fits nicer on the gauge
                angle.data=vals[1];
                angle.data=angle.data*90;
		//normalize angle to +/- 90

                vel_pub.publish(velocity);
                ros::spinOnce();
                //publish velocity and wait until done
                ang_pub.publish(angle);
                ros::spinOnce();
                //publish angle and wait until done

                loop_rate.sleep();
        }
        return 0;
}
