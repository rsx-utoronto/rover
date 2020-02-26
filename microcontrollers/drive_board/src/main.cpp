#include <Arduino.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Byte.h>

#include <Wire.h>
#include <rsx_esc.h>
#include <main.h>

// Allocate 128 bytes to hold ros loggin messages
char log_buffer[128];

// Setup all the ROS stuff
ros::NodeHandle nh;

// Set up subscribers
geometry_msgs::Twist teleop_twist;
ros::Subscriber<geometry_msgs::Twist> teleop_sub("teleop", &teleop_cb);
std_msgs::Byte motor_faults_msg;
ros::Publisher motor_faults_pub("rover/motor_faults", &motor_faults_msg);


void setup() {
	Wire.begin();
	nh.initNode();
	nh.advertise(motor_faults_pub);
}

//   __  __      _        _                  
//  |  \/  |__ _(_)_ _   | |   ___  ___ _ __ 
//  | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ \
//  |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
//                                     |_|   

void loop() {
	nh.spinOnce();
	// Declare these objects here because ROS does not like to have
	// global objects for some reason
	static ESC Drivers[6] = {
		ESC(2340.0, 24, 25, 8, 22, 23, B0001001),
		ESC(2340.0, 26, 27, 9, 28, 29, B0001101),
		ESC(2340.0, 33, 32, 7, 31, 30, B0001000),
		ESC(2340.0, 35, 34, 4, 37, 36, B0001100),
		ESC(2340.0, 39, 38, 5, 41, 40, B0001110),
		ESC(2340.0, 45, 44, 6, 43, 42, B1001100)
	};

	// Only run this code once
	static bool first_run = true;
	if(first_run) {
		nh.loginfo("Arduino booted up");
		reset_driver_faults(Drivers);
		first_run = false;
	}
}

//   ___             _   _             
//  | __|  _ _ _  __| |_(_)___ _ _  ___
//  | _| || | ' \/ _|  _| / _ \ ' \(_-<
//  |_| \_,_|_||_\__|\__|_\___/_||_/__/
                                    

void reset_driver_faults(ESC Drivers[6]) {
	// Call this if the motors fault
	for(int i = 0; i < 6; i++) {
		Drivers[i].set_enable(false);
	}
	delay(100);
	for(int i = 0; i < 6; i++) {
		Drivers[i].set_enable(true);
	}
	delay(100);
}

void set_all_vel(float vel, ESC Drivers[6]) {
	for(int i = 0; i < 6; i++) {
		Drivers[i].set_vel(vel);
	}
}

uint8_t check_motor_status(ESC Drivers[6]) {
	uint8_t faults = B00000000; // Set a bit to 1 for each faulted motor
	for(int i = 0; i < 6; i++) {
		if(!Drivers[i].get_ok_status()) {
			faults |= (1UL << i);
			sprintf(log_buffer,"Fault on motor i=%d (%d)", i, i+1);
			nh.logerror(log_buffer);
		}
	}
	motor_faults_msg.data = faults;
	motor_faults_pub.publish(&motor_faults_msg);
	return faults;
}

//   ___  ___  ___    ___      _ _ _             _       
//  | _ \/ _ \/ __|  / __|__ _| | | |__  __ _ __| |__ ___
//  |   / (_) \__ \ | (__/ _` | | | '_ \/ _` / _| / /(_-<
//  |_|_\\___/|___/  \___\__,_|_|_|_.__/\__,_\__|_\_\/__/
                                                      
void teleop_cb(geometry_msgs::Twist& teleop_msg) {
	teleop_twist = teleop_msg;
}
