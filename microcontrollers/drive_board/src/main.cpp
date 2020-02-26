#include <Arduino.h>
#include <Wire.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Byte.h>

#include <rsx_esc.h>
#include <main.h>
<<<<<<< HEAD

//   ___ _     _          _    
//  / __| |___| |__  __ _| |___
// | (_ | / _ \ '_ \/ _` | (_-<
//  \___|_\___/_.__/\__,_|_/__/

=======
//:)
>>>>>>> 6ebb068f1a75b8dbf1d25e3143d71a0f72908a31
// Allocate 128 bytes to hold ros loggin messages
char log_buffer[128];

int count = 0;
int count_reached = 500;
bool startCounting = false;

// Set up driving variables
float velocity = 0;
float turndir = 0;
float turnfactor = 0;

//   ___  ___  ___ 
//  | _ \/ _ \/ __|
//  |   / (_) \__ \
//  |_|_\\___/|___/

ros::NodeHandle nh;

// Set up subscribers
geometry_msgs::Twist teleop_twist;
ros::Subscriber<geometry_msgs::Twist> teleop_sub("drive", &teleop_cb);
std_msgs::Byte motor_faults_msg;
ros::Publisher motor_faults_pub("rover/motor_faults", &motor_faults_msg);

//   ___      _             
//  / __| ___| |_ _  _ _ __ 
//  \__ \/ -_)  _| || | '_ \
//  |___/\___|\__|\_,_| .__/
//                    |_|   

void setup() {
	Wire.begin();
	nh.initNode();
	nh.advertise(motor_faults_pub);
	nh.subscribe(teleop_sub);
}

//   __  __      _        _                  
//  |  \/  |__ _(_)_ _   | |   ___  ___ _ __ 
//  | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ 
//  |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
//                                     |_|   

void loop() {
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
	
	nh.spinOnce();
	
	if (startCounting) {
		count++; 
	}
	if (count == count_reached) {
		count = 0;
		stop(Drivers);
		startCounting = false;
	}

	// Only run this code once
	static bool first_run = true;
	if(first_run) {
		nh.loginfo("Arduino booted up");
		reset_driver_faults(Drivers);
		first_run = false;
	}

	if (turndir<0) {
		turn_right(velocity, turnfactor, Drivers);
	}
	else if (turndir>0) {
		turn_left(velocity, turnfactor, Drivers);
	} else {
		set_all_vel(velocity, Drivers);
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
	nh.loginfo("Moving forward");
	for(int i = 0; i < 6; i++) {
		Drivers[i].set_vel(vel);
	}
}

void set_left_vel(float vel, ESC Drivers[6]) {
	for(int i = 0; i < 3; i++) {
		Drivers[i].set_vel(vel);
	}
}

void set_right_vel(float vel, ESC Drivers[6]) {
	for(int i = 3; i < 6; i++) {
		Drivers[i].set_vel(vel);
	}
}

void turn_left(float vel, float turn, ESC Drivers[6]) {
	nh.loginfo("Moving left");
	set_left_vel(vel*turn, Drivers);
	set_right_vel(vel, Drivers);
}

void turn_right(float vel, float turn, ESC Drivers[6]) {
	nh.loginfo("Moving right");
	set_right_vel(vel*turn, Drivers);
	set_left_vel(vel, Drivers);
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

void stop(ESC Drivers[6]) {
    nh.loginfo("STOP");
	set_all_vel(0.0, Drivers);
}

//   ___  ___  ___    ___      _ _ _             _       
//  | _ \/ _ \/ __|  / __|__ _| | | |__  __ _ __| |__ ___
//  |   / (_) \__ \ | (__/ _` | | | '_ \/ _` / _| / /(_-<
//  |_|_\\___/|___/  \___\__,_|_|_|_.__/\__,_\__|_\_\/__/
                                                      
void teleop_cb(const geometry_msgs::Twist& msg) {
	startCounting = true;
	count = 0;
	
	//Serial.begin(9600);

	char resultAngular[8]; // Buffer big enough for 7-character float
	char resultLinear[8];
	velocity = float(msg.linear.x);
	turndir = float(msg.angular.z);
	turnfactor = 1 - abs(msg.angular.z); 

	dtostrf(velocity, 6, 2, resultAngular); // Leave room for too large numbers!
	dtostrf(turnfactor, 6, 2, resultLinear); // Leave room for too large numbers!
	//Serial.print(speed);
	//Serial.print(turnfactor);

	nh.loginfo(resultAngular);
	nh.loginfo(resultLinear);
}
