#include <Arduino.h>
#include <ros.h>
#include <Wire.h>
#include <rsx_esc.h>
#include <main.h>

// Allocate 128 bytes to hold ros loggin messages
char log_buffer[128];

void setup() {
	Wire.begin();
}

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

	// Only run this code once
	static bool first_run = true;
	if(first_run) {
		reset_driver_faults(Drivers);
		first_run = false;
	}
}

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

void check_motor_status(ESC Drivers[6]) {
	for(int i = 0; i < 6; i++) {
		if(!Drivers[i].get_ok_status()) {
			sprintf(log_buffer,"Fault on motor i=%d (%d)", i, i+1);
			nh.logerror(log_buffer);
		}
	}
}
