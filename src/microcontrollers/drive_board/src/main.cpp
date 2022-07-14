#include <Arduino.h>
#include <Wire.h>
#include <Stream.h>
#include <rsx_esc.h>
#include <main.h>

//   ___ _     _          _    
//  / __| |___| |__  __ _| |___
// | (_ | / _ \ '_ \/ _` | (_-<
//  \___|_\___/_.__/\__,_|_/__/

// Allocate 128 bytes to hold ros loggin messages

#define Controller_address 0

char log_buffer[128];

int count = 0;
int count_reached = 500;
bool startCounting = false;

// Set up driving variables
float velocity = 0;
float turndir = 0;
float turnfactor = 0;

static ESC Drivers[6] = {
	ESC(2340.0, 24, 25, 8, 22, 23, B0001001),
	ESC(2340.0, 26, 27, 9, 28, 29, B0001101),
	ESC(2340.0, 33, 32, 7, 31, 30, B0001000),
	ESC(2340.0, 35, 34, 4, 37, 36, B0001100),
	ESC(2340.0, 39, 38, 5, 41, 40, B0001110),
	ESC(2340.0, 45, 44, 6, 43, 42, B1001100)
};

//   ___      _             
//  / __| ___| |_ _  _ _ __ 
//  \__ \/ -_)  _| || | '_ \
//  |___/\___|\__|\_,_| .__/
//                    |_|   

void setup() {
	Wire.begin(Controller_address);
	Serial.begin(115200);
	while (!Serial);
	Serial.setTimeout(2);
	Serial.println("Serial initialized.");
	reset_driver_faults(Drivers);
	Serial.println("Driver faults reset.");
}

//   __  __      _        _                  
//  |  \/  |__ _(_)_ _   | |   ___  ___ _ __ 
//  | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ 
//  |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
//                                     |_|   

void loop() {
	// Declare these objects here because ROS does not like to have
	// global objects for some reason
	Serial.println("In loop");
	if (Serial.available()) {
		switch (Serial.read()) {
			case 'd':
				parse_drive();
				break;
			default: 
				Serial.println("Not a recognized command");
		}
	}
	
	if (startCounting) {
		count++; 
	}
	
	if (count == count_reached) {
		count = 0;
		stop(Drivers);
		startCounting = false;
	}

	if (turndir<0) {
		turn_right(velocity, turnfactor, Drivers);
	}
	else if (turndir>0) {
		turn_left(velocity, turnfactor, Drivers);
	} else {
		set_all_vel(velocity, Drivers);
	}
	delay(500);
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
	Serial.print("Moving forward with vel: ");
	Serial.println(vel);
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
	set_left_vel(vel*turn, Drivers);
	set_right_vel(vel, Drivers);
}

void turn_right(float vel, float turn, ESC Drivers[6]) {
	set_right_vel(vel*turn, Drivers);
	set_left_vel(vel, Drivers);
}

uint8_t check_motor_status(ESC Drivers[6]) {
	uint8_t faults = B00000000; // Set a bit to 1 for each faulted motor
	for(int i = 0; i < 6; i++) {
		if(!Drivers[i].get_ok_status()) {
			faults |= (1UL << i);
			sprintf(log_buffer,"Fault on motor i=%d (%d)", i, i+1);
		}
	}
	// motor_faults_pub.publish(&motor_faults_msg);
	return faults;
}

void stop(ESC Drivers[6]) {
	set_all_vel(0.0, Drivers);
}

void read_angular() {
	float turn;
	turn = Serial.parseFloat();
	turndir = turn;
	turnfactor = 1 - abs(turndir);
}

void read_linear() {
	float lin_vel;
	velocity = Serial.parseFloat();
}

void parse_drive() {

	if (Serial.available()) {
		switch(Serial.read()){
			case 'a':
				Serial.println("Getting angular");
				read_angular();
			case 'l':
				Serial.println("Getting linear");
				read_linear();
			default: 
				Serial.println("Not a recognized drive command");
		}
	}
}
                                                    
