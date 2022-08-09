#include <Arduino.h>
#include <Wire.h>

#include <rsx_esc.h>
#include <main.h>

//   ___ _     _          _    
//  / __| |___| |__  __ _| |___
// | (_ | / _ \ '_ \/ _` | (_-<
//  \___|_\___/_.__/\__,_|_/__/

int count = 0;
int count_reached = 500;
bool startCounting = false;

// Set up driving variables
float velocity = 100;
float turndir = 0;
float turnfactor = -1;

//persistnece and state vars
unsigned int persistence_timer = 0;
unsigned short persistence_state = 0;
#define default_persistence 10

#define state_stop 0
#define state_Forward 1
#define state_Left 2
#define state_Right 3
#define state_Back 4

/*   ___      _             
//  / __| ___| |_ _  _ _ __ 
//  \__ \/ -_)  _| || | '_ \
//  |___/\___|\__|\_,_| .__/
//                    |_|   
*/

void setup() {
	Wire.begin();
    Serial.begin (115200);
    while (!Serial);
    Serial.setTimeout(20);//set serial.available timeout
    Serial.println("Started serial");
}

//   __  __      _        _                  
//  |  \/  |__ _(_)_ _   | |   ___  ___ _ __ 
//  | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ 
//  |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
//                                     |_|   

void loop() {
	// Declare these objects here because ROS does not like to have global objects for some reason
	static ESC Drivers[6] = {
		ESC(2340.0, 24, 25, 8, 22, 23, B0001001),
		ESC(2340.0, 26, 27, 9, 28, 29, B0001101),
		ESC(2340.0, 33, 32, 7, 31, 30, B0001000),
		ESC(2340.0, 35, 34, 4, 37, 36, B0001100),
		ESC(2340.0, 39, 38, 5, 41, 40, B0001110),
		ESC(2340.0, 45, 44, 6, 43, 42, B1001100)
	};
	Serial.println("In loop");
    //Im sorry, this is what its come to. Serial handler function for moving rover
    if (Serial.available()){
        switch(Serial.read()){
            case 'w'://forward command
				Serial.println("Going forward");
                persistence_state = state_Forward;
                break;
            case 'a'://Left command
                persistence_state = state_Left;
                break;
            case 'd'://Right command
                persistence_state = state_Left;
                break;
            case 's'://Back command
                persistence_state = state_Back;
                break;
            case 'p'://set speed
                    velocity = Serial.parseFloat();
                break;
            default://stop command: all errors are stop
                persistence_state = state_stop;
                persistence_timer = 0;
				Serial.println("in default");
        }
        persistence_timer = Serial.parseInt() + default_persistence;//default is added, so the motor controller is proc'ed.
    }
    //keep previous state, decrement timer
    if (persistence_timer){
        Serial.println(persistence_timer);
        Serial.println(persistence_state);
        switch (persistence_state){
            case state_Forward:
                set_all_vel (velocity, Drivers);
                break;
            case state_Left:
                turn_left (velocity, turnfactor, Drivers);
                break;
            case state_Right:
                turn_right (velocity, turnfactor, Drivers);
                break; 
            case state_Back:
                set_all_vel (-velocity, Drivers);
                break;
            case state_stop:
                stop (Drivers);
                break;
            default:
                stop (Drivers);
        }
        persistence_timer--;
    }
    else{//stop
        stop (Drivers);
    }
    //print State of drivers
    // for(int i = 0; i < 6; i++) {
    //     Serial.print("Driver ");
    //     Serial.print(i);
    //     Serial.print(" Status: ");
    //     Serial.println(Drivers[i].get_ok_status());
	// }

	// Only run this code once
	static bool first_run = true;
	if(first_run) {
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
		}
	}
	return faults;
}

void stop(ESC Drivers[6]) {
    Serial.println("Stopping all motors");
	set_all_vel(0.0, Drivers);
}