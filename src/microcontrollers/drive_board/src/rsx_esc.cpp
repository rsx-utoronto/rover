#include <Arduino.h>
#include <Wire.h>
#include <rsx_esc.h>

//Ranges of Analog Escon Analog Outputs to Arduino
int analog_velocity_min = -4095; // The lowest number of the range output.
int analog_velocity_max = 4095;  // The largest number of the range output.


// _full_speed_rpm units: RPM
ESC::ESC(float _full_speed_rpm, int _enable_pin, int _direction_pin, int _current_limit_pin, int _ready_status_pin, int _commutation_freq_pin, uint8_t _I2C_addr, int _current_feedback_pin, int _speed_feedback_pin) {
	full_speed_rpm = _full_speed_rpm;
	enable_pin = _enable_pin;
	direction_pin = _direction_pin;
	current_limit_pin = _current_limit_pin;
	commutation_freq_pin = _commutation_freq_pin;
	I2C_addr = _I2C_addr;
	current_feedback_pin =  _current_feedback_pin;
	speed_feedback_pin = _speed_feedback_pin;

	ESC::initialize();
	ESC::set_vel(0);
	ESC::set_enable(true);
}

void ESC::set_vel(float vel) {
	int spd = abs(vel * 60.0 / (2.0*pi) / full_speed_rpm * 4096.0);
	if(spd >= 4095)  {
		spd =  4095;
	}
	digitalWrite(direction_pin, (vel > 0.0));
	Wire.beginTransmission(I2C_addr);
	Wire.write(spd & 0xFF);
	Wire.write((spd >> 8) & 0xFF);
	Wire.endTransmission();
}

void ESC::set_enable(bool state){
	digitalWrite(enable_pin, state);
}

// Sets up all the inputs, outputs, and sets speed to zero.
void ESC::initialize() {
	pinMode(enable_pin, OUTPUT);
	pinMode(direction_pin, OUTPUT);
	pinMode(current_limit_pin, OUTPUT);
	pinMode(ready_status_pin, INPUT);
	pinMode(commutation_freq_pin, INPUT);
	//pinMode(current_feedback_pin, INPUT);   Dont need to pinMode analog input/output when using analogRead
	//pinMode(speed_feedback_pin, INPUT);
}

int ESC::get_vel_4095(){
	/*
	Arduino Mega 2560 Rev3 has a 10bit resolution for the 16 analog inputs
	2^10 = 1024
	Range: 0-5V
	0-1023 values
	4V = 818.4 ~= 819
	
	
	-4095 reverse direction max speed
	4095 forward direction max speed 
	*/

	int vel_val = analogRead(speed_feedback_pin); //0-5V return 0-1023 must map 0-4V to -4095-4095
	int  vel_4095 = map(vel_val, 0,819, analog_velocity_min, analog_velocity_max); 
	return vel_4095;


}
int ESC::get_vel_255(int first_vel_val){
	/*
	Converts the 4095 to -255 - 255 values
	could make it call getvel4095
	*/
	int vel_255 = map(first_vel_val, analog_velocity_min, analog_velocity_max, -255, 255); 
	return vel_255;
}

float ESC::get_cur(){
	/*
	TODO
	Cont Current: 8A
	Max Current: 15A??? not entirely sure
	*/
	return 0;
}

bool ESC::get_ok_status(){
	return digitalRead(ready_status_pin);
}
