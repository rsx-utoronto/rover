#include <Arduino.h>
#include <Wire.h>
#include <rsx_esc.h>

// _full_speed_rpm units: RPM
ESC::ESC(float _full_speed_rpm, int _enable_pin, int _direction_pin, int _current_limit_pin, int _ready_status_pin, int _commutation_freq_pin, uint8_t _I2C_addr) {
	full_speed_rpm = _full_speed_rpm;
	enable_pin = _enable_pin;
	direction_pin = _direction_pin;
	current_limit_pin = _current_limit_pin;
	commutation_freq_pin = _commutation_freq_pin;
	I2C_addr = _I2C_addr;
}

void ESC::init() {
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
	Wire.write((spd >> 8) & 0xFF);
	Wire.write(spd & 0xFF);
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
}

float get_vel(){
	// TODO
	return 0;
}

bool ESC::get_ok_status(){
	return digitalRead(ready_status_pin);
}