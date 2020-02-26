#include <Arduino.h>
#include <ros.h>
#include <Wire.h>
#include <rsx_esc.h>
#include <main.h>


void setup() {
	Wire.begin();
}

void loop() {
	static ESC ESC1(2340.0, 24, 25, 8, 22, 23, B0001001);
	static ESC ESC2(2340.0, 26, 27, 9, 28, 29, B0001101);
	static ESC ESC3(2340.0, 33, 32, 7, 31, 30, B0001000);
	static ESC ESC4(2340.0, 35, 34, 4, 37, 36, B0001100);
	static ESC ESC5(2340.0, 39, 38, 5, 41, 40, B0001110);
	static ESC ESC6(2340.0, 45, 44, 6, 43, 42, B1001100);
	static bool first_run = true;
	if(first_run) {
		ESC1.set_enable(false);
		ESC2.set_enable(false);
		ESC3.set_enable(false);
		ESC4.set_enable(false);
		ESC5.set_enable(false);
		ESC6.set_enable(false);
		delay(1000);
		ESC1.set_enable(true);
		ESC2.set_enable(true);
		ESC3.set_enable(true);
		ESC4.set_enable(true);
		ESC5.set_enable(true);
		ESC6.set_enable(true);
		delay(1000);
		first_run = false;
	}

	float a = -200.0;
	ESC1.set_vel(a);
	ESC2.set_vel(a);
	ESC3.set_vel(a);
	ESC4.set_vel(a);
	ESC5.set_vel(a);
	ESC6.set_vel(a);
}
