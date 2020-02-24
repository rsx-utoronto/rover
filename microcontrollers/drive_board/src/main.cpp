#include <Arduino.h>
#include <ros.h>
#include <rsx_esc.h>

ESC ESC1(2340.0, 24, 25, 8, 22, 23, B0001001);
ESC ESC2(2340.0, 26, 27, 9, 28, 29, B0001101);
ESC ESC3(2340.0, 33, 32, 7, 31, 30, B0001000);
ESC ESC4(2340.0, 35, 34, 4, 37, 36, B0001100);
ESC ESC5(2340.0, 39, 38, 5, 41, 40, B0001110);
ESC ESC6(2340.0, 45, 44, 6, 43, 42, B1001100);

void setup() {
	
}

void loop() {
	ESC1.set_vel(10.0);
	ESC2.set_vel(10.0);
	ESC3.set_vel(10.0);
	ESC4.set_vel(10.0);
	ESC5.set_vel(10.0);
	ESC6.set_vel(10.0);
}
