#include <Arduino.h>
#include <sensors_3304.h>
#include <science_2018.h>

float e = 2.7182818284590452353602874713527;
float RL[8] = {10, 10, 20, 10, 200, 10, 10, 10};
float RO[8];
int raw_sensor_readings[8];
char sensor_name[8][8] = {"MQ-8", "MQ-4", "MQ-135", "MQ-2", "MQ-3", "Soil-1", "Soil-2", "Soil-3"};

/**************************************/
/* Print Everything                   */
/**************************************/
// equations from http://sandboxelectronics.com
// and lin-reg with http://www.xuru.org/rt/ExpR.asp
void printSensors(){
	float RS[8];
	for(int i = 0; i < 8; i++) {
		RS[i] = resistance_read(i, RL[i]);
	}
	// MQ-8
	// H2
	float pcurve[3] = {2.3, 0.93, -1.44};
	Serial.print("*MQ-8* H2: ");
	Serial.println(pow(10, (log(RS[0]/RO[0]) - pcurve[1]) / pcurve[2] + pcurve[0]));
	Serial.print(" ppm ");
	// MQ-4
	// CH4
	RO[1] = resistance_read(1, RL[1]);
	Serial.print("*MQ-4* CH4: ");
	Serial.print(4.2 + 3.235676266 * pow(e, -5.470334184e-5 * RS[1]/RL[1]));
	Serial.print(" ppm ");
	// MQ-135
	// CO2
	RO[2] = resistance_read(1, RL[2]);
	Serial.print("*MQ-135* CO2: ");
	Serial.print(402.5 + 3.235676266 * pow(e, -5.470334184e-5 * RS[2]/RL[2]));
	Serial.print(" ppm ");
	// MQ-2
	// LPG
	float pcurve1[3] = {2.3,0.21,-0.47};
	Serial.print("*MQ-8* LPG: ");
	Serial.println(pow(10, (log(RS[3]/RO[3]) - pcurve1[1]) / pcurve1[2] + pcurve1[0]));
	Serial.print(" ppm ");
	// CO
	float pcurve2[3] = {2.3,0.72,-0.34};
	Serial.print("*MQ-8* CO: ");
	Serial.println(pow(10, (log(RS[3]/RO[3]) - pcurve2[1]) / pcurve2[2] + pcurve2[0]));
	Serial.print(" ppm ");
	// SMOKE
	// float pcurve3[3] = {2.3,0.53,-0.44};
	// Serial.print("SMOKE: ");
	// Serial.println(pow(10, (log(RS[3]/RO[3]) - pcurve3[1]) / pcurve3[2] + pcurve3[0]));
	// Serial.print(" ppm ");
	// MQ-3
	// ALCOHOL
	RO[1] = resistance_read(4, RL[4]);
	Serial.print("*MQ-3* ALCOHOL: ");
	Serial.print(1.03840724 * pow(e, -2.325843528e-1 * RS[4]/RL[4]));
	Serial.print(" mg/L ");
	// Soil-1
	Serial.print("Soil 1: ");
	Serial.print(adc_read(5));
	Serial.print(" \% ");
	// Soil-2
	Serial.print("Soil 2: ");
	Serial.print(adc_read(6));
	Serial.print(" \% ");
	// Soil-1
	Serial.print("Soil 3: ");
	Serial.print(adc_read(7));
	Serial.println(" \%");
}

/**************************************/
/* Sensor Calibration                 */
/**************************************/
void calibrate_sensors(){
	/***** MQ-8    *****/
	float RO_clean_air_factor = 9.21;
	RO[0] = 0.0;
	for(int i = 0; i < 10; i++) {
		RO[0] += resistance_read(0, RL[0]);
		delay(100);
	}
	RO[0] = RO[0] / RL[0];
	RO[0] = RO[0] / RO_clean_air_factor;
	/***** MQ-4    *****/
	RO_clean_air_factor = 1.0;
	RO[1] = 0.0;
	for(int i = 0; i < 10; i++) {
		RO[1] += resistance_read(0, RL[1]);
		delay(100);
	}
	RO[1] = RO[1] / RL[1];
	RO[1] = RO[1] / RO_clean_air_factor;
	/***** MQ-2    *****/
	RO_clean_air_factor = 9.83;
	RO[3] = 0.0;
	for(int i = 0; i < 10; i++) {
		RO[3] += resistance_read(0, RL[3]);
		delay(100);
	}
	RO[3] = RO[3] / RL[3];
	RO[3] = RO[3] / RO_clean_air_factor;
	/***** MQ-3    *****/
	RO_clean_air_factor = 9.83;
	RO[4] = 0.0;
	for(int i = 0; i < 10; i++) {
		RO[4] += resistance_read(0, RL[4]);
		delay(100);
	}
	RO[4] = RO[4] / RL[4];
	RO[4] = RO[4] / RO_clean_air_factor;
	Serial.print("RO = ");
	for(int i = 0; i < 8; i++){
		Serial.print(RO[i]);
		Serial.print(" ");
	}
}

/**************************************/
/* Sensor Functions                   */
/**************************************/
float resistance_read(int pin, float RL_val) {
	// Returns the resistance of the sensor
	float raw_val = adc_read_3304(pin);
	if(raw_val == 0.0) {
		return 1000000000;
	}
	return RL_val * (4096.0 - raw_val) / raw_val;
}

/**************************************/
/* 3304/3308 funcitons                */
/**************************************/
float adc_read(char ch) {
	if (ch > 8) {
		Serial.println("error: invalid adc channel");
		return;
	}
	if (ADC_MODEL == 3008) {
		return (float)adc_read_3008(ch)/1024.0;
	}
	else if (ADC_MODEL == 3304) {
		return (float)adc_read_3304(ch)/4096.0;
	}
	else {
		Serial.println("error: unknown ADC Model Number");
		return;
	}
}

int adc_read_3008(char ch) {
	int a = 0;
	char i;
	digitalWrite(ADC_SELECTN, 0); //init conversation
	clk_out(1);// start bit
	clk_out(1);// CONFIG MODE: SINGLE (O for differential
	clk_out((ch & 4) >> 2);
	clk_out((ch & 2) >> 1);
	clk_out(ch & 1);
	delayMicroseconds(4); // Wait for sample (way longer than necessary)
	clk_out(0); // Don't care
	for (i = 0; i < (10 + 1); i++) {
		a = (a << 1) + clk_in();
	}
	digitalWrite(ADC_SELECTN, 1); // end conversation
	return a;
}

void clk_out(char b) {
	digitalWrite(SCK, 0);
	delayMicroseconds(4); // probably removable
	digitalWrite(MOSI, b);
	digitalWrite(SCK, 1);
	delayMicroseconds(4); // probably removable
	return;
}

char clk_in(void) {
	char a;
	digitalWrite(SCK, 0);
	a = digitalRead(MISO);
	digitalWrite(SCK, 1);
	return a;
}

int adc_read_3304(char ch) {
	int a = 0;
	char i;
	digitalWrite(ADC_SELECTN, 0); //init conversation
	clk_out(1);// start bit
	clk_out(1);// CONFIG MODE: SINGLE (O for differential)
	clk_out((ch & 4) >> 2);
	clk_out((ch & 2) >> 1);
	clk_out(ch & 1);
	delayMicroseconds(4); // Wait for sample (way longer than necessary)
	clk_out(0); // Don't care
	for (i = 0; i < (13 + 1); i++) {
		a = (a << 1) + clk_in();
	}
	digitalWrite(ADC_SELECTN, 1); // end conversation
	return a;
}
