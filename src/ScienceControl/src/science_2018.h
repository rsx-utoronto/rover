#ifndef science_2018_h
#define science_2818_h

// These aren't being used
#define SCK 3
#define MISO 2
#define MOSI A2
#define SEALEVELPRESSURE_HPA (1013.25)
#define DS18S20_PIN 4 // pin for maxim digital air temp sensor
#define LM35_PIN A3 // pin for lm35 analog air temp sensor
#define MOISTURE0_PIN A0 // pin for soil moisture sensor
#define SERVO0_PIN 5 // pin for Servo 0
#define SERVO1_PIN 6
#define SERVO2_PIN 7
#define SERVO3_PIN 8
#define SERVO4_PIN 9
#define DS18S20_CONVERSION_CONST 0.0625 //85 degrees celsius per 0x0550

float adc_read(char ch);
int adc_read_3008(char ch);
void clk_out(char b);
char clk_in(void);
int adc_read_3304(char ch);
void setup_BME();
float get_soil_moisture(char analog_pin);
float get_DS18S20_air_temp(char digital_pin);
void I2C_TX(char device, char data);

#endif
