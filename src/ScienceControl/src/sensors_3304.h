#ifndef analog_sensors_h
#define analog_sensors_h

// I got two different chips. Hardware compatible but different software!
//#define ADC_MODEL 3008
#define ADC_MODEL 3304
#define ADC_SELECTN A3

float sensor_model(int i, int sensorpin);
void printSensors();
void calibrate_sensors();
float resistance_read(int pin, float RL_val);
float adc_read(char ch);
int adc_read_3008(char ch);
void clk_out(char b);
char clk_in(void);
int adc_read_3304(char ch);
#endif