#ifndef analog_sensors_h
#define analog_sensors_h

// I got two different chips. Hardware compatible but different software!
//#define ADC_MODEL 3008
#define ADC_MODEL 3304

float sensor_model(int i, int sensorpin);
#endif