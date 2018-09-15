#include <Arduino.h>
#include <sensors_3304.h>

float RL[8] = {10, 10, 10, 10, 10, 10, 10, 10};
float RO[8];
int raw_sensor_readings[8];
char sensor_name[8][8] = {"MQ-8", "MQ-4", "MQ-135", "MQ-2", "MQ-3", "Soil-1", "Soil-2", "Soil-3"}
char sensor_type[8][10] = {"H2", };
char sensor_units[8][10] = {"ppm", }

/**************************************/
/*       Sensor Calibration           */
/**************************************/
calibrate_sensors(){
  /***** H2 *****/
  float x = 0.0;
  for(int i = 0; i < 10; i++) {
    val += 
  }
}

/**************************************/
/*       3304/3308 funcitons          */
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

float resistance_read(int pin, RL_val) {
  int raw_val = adc_read_3304(pin);
  return (float) RL_val * (4096 - raw_val)
}