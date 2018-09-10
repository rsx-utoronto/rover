#include <Arduino.h>
#include <science_2018.h>
#include <stdint.h>
#include <OneWire.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_MLX90614.h>
#include <Servo.h>

// Comment in the appropriate line for the BME interface here
Adafruit_BME680 bme; // I2C

// DO NOT USE THIS LINE: BME WILL LIVE ON I2C!
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// DS18S20 Temperature chip i/o

char LCD = 6; // address of LCD Display
char c = 'A';

Servo servo[5]; //5 servos
// Servos 0:3 control the doors
// Servo 4 controls the swing arm

  int door_closed_angle[4] = {180, 180, 180, 180};
  int door_open_angle[4] = {0, 0, 0, 0};

void setup() {
  Serial.begin(9600);
  setup_BME();
  mlx.begin();
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(ADC_SELECTN, OUTPUT);
  pinMode(SERVO0_PIN , OUTPUT);
  pinMode(SERVO1_PIN , OUTPUT);
  pinMode(SERVO2_PIN , OUTPUT);
  pinMode(SERVO3_PIN , OUTPUT);
  pinMode(SERVO4_PIN , OUTPUT);
  servo[0].attach(SERVO0_PIN);
  servo[1].attach(SERVO1_PIN);
  servo[2].attach(SERVO2_PIN);
  servo[3].attach(SERVO3_PIN);
  servo[4].attach(SERVO4_PIN);

  digitalWrite(ADC_SELECTN, HIGH); //active low, so should on by default
  //  Wire.begin();

  analogReference(EXTERNAL);
  servo[0].write(door_closed_angle[0]);
  servo[1].write(door_closed_angle[1]);
  servo[2].write(door_closed_angle[2]);
  servo[3].write(door_closed_angle[3]);
}

void loop() {
  float air_temp_maxim;
  float air_temp_lm35;
  float soil_moisture_0;
  float air_temp_ch0;
  float as[8];

  // Parse commands from the serial buffer
  if (Serial.available()) {
    switch (Serial.read())
    {
      case 'c': // Close all doors
        servo[0].write(door_closed_angle[0]);
        servo[1].write(door_closed_angle[1]);
        servo[2].write(door_closed_angle[2]);
        servo[3].write(door_closed_angle[3]);
        break;
      case '0':
        servo[0].write(door_open_angle[0]);
        servo[1].write(door_closed_angle[1]);
        servo[2].write(door_closed_angle[2]);
        servo[3].write(door_closed_angle[3]);
        break;
      case '1':
        servo[0].write(door_closed_angle[0]);
        servo[1].write(door_open_angle[1]);
        servo[2].write(door_closed_angle[2]);
        servo[3].write(door_closed_angle[3]);
        break;
      case '2':
        servo[0].write(door_closed_angle[0]);
        servo[1].write(door_closed_angle[1]);
        servo[2].write(door_open_angle[2]);
        servo[3].write(door_closed_angle[3]);
        break;
      case '3':
        servo[0].write(door_closed_angle[0]);
        servo[1].write(door_closed_angle[1]);
        servo[2].write(door_closed_angle[2]);
        servo[3].write(door_open_angle[3]);
        break;
      case 'o':
        servo[0].write(door_open_angle[0]);
        servo[1].write(door_open_angle[1]);
        servo[2].write(door_open_angle[2]);
        servo[3].write(door_open_angle[3]);
      case 't':
        servo[4].write(22);
        break;
      case 'r':
        servo[4].write(150);
        break;
      default:
        Serial.println("Command not recognized");
    }
  }

  for (int i = 0; i < 8; i++) {
    as[i] = adc_read(i);
  }

  // air_temp_maxim = get_DS18S20_air_temp(DS18S20_PIN);
  // air_temp_lm35 = get_LM35_air_temp(LM35_PIN);
  // soil_moisture_0 = get_soil_moisture(MOISTURE0_PIN);
  // soil_moisture_0 = Serial.read();
  // air_temp_maxim = Serial.read();
  //Serial.print("\t Moisture: ");    Serial.print(soil_moisture_0);
  //Serial.print("\t Maxim: ");       Serial.print(air_temp_maxim);
  //Serial.print("\t LM35: ");        Serial.print(air_temp_lm35);
  Serial.print("Timestamp: ");              Serial.print(millis());
  Serial.print("\t MLX ambient: ");         Serial.print(mlx.readAmbientTempC());
  Serial.print("\t MLX Object: ");          Serial.print(mlx.readObjectTempC());
  Serial.print("\t BME680 Temp: ");         Serial.print(bme.temperature);
  Serial.print("\t Pressure: ");            Serial.print(bme.pressure);
  Serial.print("\t Humidity: ");            Serial.print(bme.humidity);
  Serial.print("\t VOCs (kOhms): ");        Serial.print(bme.gas_resistance / 1000.0);
  Serial.print("\t Altitude wrt SL (m): "); Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println();
  
  for (int i = 0; i < 8; i++) {
    Serial.print("\t ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(as[i],5);
  }
  Serial.println();
  
  /*
  air_temp_ch0 = (5.0 * as[0] * 100.0);
  Serial.print("\t CH0TMP: ");
  air_temp_ch0 = Serial.read();
  Serial.print(air_temp_ch0);
  */

  // put your main code here, to run repeatedly:

  /*
  I2C_TX(LCD, c);
  c++;
  if (c > 'Z') {
    c = 'A';
  }
  */
  delay(250);
}

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

void setup_BME(void) {
  Serial.println(F("BME680 test"));
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

// float get_LM35_air_temp(char analog_pin) {
//   float air_temp;
//   pinMode(analog_pin, INPUT);
//   delay(100); // wait a bit to clear jitter?
//   air_temp = analogRead(analog_pin);
//   air_temp = Serial.read();
//   Serial.println(" ");
//   Serial.println(air_temp);
//   air_temp = (5.0 * air_temp * 100.0) / 1024;
//   return air_temp;
// }

float get_soil_moisture(char analog_pin) {
  float moisture;
  pinMode(analog_pin, INPUT);
  delay(100);
  moisture = analogRead(analog_pin);
  //moisture = moisture*5.0/1024.0;
  return moisture;
}

float get_DS18S20_air_temp(char digital_pin) {
  char i;
  char present = 0;
  char data[12];
  char addr[8];
  char data_crc = 0;
  OneWire ds(digital_pin);
  int16_t DS18S20_tempdata = 0;
  float DS18S20_airtemp = 0;
  ds.reset_search();
  if ( !ds.search(addr)) {
    Serial.print("No more addresses.\n");
    Serial.println("Could not find any 1-wire devices");
    ds.reset_search();
    while (1);
    return;
  }

  //Serial.print("R=");
  for ( i = 0; i < 8; i++) {
    //Serial.print(addr[i], HEX);
    //Serial.print(" ");
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.print("CRC is not valid!\n");
    return;
  }

  if ( addr[0] == 0x10) {
    //Serial.print("Device is a DS18S20 family device.\n");
  }
  else if ( addr[0] == 0x28) {
    //Serial.print("Device is a DS18B20 family device.\n");
  }
  else {
    //Serial.print("Device family is not recognized: 0x");
    Serial.println(addr[0], HEX);
    return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("P=");
  //Serial.print(present,HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 chars
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  data_crc = OneWire::crc8( data, 8);
  //Serial.print(" CRC=");
  //Serial.print( data_crc, HEX);

  if (data[8] != data_crc) {
    Serial.println("Warning: DS18S20 Data CRC Mismatch");
    //while(1);
  }

  DS18S20_tempdata = 0;
  DS18S20_tempdata = data[1];
  DS18S20_tempdata <<= 8;
  DS18S20_tempdata += data[0];
  DS18S20_airtemp = DS18S20_tempdata;
  DS18S20_airtemp = DS18S20_airtemp * DS18S20_CONVERSION_CONST;
  //Serial.print(" Temp: ");
  //Serial.print(DS18S20_airtemp);
  //Serial.println();

  return DS18S20_airtemp;
}

void I2C_TX(char device, char data) {
  Wire.beginTransmission(device);
  Wire.write(data);
  Wire.endTransmission();
}


