#include <Wire.h>


struct esc {
  int ready_pin, cfreq_pin, en_pin, dir_pin, I_lim_pin;
  uint8_t I2C_addr;
};


//Initialize all ESC structs with designated pins
//Find Spreadsheet here: https://docs.google.com/spreadsheets/d/1BMbrglyVKrIZHzoSlqtJ0eHUFT43horeqglTX_FpRUA/edit?usp=sharing
//*****************************************************************************
struct esc ESC1 = {.ready_pin = 22, .cfreq_pin = 23, .en_pin = 24, .dir_pin = 25, .I_lim_pin = 8, .I2C_addr = B0001001};
struct esc ESC2 = {.ready_pin = 28, .cfreq_pin = 29, .en_pin = 26, .dir_pin = 27, .I_lim_pin = 9, .I2C_addr = B0001101};
struct esc ESC3 = {.ready_pin = 31, .cfreq_pin = 30, .en_pin = 33, .dir_pin = 32, .I_lim_pin = 7, .I2C_addr = B0001000};
struct esc ESC4 = {.ready_pin = 37, .cfreq_pin = 36, .en_pin = 35, .dir_pin = 34, .I_lim_pin = 4, .I2C_addr = B0001100};
struct esc ESC5 = {.ready_pin = 41, .cfreq_pin = 40, .en_pin = 39, .dir_pin = 38, .I_lim_pin = 5, .I2C_addr = B0001110};
struct esc ESC6 = {.ready_pin = 43, .cfreq_pin = 42, .en_pin = 45, .dir_pin = 44, .I_lim_pin = 6, .I2C_addr = B1001100};


void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //begin setup for the mega as a master 
  Serial.begin(9600);

//Drive Motor 1
//*****************************************************************************
  esc_setup(ESC1);

  digitalWrite(ESC1.en_pin, HIGH);
  digitalWrite(ESC1.dir_pin, LOW);

  Wire.beginTransmission(ESC1.I2C_addr);
  Wire.write(0x00);
  Wire.write(0xFF);
  Wire.endTransmission();



//Drive Motor 2
//*****************************************************************************
  esc_setup(ESC2);

  digitalWrite(ESC2.en_pin, HIGH);
  digitalWrite(ESC2.dir_pin, LOW);

  Wire.beginTransmission(ESC2.I2C_addr);
  Wire.write(0x00);
  Wire.write(0xFF);
  Wire.endTransmission();
  
//Drive Motor 3
//*****************************************************************************
  esc_setup(ESC3);

  digitalWrite(ESC3.en_pin, HIGH);
  digitalWrite(ESC3.dir_pin, LOW);

  Wire.beginTransmission(ESC3.I2C_addr);
  Wire.write(0x00);
  Wire.write(0xFF);
  Wire.endTransmission();

  
//Drive Motor 4
//*****************************************************************************  
  esc_setup(ESC4);

  digitalWrite(ESC4.en_pin, HIGH);
  digitalWrite(ESC4.dir_pin, LOW);

  Wire.beginTransmission(ESC4.I2C_addr);
  Wire.write(0x00);
  Wire.write(0xFF);
  Wire.endTransmission();
  

//Drive Motor 5
//*****************************************************************************
  esc_setup(ESC5);

  digitalWrite(ESC5.en_pin, HIGH);
  digitalWrite(ESC5.dir_pin, LOW);

  Wire.beginTransmission(ESC5.I2C_addr);
  Wire.write(0x00);
  Wire.write(0xFF);
  Wire.endTransmission();
  

//Drive Motor 6
//*****************************************************************************
  esc_setup(ESC6);

  digitalWrite(ESC6.en_pin, HIGH);
  digitalWrite(ESC6.dir_pin, LOW);

  Wire.beginTransmission(ESC6.I2C_addr);
  Wire.write(0x00);
  Wire.write(0xFF);
  Wire.endTransmission();
  
}

void loop() {
  
}






//Rest of code was written by Hudson, not tested so far


void esc_set_speed(struct esc driver, uint16_t spd) {
  Wire.beginTransmission(driver.I2C_addr);
  Wire.write(spd & 0xFF);
  Wire.write((spd >> 8) & 0xFF);
  Wire.endTransmission();
}

void esc_setup(struct esc driver) {
  pinMode(driver.ready_pin, OUTPUT);
  pinMode(driver.en_pin, OUTPUT);
  pinMode(driver.dir_pin, OUTPUT);
  pinMode(driver.I_lim_pin, INPUT);   //solely for testing purposes, should be an output
  pinMode(driver.cfreq_pin, INPUT);
}
