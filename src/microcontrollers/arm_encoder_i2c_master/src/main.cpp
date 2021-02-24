#include <Arduino.h>
#include <Wire.h>

// The Addresses are relative, so A->B->C, then A being the master, then B must be 2 and C must be 3 and so on...
#define total_no_of_slaves 4
#define total_no_of_sensors 7
int Sensor_values[total_no_of_sensors] = { 0 };
byte Sensor_value_bytes[2] = {7};
// Specific to I2C System Design - function has the limit below in the for loop
int Sensors_per_slave[total_no_of_slaves] = {0,2,2,3};

void setup() {
    Wire.begin(); // put your setup code here, to run once:
    Serial.begin(115200);
}

void loop() {

    int sensor_no = 0;
    for (int i = 0; i < total_no_of_slaves; i++) {
      if(i == 0){
        Wire.beginTransmission(i);
        
      }
        for (int sensor = 0; sensor < Sensors_per_slave[i]; sensor++) {
            
            if (sensor > 0) {
                delay(10); // delay so same slave is not overloaded
            }

            Wire.beginTransmission(i); // transmit to device i
            Wire.write(sensor);        // sends which Sensor per slave is desired
            Wire.endTransmission();    // stop transmitting
            Wire.requestFrom(i, 2);    // request info from slave device i

            while (Wire.available()) { // slave may send less than requested
                byte a = Wire.read();
                byte b = Wire.read();

                

                Sensor_values[sensor_no] = a;
                Sensor_values[sensor_no] = (Sensor_values[sensor_no] << 8) | b;
                Wire.beginTransmission(0);
                Wire.write(sensor_no);
                Wire.write(a);
                Wire.write(b);
                Wire.endTransmission();
            }
           
            sensor_no++;
        }
      //  Serial.println(Sensor_values[0]);
    }
    delay(10); // Might not be needed because different slave
}