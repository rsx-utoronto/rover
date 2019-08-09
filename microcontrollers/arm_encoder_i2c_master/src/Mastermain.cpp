#include <Arduino.h>
#include <Wire.h>

// The Addresses are relative, so A->B->C, then A being the master, then B must be 2 and C must be 3 and so on...
#define total_no_of_slaves 3
#define Controller_Address 1
#define total_no_of_sensors 7
int Sensor_values[total_no_of_sensors] = { 0 };

// {Sensor Number, Int as Byte Part I, Int as Byte Part II}
// Sent to Controller
byte Sensor_value_bytes[3] = { 0 };

// Specific to I2C System Design - function has the limit below in the for loop
int Sensors_per_slave[3] = {3, 2, 2};

void setup() {
    Wire.begin(); // put your setup code here, to run once:
    Serial.begin(9600);
}

void loop() {

    int sensor_no = 0;
    // It starts at 1, not 0 because Contoller is 1
    for (int i = 1; i < total_no_of_slaves + 1; i++) {
        for (int sensor = 0; sensor < Sensors_per_slave[i]; sensor++) {
            
            if (sensor > 0) {
                delay(10); // delay so same slave is not overloaded
            }

            Wire.beginTransmission(i);     // transmit to device i
            Wire.write(sensor);    // sends which Sensor per slave is desired
            Wire.endTransmission();        // stop transmitting
            Wire.requestFrom(i, 2);        // request info from slave device i

            while (Wire.available()) { // slave may send less than requested
                byte a = Wire.read();
                byte b = Wire.read();

                Sensor_values[sensor_no] = a;
                Sensor_values[sensor_no] = (Sensor_values[sensor_no] << 8) | b;
            }
            sensor_no++;
            
            // Send Updated Value to Controller
            Sensor_value_bytes[0] = sensor_no;
            Sensor_value_bytes[1] = (Sensor_values[sensor_no] >> 8) & (0xFF);
            Sensor_value_bytes[2] = (Sensor_values[sensor_no]) & (0xFF);

            Serial.println(i, sensor, Sensor_values[sensor_no]);

            Wire.beginTransmission(Controller_Address);
            Wire.write(Sensor_value_bytes, 3);
            Wire.endTransmission();
        }
    }
    delay(10); // Might not be needed because different slave
}