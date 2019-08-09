#include <Arduino.h>
#include <Wire.h>

// The Addresses are relative, so A->B->C, then A being the master, then B must be 2 and C must be 3 and so on...
#define slave_address 0
#define total_no_of_sensors 2

// Set Apin as -1 if not used
// #define outputA -1  <- A1 & B1 are disabled

#define outputA 4
#define outputB 5
#define outputA2 9
#define outputB2 8
#define outputA3 -1
#define outputB3 -1

int outputA_pins[total_no_of_sensors] = { 0 };
int outputB_pins[total_no_of_sensors] = { 0 };
int Sensor_values[total_no_of_sensors] = { 0 };
int aStates[total_no_of_sensors] = { 0 };
int aLastStates[total_no_of_sensors] = { 0 };
byte Sensor_value_bytes[2] = { 0 };

int currentSensor = 0; // Returns Sensor_value of <currentSensor>
/*
A - 0
A2 - 1
A3 - 2
 */
void receiveEvent(int Sensor_no);
void requestEvent();

void setup() {
    Wire.begin(slave_address); // put your setup code here, to run once:
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent); //put in code to make sure the slave knows how to handle a request from the master
    Serial.begin(9600);

    // Setting all pin values
    int i = 0;
    if (outputA != -1) {
        outputA_pins[i] = outputA;
        outputB_pins[i] = outputB;
        i++;
    }
    if (outputA2 != -1) {
        outputA_pins[i] = outputA2;
        outputB_pins[i] = outputB2;
        i++;
    }
    if (outputA3 != -1) {
        outputA_pins[i] = outputA3;
        outputB_pins[i] = outputB3;
        i++;
    }

    // Setting all pinModes and Sensor states
    for (i = 0; i < total_no_of_sensors; i++) {
        pinMode(outputA_pins[i], INPUT);
        pinMode(outputB_pins[i], INPUT);
        aLastStates[i] = digitalRead(outputA_pins[i]);
    }
}

void loop() {
    Serial.println(5);
    /* 
    // Go through n number of sensors and update them all
    for (int i = 0; i < total_no_of_sensors; i++) {
        aStates[i] = digitalRead(outputA_pins[i]);  // Reads the "current" state of the outputA[i]
        
        // If the previous and the current state of the outputA[i] are different, that means a Pulse has occured
        if (aStates[i] != aLastStates[i]) {

            // If the outputB[i] st_ate is different to the outputA[i] state, that means the encoder is rotating clockwise
            if (digitalRead(outputB_pins[i]) != aStates[i]) {

                Sensor_values[i] = Sensor_values[i] + 1;
            } else {
                Sensor_values[i] = Sensor_values[i] - 1;
            }
        }
        aLastStates[i] = aStates[i]; // Updates the previous state of the outputA[i] with the current state
        Serial.println(outputA_pins[i]);
        Serial.println(Sensor_values[i]);
        Serial.println("\n");
    }
    delay(10); // might not be needed maybe*/
}

void receiveEvent(int num_bytes) {

    while (Wire.available()) { // slave may send less than requested

        // Set currentSensor so the desired sensor is reported
        currentSensor = Wire.read();
    }
}

void requestEvent() {

    // Bit Manipulation so an entire integer can be sent
    /*
    for (int i = 0; i < total_no_of_sensors; i++) {
        Sensor_value_bytes[2*i] = (Sensor_values[i]>>8)&(0xFF);
        Sensor_value_bytes[2*i + 1] = (Sensor_values[i])&(0xFF);
    }
    Wire.write(Sensor_value_bytes, total_no_of_sensors*2);
    */
    Sensor_value_bytes[0] = (Sensor_values[currentSensor] >> 8) & (0xFF);
    Sensor_value_bytes[1] = (Sensor_values[currentSensor]) & (0xFF);
    Wire.write(Sensor_value_bytes, 2);

    // May need to adjust the number of bytes
}