#include <Arduino.h>
#include <Wire.h>

#define slave_address 1

const int encoder_a_pins[3] = {4, 9, 11};
const int encoder_b_pins[3] = {5, 10, 12};
int Sensor_values[3] = { 0 };
int aStates[3] = { 0 };
int bStates[3] = { 0 };
int bLastStates[3] = { 0 };
int aLastStates[3] = { 0 };
byte Sensor_value_bytes[2] = { 7 };

int currentSensor = 0;

void receiveEvent(int numBytes);
void requestEvent();

void setup() {
    Wire.begin(slave_address);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.begin(115200);

    // Setting all pinModes and Sensor states
    for (int i = 0; i < 3; i++) {
        pinMode(encoder_a_pins[i], INPUT);
        pinMode(encoder_b_pins[i], INPUT);
        aLastStates[i] = digitalRead(encoder_a_pins[i]);
        bLastStates[i] = digitalRead(encoder_b_pins[i]);
    }
}

void loop() {
    // Update each sensor's state
    for (int i = 0; i < 3; i++) {
        aStates[i] = digitalRead(encoder_a_pins[i]);  // Reads the "current" state
        bStates[i] = digitalRead(encoder_b_pins[i]);
        // compare to previous state, increment or decrement
        if (aStates[i] != aLastStates[i]) {
            if (aStates[i]) {
                bStates[i] ? Sensor_values[i]-- : Sensor_values[i]++;
            } else {
                bStates[i] ? Sensor_values[i]++ : Sensor_values[i]--;
            }
        }

        if(bLastStates[i] != bStates[i]) {
            if(bStates[i]) {
                aStates[i] ? Sensor_values[i]++ : Sensor_values[i]--;
            } else {
                aStates[i] ? Sensor_values[i]-- : Sensor_values[i]++;
            }
        }
        bLastStates[i] = bStates[i];
        aLastStates[i] = aStates[i];
    }
    // Serial.println(Sensor_values[0]);
    // delay(); // might not be needed maybe
}

void receiveEvent(int numBytes) {
    // Set currentSensor so the desired sensor is reported
    currentSensor = Wire.read();
    // Serial.println(currentSensor);
}

void requestEvent() {
    // Report back the value of the sensor requested
    Sensor_value_bytes[0] = ((Sensor_values[0]) >> 8) & (0xFF);
    Sensor_value_bytes[1] = (Sensor_values[0]) & (0xFF);
    Wire.write(Sensor_value_bytes, 2);
}