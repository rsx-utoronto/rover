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

int currentSensor = 0;

void receiveEvent(int numBytes);
void requestEvent();

void setup() {
    Wire.begin(slave_address);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.begin(115200);
    pinMode(LED_BUILTIN, INPUT);

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
        // Serial.print(i);
        // Serial.print(" = ");
        // Serial.print(Sensor_values[i]);
        // Serial.print(", ");
    }
    // Serial.println();
    // delay(); // might not be needed maybe
}

void receiveEvent(int numBytes) {
    // nothing
}

void requestEvent() { // Report back the value of the sensor requested
    digitalWrite(LED_BUILTIN, 1);
    byte out_bytes[6];
    for (int i = 0; i < 3; i++) {
        out_bytes[2 * i] = (Sensor_values[i] >> 8) & 0xFF;
        out_bytes[2 * i + 1] = Sensor_values[i] & 0xFF;
    }
    Wire.write(out_bytes, 6);
    digitalWrite(LED_BUILTIN, 0);
}