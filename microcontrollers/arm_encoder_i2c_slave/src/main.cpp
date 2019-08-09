#include <Arduino.h>
#include <Wire.h>

// The Addresses are relative, so A->B->C, then A being the master, then B must be 2 and C must be 3 and so on...
#define slave_address 0
#define total_no_of_sensors 2

// Set Apin as -1 if not used
// #define outputA -1  <- A1 & B1 are disabled

#define outputA 4
#define outputB 5
#define outputA2 11
#define outputB2 12
#define outputA3 -1
#define outputB3 -1

int outputA_pins[total_no_of_sensors] = { 0 };
int outputB_pins[total_no_of_sensors] = { 0 };
int Sensor_values[total_no_of_sensors] = { 0 };
int aStates[total_no_of_sensors] = { 0 };
int bStates[total_no_of_sensors] = { 0 };
int bLastStates[total_no_of_sensors] = { 0 };
int aLastStates[total_no_of_sensors] = { 0 };
byte Sensor_value_bytes[2] = { 7 };

int currentSensor = 0; // Returns Sensor_value of <currentSensor>
/*
A - 0
A2 - 1
A3 - 2
 */
void receiveEvent(int numBytes);
void requestEvent();

void setup() {
    Wire.begin(slave_address); // put your setup code here, to run once:
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.begin(115200);

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
        bLastStates[i] = digitalRead(outputB_pins[i]);
    }
}

void loop() {
    
    // Go through n number of sensors and update them all
    for (int i = 0; i < total_no_of_sensors; i++) {
        aStates[i] = digitalRead(outputA_pins[i]);  // Reads the "current" state of the outputA[i]
        bStates[i] = digitalRead(outputB_pins[i]);
        if(aStates[i] != aLastStates[i]){
          if(aStates[i]){
            bStates[i] ? Sensor_values[i]--:Sensor_values[i]++;
          } else{
            bStates[i] ? Sensor_values[i]++:Sensor_values[i]--;
          }
        }

        if(bLastStates[i] != bStates[i]){
          if(bStates[i]){
            aStates[i] ? Sensor_values[i]++:Sensor_values[i]--;
            
          }else{
            aStates[i] ? Sensor_values[i]--:Sensor_values[i]++;
          }
        }
        bLastStates[i] = bStates[i];
        aLastStates[i] = aStates[i];
    }
    Serial.println(Sensor_values[0]);
    //delay(); // might not be needed maybe
}

void receiveEvent(int numBytes) {

    // Set currentSensor so the desired sensor is reported
    currentSensor = Wire.read();
    Serial.println(currentSensor);
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
    Sensor_value_bytes[0] = ((Sensor_values[0])>>8)&(0xFF);
    Sensor_value_bytes[1] = (Sensor_values[0])&(0xFF);
//    Serial.println(Sensor_values[0]);
//    Serial.println(Sensor_values[1]);
//    Serial.println(Sensor_values[2]);
    Wire.write(Sensor_value_bytes, 2);
  //  Wire.write(Sensor_value_bytes[1]);
    
    // May need to adjust the number of bytes
}