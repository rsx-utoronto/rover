#include <Arduino.h>
#include <Stream.h>
#include <PID_v1.h>
#include <main.h>
#include <Wire.h>

#define Controller_address 0

// IDs for each of the joints and motors
typedef enum joint {  SHR,   SHP,   ELB,   FAR,   WPI,   WRO,   GRP};
typedef enum motor {M_SHR, M_SHP, M_ELB, M_FAR, M_WRL, M_WRR, M_GRP};

double goal_pos[7] = {0, 0, 0, 0, 0, 0, 0};         // The goal position for each MOTOR
volatile int actual_pos[7] = {0, 0, 0, 0, 0, 0, 0}; // The reading of each MOTOR encoder
double actual_pos_float[7] = {0, 0, 0, 0, 0, 0, 0}; // Floating point copy of above for PID lib
double vel[7] = {0, 0, 0, 0, 0, 0, 0};              // The PWM voltage to apply to each MOTOR

// Angle limits for each JOINT
double high_pos_limit[7] = {3000,  1956,  1263, 840, 400, 1e10, 21000};
double low_pos_limit[7] = {-1400, -2158, -1180, -840, -550, -1e10, 0};

// PID parameters for each MOTOR
double Kp[7] = {0.7,     8,   10,  1.0, 1.32, 1.32,   1};
double Ki[7] = {1,     0.8,    0,  1.0,    1,    1,   0};
double Kd[7] = {0.06, 0.05, 0.05, 0.05, 0.04, 0.04,   0};

// Pins for each motor
const char dirPin[7] = {12, 10, 13, 9, 11, 15, 14};
const char pwmPin[7] = {7, 5, 8, 4, 6, 2, 3};

const double spdLimit[7] = {255, 255, 255, 255, 255, 255, 255};

// Flags
bool running = true;          // Used for emergency stopping
bool manual_override = false; // Used for the 'm' command

// Pins for A and B output of each encoder
const char enc_A[7] = {38, 24, 21, 32, 46, 52, 34};
const char enc_B[7] = {44, 26, 20, 48, 50, 42, 30};

// PID objects operate on the values of vel[] directly
// Change to REVERSE if PID control is backwards
PID PID_0(&actual_pos_float[0], &vel[0], &goal_pos[0], Kp[0], Ki[0], Kd[0], DIRECT);
PID PID_1(&actual_pos_float[1], &vel[1], &goal_pos[1], Kp[1], Ki[1], Kd[1], DIRECT);
PID PID_2(&actual_pos_float[2], &vel[2], &goal_pos[2], Kp[2], Ki[2], Kd[2], DIRECT);
PID PID_3(&actual_pos_float[3], &vel[3], &goal_pos[3], Kp[3], Ki[3], Kd[3], REVERSE);
PID PID_4(&actual_pos_float[4], &vel[4], &goal_pos[4], Kp[4], Ki[4], Kd[4], DIRECT);
PID PID_5(&actual_pos_float[5], &vel[5], &goal_pos[5], Kp[5], Ki[5], Kd[5], REVERSE);
PID PID_6(&actual_pos_float[6], &vel[6], &goal_pos[6], Kp[6], Ki[6], Kd[6], REVERSE);

void setup() {
    Wire.begin(Controller_address); // put your setup code here, to run once:
    Wire.onReceive(receiveEvent);
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.setTimeout(2); // Set the timeout to 2ms, otherwise parsing can hang for up to a second
    Serial.println("Serial initialized.");
    drivers_initilize();
    starting_position();
    setup_PID();
    Serial.println("drivers and encoders initialized.");
}

unsigned long last_print = millis();
unsigned long last_override = 0;

void loop() {
    if (Serial.available()) {
        switch (Serial.read()) {
            case 'p': // Move to absolute position within limits
                Serial.read(); // there should be a space, discard it.
                update_goals(false, true);
                break;
            case 'f': // Move to position with NO LIMITS
                Serial.read();
                update_goals(true, true);
                break;
            case 'r': // Relative move
                Serial.read();
                update_goals(true, false);
                break;
            case 'e': // Emergency stop
                running = 0;
                break;
            case 'c': // Resume from emergency stop
                running = 1;
                updatePID(); // update PID twice so D term does not explode
                break;
            case 'z': // Zero out encoders
                for (int i = 0; i < 7; i++) {
                    actual_pos[i] = 0;
                    goal_pos[i] = 0;
                }
                updatePID();
                break;
            case 'm':                      // direct manual control of joint velocities
                last_override = millis();
                manual_override = true;
                direct_velocity_control(); // paser
                break;
            case 's': // starting position (fully upright and center)
                starting_position(); // calibrate encoders
                updatePID();         // update PID twice so D term does not explode
                break;
            case 'a':                // print encoder positions
                PRINT_encoder_positions();
                break;
            default:
                Serial.println("parse err");
        }
    }
    if (!manual_override) {
        updatePID();
    } else {
        // we are in manual mode, so we already set the velocities we want.
        if (millis() - last_override > 100) {
            // if we have not recieved an update for a while, set all velocities to zero
            // to prevent damage
            for (int i = 0; i < 7; i++) {
                    vel[i] = 0;
            }
        }
    }
    update_velocity();
}

void receiveEvent(int number_of_bytes) {
    while (Wire.available()) {
        byte sensor_no = Wire.read(); // sensor_no
        byte a = Wire.read();
        byte b = Wire.read();

        actual_pos[sensor_no] = a;
        actual_pos[sensor_no] = (actual_pos[sensor_no] << 8) | b;
    }
}

void updatePID() {
    actual_pos_float[0] = (double) actual_pos[0];
    PID_0.Compute();
    actual_pos_float[1] = (double) actual_pos[1];
    PID_1.Compute();
    actual_pos_float[2] = (double) actual_pos[2];
    PID_2.Compute();
    actual_pos_float[3] = (double) actual_pos[3];
    PID_3.Compute();
    actual_pos_float[4] = (double) actual_pos[4];
    PID_4.Compute();
    actual_pos_float[5] = (double) actual_pos[5];
    PID_5.Compute();
    actual_pos_float[6] = (double) actual_pos[6];
    PID_6.Compute();
}

void update_goals(bool no_limits = false, bool absolute = true) {
    // disable manual override
    manual_override = false;

    int raw_pos[7];
    for (int i = 0; i < 7; i++) {
        // Parse incoming integer. raw_pos holds the angles of the IK model.
        // These angles will be translated to output joints so that the
        // differential spherical wrist and gripper can operate
        raw_pos[i] = Serial.parseInt();
        // apply constraints at raw joint angle level (if applicable)
        if (!no_limits) {
            // TODO: this is not working correctly???
            raw_pos[i] = constrain(raw_pos[i], low_pos_limit[i], high_pos_limit[i]);
        }
    }
    if (absolute) {
        goal_pos[0] = raw_pos[0];
        goal_pos[1] = -raw_pos[1];
        goal_pos[2] = raw_pos[2];
        goal_pos[3] = -raw_pos[3];
        // Translate IK spherical model to differential wrist
        goal_pos[4] = -raw_pos[4] - raw_pos[5];  // tilt + rot
        goal_pos[5] = -raw_pos[4] + raw_pos[5]; // tilt + rot
        // Take into account spherical wrist rotation for the gripper output
        goal_pos[6] = raw_pos[6] + ((double) raw_pos[5] * 1680.0/(26.9*64.0));
    } else {
        // RELATIVE mode, just add the values.
        goal_pos[0] += raw_pos[0];
        goal_pos[1] += -raw_pos[1];
        goal_pos[2] += raw_pos[2];
        goal_pos[3] += -raw_pos[3];
        // Translate IK spherical model to differential wrist
        goal_pos[4] += -raw_pos[4] - raw_pos[5];  // tilt + rot
        goal_pos[5] += -raw_pos[4] + raw_pos[5]; // tilt + rot
        // Take into account spherical wrist rotation for the gripper output
        goal_pos[6] += raw_pos[6] + ((double) raw_pos[5] * 1680.0/(26.9*64.0));
    }
}

void direct_velocity_control(){
    int raw_vel[7];
    for (int i = 0; i < 7; i++) {
        raw_vel[i] = Serial.parseInt();
    }
    vel[0] = raw_vel[0];
    vel[1] = -raw_vel[1];
    vel[2] = raw_vel[2];
    vel[3] = -raw_vel[3];
    // Translate IK spherical model to differential wrist
    vel[4] = -raw_vel[4] - raw_vel[5];  // tilt + rot
    vel[5] = -raw_vel[4] + raw_vel[5]; // tilt + rot
    // Take into account spherical wrist rotation for the gripper output
    vel[6] = raw_vel[6] + ((double) raw_vel[5] * 1680.0/(26.9*64.0));
}

void update_velocity() {
    for (int i = 0; i < 7; i++) {
        if (running) {
            digitalWrite(dirPin[i], vel[i] > 0);
            analogWrite(pwmPin[i], abs(vel[i]));
        } else {
            // e-stop activated, stop running
            analogWrite(pwmPin[i], 0);
        }
    }
}

void drivers_initilize() {
    for (int i = 0; i < 7; i++) {
        pinMode(dirPin[i], OUTPUT);
        pinMode(pwmPin[i], OUTPUT);
    }
}

void setup_PID(){
    PID_0.SetMode(AUTOMATIC);
    PID_0.SetOutputLimits(-spdLimit[0], spdLimit[0]);
    PID_1.SetMode(AUTOMATIC);
    PID_1.SetOutputLimits(-spdLimit[1], spdLimit[1]);
    PID_2.SetMode(AUTOMATIC);
    PID_2.SetOutputLimits(-spdLimit[2], spdLimit[2]);
    PID_3.SetMode(AUTOMATIC);
    PID_3.SetOutputLimits(-spdLimit[3], spdLimit[3]);
    PID_4.SetMode(AUTOMATIC);
    PID_4.SetOutputLimits(-spdLimit[4], spdLimit[4]);
    PID_5.SetMode(AUTOMATIC);
    PID_5.SetOutputLimits(-spdLimit[5], spdLimit[5]);
    PID_6.SetMode(AUTOMATIC);
    PID_6.SetOutputLimits(-spdLimit[6], spdLimit[6]);
}

void starting_position() {
    // shoulder rotation centered
    actual_pos[0] = 0;
    goal_pos[0] = 0;
    // shoulder and elbow fully upright
    actual_pos[1] = -high_pos_limit[1];
    goal_pos[1] = -high_pos_limit[1];
    actual_pos[2] = high_pos_limit[2];
    goal_pos[2] = high_pos_limit[2];
    for (int i = 3; i <= 6; i++){
        actual_pos[i] = 0;
        goal_pos[i] = 0;
    }
}

/*
// Testing functions
void TEST_find_encoder_pins() {
    while(true){
        for(int i = 2; i <= 53; i++){
            pinMode(i, INPUT);
        }
        for(int i = 2; i <= 53; i++){
            Serial.print(i);
            Serial.print(digitalRead(i) ? 'X' : ' ');
            Serial.print(' ');
        }
        Serial.println();
    }
}

void TEST_print_encoder_pins(){
    for(int i = 0; i < 7; i++) {
        pinMode(enc_A[i], INPUT);
        pinMode(enc_B[i], INPUT);
    }
    while(true){
        Serial.print("A: ");
        for(int i = 0; i < 7; i++) {
            Serial.print(digitalRead(enc_A[i]));
        }
        Serial.print(" B: ");
        for(int i = 0; i < 7; i++) {
            Serial.print(digitalRead(enc_B[i]));
        }
        Serial.println();
    }
}
*/

void PRINT_encoder_positions(){
    Serial.print("Goals: ");
    for (int i = 0; i < 7; i++) {
        Serial.print(goal_pos[i]);
        Serial.print(' ');
    }
    Serial.print("Encoders: ");
    for(int i = 0; i < 7; i++){
        Serial.print(actual_pos[i]);
        Serial.print(' ');
    }
    Serial.println();
}

void TEST_PID(){
    unsigned long last_goal_change = millis();
    for(int i = 0; i < 7; i++){
        goal_pos[i] = 1000;
    }
    while(true) {
        // change goal ever second for testing
        if(millis() - last_goal_change > 1000){
            for(int i = 0; i < 7; i++){
                goal_pos[i] = -goal_pos[i];
            }
            last_goal_change = millis();
        }
        Serial.println("goal: ");
        Serial.print(goal_pos[0]);
        Serial.print(" actual_pos: ");
        for(int i = 0; i < 7; i++){
            Serial.print(actual_pos[i]);
            Serial.print(' ');
        }
        Serial.print(" vel: ");
        updatePID();
        for(int i = 0; i < 7; i++){
            Serial.print(vel[i]);
            Serial.print(' ');
        }
        Serial.println();
    }
}

void TEST_motor_pins(){
    while(true){
        for(int i = 0; i < 7; i++) {
            Serial.println(i);
            // forwards
            digitalWrite(dirPin[i], 0);
            analogWrite(pwmPin[i], spdLimit[i]);
            delay(200);
            // stop
            analogWrite(pwmPin[i], 0);
            delay(1000);
            // back
            digitalWrite(dirPin[i], 1);
            analogWrite(pwmPin[i], spdLimit[i]);
            delay(200);
            // stop
            analogWrite(pwmPin[i], 0);
            delay(3000);
        }
    }
}

void PRINT_oscilloscope(int motor){
    if (millis() - last_print > 15) {
        int x = actual_pos[motor];
        for (int i = 0; i < 200; i++){
            if (i == goal_pos[motor]){
                Serial.print('|');
            } else if (i == x) {
                Serial.print('#');
            } else {
                Serial.print(' ');
            }
        }
        last_print = millis();
        Serial.println(last_print);
    }
}