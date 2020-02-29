#include <Arduino.h>
#include <Stream.h>
#include <PID_v1.h>
#include <WSWire.h>

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

#include <main.h>

//   ___ _     _          _    
//  / __| |___| |__  __ _| |___
// | (_ | / _ \ '_ \/ _` | (_-<
//  \___|_\___/_.__/\__,_|_/__/

const char dirPin[7] = {22, 23, 4, 6, 8, 10, 12};
const char pwmPin[7] = {2, 3, 5, 7, 9, 11, 13};

const int spdLimit[7] = {255, 255, 255, 255, 255, 255, 255};

// IDs for each of the joints and motors
typedef enum joint {  SHR,   SHP,   ELB,   FAR,   WPI,   WRO,   GRP};
typedef enum motor {M_SHR, M_SHP, M_ELB, M_FAR, M_WRL, M_WRR, M_GRP};

// The PWM voltage to apply to each MOTOR
double vel[7] = {0, 0, 0, 0, 0, 0, 0};

bool running = true;

//   ___  ___  ___ 
//  | _ \/ _ \/ __|
//  |   / (_) \__ \
//  |_|_\\___/|___/

ros::NodeHandle nh;

// Set up subscribers
int_fast16_t arm_direct_pwm_vels[7];
ros::Subscriber<std_msgs::Int16MultiArray> arm_direct_pwm_sub("rover/arm_direct_pwm", &arm_direct_pwm_cb);

//   ___      _             
//  / __| ___| |_ _  _ _ __ 
//  \__ \/ -_)  _| || | '_ \
//  |___/\___|\__|\_,_| .__/
//                    |_|   

void setup() {
    drivers_initilize();
    //setup_PID();
    nh.getHardware()->setBaud(115200);
    nh.initNode();
	nh.subscribe(arm_direct_pwm_sub);
    nh.loginfo("arm drivers and encoders initialized.");
}

//   __  __      _        _                  
//  |  \/  |__ _(_)_ _   | |   ___  ___ _ __ 
//  | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ 
//  |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
//                                     |_|   

void loop() {
    nh.spinOnce();
    arm_direct_pwm_update();
    update_velocity();
}

//   ___             _   _             
//  | __|  _ _ _  __| |_(_)___ _ _  ___
//  | _| || | ' \/ _|  _| / _ \ ' \(_-<
//  |_| \_,_|_||_\__|\__|_\___/_||_/__/

void arm_direct_pwm_update() {
    vel[0] = arm_direct_pwm_vels[0];
    vel[1] = -arm_direct_pwm_vels[1];
    vel[2] = arm_direct_pwm_vels[2];
    vel[3] = -arm_direct_pwm_vels[3];
    // Translate IK spherical model to differential wrist
    vel[4] = -arm_direct_pwm_vels[4];// - arm_direct_pwm_vels[5];  // tilt + rot
    vel[5] = -arm_direct_pwm_vels[5];// + arm_direct_pwm_vels[4]; // tilt + rot
    //Serial.println(vel[5]);
    // Take into account spherical wrist rotation for the gripper output
    vel[6] = arm_direct_pwm_vels[6];
}

void update_velocity() {
    for (int i = 0; i < 7; i++) {
        if (running) {
            int dir;
            if(vel[i] > 0) {
                dir = HIGH;
            } else {
                dir = LOW;
            }
            digitalWrite(dirPin[i], dir);
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

//   ___  ___  ___    ___      _ _ _             _       
//  | _ \/ _ \/ __|  / __|__ _| | | |__  __ _ __| |__ ___
//  |   / (_) \__ \ | (__/ _` | | | '_ \/ _` / _| / /(_-<
//  |_|_\\___/|___/  \___\__,_|_|_|_.__/\__,_\__|_\_\/__/

void arm_direct_pwm_cb(const std_msgs::Int16MultiArray& msg) {
    for (int i = 0; i < 7; i++) {
        if(msg.data[i] > 255) {
            arm_direct_pwm_vels[i] = 255;
        } else if (msg.data[i] < -255) {
            arm_direct_pwm_vels[i] = -255;
        } else {
            arm_direct_pwm_vels[i] = msg.data[i];
        }
    }
}
