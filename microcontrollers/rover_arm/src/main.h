#ifndef main_h
#define main_h

void drivers_initilize();
void update_velocity();
void update_encoders();
void update_goals(bool no_limits, bool absolute);
void setup_PID();
void direct_velocity_control();
void starting_position();
void receiveEvent(int number_of_bytes);
void get_encoder_values();

void arm_direct_pwm_update();

//   ___  ___  ___    ___      _ _ _             _       
//  | _ \/ _ \/ __|  / __|__ _| | | |__  __ _ __| |__ ___
//  |   / (_) \__ \ | (__/ _` | | | '_ \/ _` / _| / /(_-<
//  |_|_\\___/|___/  \___\__,_|_|_|_.__/\__,_\__|_\_\/__/

void arm_direct_pwm_cb(const std_msgs::Int16MultiArray&);


#endif
