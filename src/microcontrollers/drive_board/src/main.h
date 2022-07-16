#ifndef main_h
#define main_h

//   ___             _   _             
//  | __|  _ _ _  __| |_(_)___ _ _  ___
//  | _| || | ' \/ _|  _| / _ \ ' \(_-<
//  |_| \_,_|_||_\__|\__|_\___/_||_/__/

void reset_driver_faults(ESC[6]);
void set_all_vel(float, ESC[6]);
uint8_t check_motor_status(ESC[6]);
void turn_left(float, float, ESC[6]);
void turn_right(float, float, ESC[6]);
void set_left_vel(float, ESC[6]);
void set_right_vel(float, ESC[6]);
void stop(ESC[6]);

//   ___  ___  ___    ___      _ _ _             _       
//  | _ \/ _ \/ __|  / __|__ _| | | |__  __ _ __| |__ ___
//  |   / (_) \__ \ | (__/ _` | | | '_ \/ _` / _| / /(_-<
//  |_|_\\___/|___/  \___\__,_|_|_|_.__/\__,_\__|_\_\/__/

// void teleop_cb(const geometry_msgs::Twist&);
void turn_cb(const std_msgs::Float32& turn_msg);
void lin_vel_cb(const std_msgs::Float32& lin_vel_msg);

#endif

