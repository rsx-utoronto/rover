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
void read_angular();
void read_linear();
void parse_drive();

#endif
