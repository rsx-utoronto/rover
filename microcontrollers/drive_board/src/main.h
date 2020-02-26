#ifndef main_h
#define main_h

// Functions
void reset_driver_faults(ESC[6]);
void set_all_vel(float, ESC[6]);
uint8_t check_motor_status(ESC[6]);

// Callbacks
void teleop_cb(geometry_msgs::Twist&);

#endif
