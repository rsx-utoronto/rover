#ifndef main_h
#define main_h

void updatePID();
void drivers_initilize();
void update_velocity();
void update_encoders();
void update_goals(bool no_limits, bool absolute);
void setup_PID();
void direct_velocity_control();
void starting_position();

void TEST_find_encoder_pins();
void TEST_print_encoder_pins();
void PRINT_encoder_positions();
void TEST_PID();
void TEST_motor_pins();
void PRINT_oscilloscope(int motor);
#endif
