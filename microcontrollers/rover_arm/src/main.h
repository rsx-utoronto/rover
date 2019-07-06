#ifndef main_h
#define main_h

void updatePID();
void drivers_initilize();
void update_velocity();
void update_encoders();
void update_goals(bool no_limits, bool absolute);
void setup_interrupts();
void setup_PID();
void direct_velocity_control();
void starting_position();

void A0_handler();
void B0_handler();
void A1_handler();
void B1_handler();
void A2_handler();
void B2_handler();
void A3_handler();
void B3_handler();
void A4_handler();
void B4_handler();
void A5_handler();
void B5_handler();
void A6_handler();
void B6_handler();

void TEST_find_encoder_pins();
void TEST_print_encoder_pins();
void PRINT_encoder_positions();
void TEST_PID();
void TEST_motor_pins();
void PRINT_oscilloscope(int motor);
#endif
