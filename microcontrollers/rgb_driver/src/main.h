#ifndef main_h
#define main_h

void colorWipe(uint32_t color, int wait);
void theaterChase(uint32_t color, int wait);
void rainbow(int wait);
void theaterChaseRainbow(int wait);
void rgb_mode_callback( const std_msgs::String& control_msg);

#endif