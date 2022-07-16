#ifndef rsx_esc_h
#define rsx_esc_h

class ESC {
	public:
		ESC(float, int, int, int, int, int, uint8_t);     // constructor
		void set_vel(float);                              //
		void set_current_limit(float);
		void set_vel_limit(float);
		void set_enable(bool);
		bool get_ok_status();
		float get_vel();
	
	private:
		void initialize();
		float pi = 3.14159265;
		float full_speed_rpm;
		int ready_status_pin;
		int commutation_freq_pin;
		int enable_pin;
		int direction_pin;
		int current_limit_pin;
		uint8_t I2C_addr;
};

#endif