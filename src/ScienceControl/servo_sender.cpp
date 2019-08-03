#include <ros/ros.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>

#define KEYCODE_0 0x30 
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_s 0x73
#define KEYCODE_c 0x63		
#define KEYCODE_o 0x6f

// GLOBALS.
int servo = 0; 
//PUBLISH FORMAT 'NUMBER(1-6)LETTER(O/C)'

class TeleopRover{
    public:
        TeleopRover();
        void keyLoop();

    private:
        ros::NodeHandle nh_;
        double linear_, angular_, l_scale_, a_scale_;
        ros::Publisher message_pub;
};

TeleopRover::TeleopRover():
	linear_(0),
  	angular_(0),
  	l_scale_(1.0),
  	a_scale_(1.0)
{
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	message_pub = nh_.advertise<std_msgs::String>("servo", 1);
	
}

int kfd = 0;
struct termios cooked, raw;
bool alreadyStopped = false;
bool key_pressed = false;

void quit(int sig)
{
	(void)sig;
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "servo_sender");
	TeleopRover teleop_rover;

	signal(SIGINT,quit);

	teleop_rover.keyLoop();
	  
	return(0);
}

void TeleopRover::keyLoop()
{
	char c;
	bool dirty = false;
 
	// get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file                         
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
	std_msgs::String msg;
	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the servo.");

	for(;;)
	{
	    // get the next event from the keyboard  
	    if(read(kfd, &c, 1) < 0)
	    {
	    	ROS_INFO("READKEYBOARDFAILED");
	      	perror("read():");
	      	exit(-1);
	    }

	    linear_=angular_=0;
	    ROS_DEBUG("value: 0x%02X\n", c);

		// Update servo choice.
		if(c==KEYCODE_0 || c==KEYCODE_1 || c==KEYCODE_2 || c==KEYCODE_3 || c==KEYCODE_4 || c==KEYCODE_5 || c==KEYCODE_6) {
			switch(c)
			{
				case KEYCODE_0:
					servo = 0;
					break;
				case KEYCODE_1:
					servo = 1;
					break;
				case KEYCODE_2:
					servo = 2;
					break;
				case KEYCODE_3:
					servo = 3;
					break;
				case KEYCODE_4:
					servo = 4;
					break;
				case KEYCODE_5:
					servo = 5;
					break;
				case KEYCODE_6:
					servo = 6;
					break;																									
			}
		}
		// Move servo.
		else if(c==KEYCODE_c || c==KEYCODE_o) {
			switch(c)
			{
				case KEYCODE_c:
					switch(servo)
					{
						case 0:
							ROS_INFO("OPEN ALL SERVOS");
							msg.data = "0o";
							dirty = true;
							break;
						case 1:
							ROS_INFO("OPEN SERVO1");
							msg.data = "1o";
							dirty = true;
							break;
						case 2:
							ROS_INFO("OPEN SERVO2");
							msg.data = "2o";
							dirty = true;
							break;
						case 3:
							ROS_INFO("OPEN SERVO3");
							msg.data = "3o";
							dirty = true;
							break;
						case 4:
							ROS_INFO("OPEN SERVO4");
							msg.data = "4o";
							dirty = true;
							break;
						case 5:
							ROS_INFO("OPEN SERVO5");
							msg.data = "5o";
							dirty = true;
							break;
						case 6:
							ROS_INFO("OPEN SERVO6");
							msg.data = "6o";
							dirty = true;
							break;
					}
					break;
				case KEYCODE_o:
					switch(servo)
					{
						case 0:
							ROS_INFO("CLOSE ALL SERVOS");
							msg.data = "0c";
							dirty = true;
							break;						
						case 1:
							ROS_INFO("CLOSE SERVO1");
							msg.data = "1c";
							dirty = true;
							break;
						case 2:
							ROS_INFO("CLOSE SERVO2");
							msg.data = "2c";
							dirty = true;
							break;
						case 3:
							ROS_INFO("CLOSE SERVO3");
							msg.data = "3c";
							dirty = true;
							break;
						case 4:
							ROS_INFO("CLOSE SERVO4");
							msg.data = "4c";
							dirty = true;
							break;
						case 5:
							ROS_INFO("CLOSE SERVO5");
							msg.data = "5c";
							dirty = true;
							break;
						case 6:
							ROS_INFO("CLOSE SERVO6");
							msg.data = "6c";
							dirty = true;
							break;
					}
					break;	
			}
		}

	    if(dirty ==true)
	    {
	      message_pub.publish(msg);    
	      dirty=false;
	    }
	}

	return;
}
