#include <ros/ros.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_0 0x30 
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_s 0x73
#define KEYCODE_c 0x63
#define KEYCODE_o 0x6f

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
	  
	    switch(c)
	    {
	      	case KEYCODE_0:
	        	ROS_INFO("OPEN SERVO0");
		        msg.data = "0";
		        dirty = true;
		        break;
			case KEYCODE_1:
			       	ROS_INFO("OPEN SERVO1");
			        msg.data = "1";
			        dirty = true;
			        break;
			case KEYCODE_2:
			        ROS_INFO("OPEN SERVO2");
			        msg.data = "2";
			        dirty = true;
			        break;
			case KEYCODE_3:
			        ROS_INFO("OPEN SERVO3");
			        msg.data = "3";
			        dirty = true;
			        break;
			case KEYCODE_c:
			        ROS_INFO("CLOSE ALL");
			        msg.data = "c";
			        dirty = true;
			        break;
			case KEYCODE_o:
			        ROS_INFO("OPEN ALL");
			        msg.data = "o";
			        dirty = true;
			        break;
			default:
				continue;
	    }

	     if(dirty ==true)
	    {
	      message_pub.publish(msg);    
	      dirty=false;
	    }
	  }


	  return;
}
