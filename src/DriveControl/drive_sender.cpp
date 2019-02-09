#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread.hpp>

#define KEYCODE_0 0x30 
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39
#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_SPACE 0x20

class TeleopRover{
    public:
        TeleopRover();
        void keyLoop();

    private:
        ros::NodeHandle nh_;
        double linear_, angular_, l_scale_, a_scale_;
        ros::Publisher twist_pub_;
};

TeleopRover::TeleopRover():
	linear_(0),
  	angular_(0),
  	l_scale_(1.0),
  	a_scale_(1.0)
{
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("drive", 1);
	
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
	ros::init(argc, argv, "drive_sender");
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

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the rover.");

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
	      	case KEYCODE_4:
	        	ROS_DEBUG("LEFT");
	        	ROS_INFO("LEFT");
		        angular_ = 1.0;
		        dirty = true;
		        alreadyStopped = false;
		        break;
		    case KEYCODE_6:
		        ROS_DEBUG("RIGHT");
		        ROS_INFO("RIGHT");
		        angular_ = -1.0;
		        dirty = true;
		        alreadyStopped = false;
		        break;
		    case KEYCODE_8:
		        ROS_DEBUG("UP");
		        ROS_INFO("UP");
		        linear_ = 1.0;
		        dirty = true;
		        alreadyStopped = false;
		        break;
		    case KEYCODE_2:
		        ROS_DEBUG("DOWN");
		        ROS_INFO("DOWN");
		        linear_ = -1.0;
		        dirty = true;
		        alreadyStopped = false;
		        break;
		    case KEYCODE_1:
		        ROS_DEBUG("LEFT B");
		        ROS_INFO("LEFT B");
		        angular_ = -0.5;
		        dirty = true;
		        alreadyStopped = false;
		        break;
		    case KEYCODE_3:
		        ROS_DEBUG("RIGHT B");
		        ROS_INFO("RIGHT B");
		        angular_ = -1.5;
		        dirty = true;
		        alreadyStopped = false;
		        break;
		     case KEYCODE_7:
		        ROS_DEBUG("LEFT F");
		        ROS_INFO("LEFT F");
		        angular_ = 0.5;
		        dirty = true;
		        alreadyStopped = false;
		        break;
		    case KEYCODE_9:
		        ROS_DEBUG("RIGHT F");
		        ROS_INFO("RIGHT F");
		        angular_ = 1.5;
		        dirty = true;
		        alreadyStopped = false;
		        break;
		    case KEYCODE_U:
				ROS_DEBUG("INC");
		        ROS_INFO("INC");
		        linear_= 1.5; //Positive increment 
		        dirty = true;
		        alreadyStopped = false;
		      break;
		    case KEYCODE_R:
				ROS_DEBUG("INC");
		        ROS_INFO("INC");
		        linear_= 1.5; //Positive increment 
		        dirty = true;
		        alreadyStopped = false;
		      break;
		    case KEYCODE_D:
				ROS_DEBUG("DEC");
		        ROS_INFO("DEC");
		        linear_= -1.5;//Negative increment 
		        dirty = true;
		        alreadyStopped = false;
		      break;
		    case KEYCODE_L:
				ROS_DEBUG("DEC");
		        ROS_INFO("DEC");
		        linear_= -1.5;//Negative increment 
		        dirty = true;
		        alreadyStopped = false;
		      break;
		    case KEYCODE_SPACE:
				ROS_DEBUG("STOP");
		        ROS_INFO("STOP");
		        linear_ = 0.0;
				angular_ = 0.0;
		        dirty = true;
		        alreadyStopped = false;
		        break;

		    default:
		    	if (alreadyStopped = false)
		    	{
		    		linear_ = 0.0;
					angular_ = 0.0;
		        	dirty = true;
		        	alreadyStopped = true;
		        	ROS_INFO("STOP");
		        	break;
		    	}
				continue;
	    }

	    geometry_msgs::Twist twist;
	    twist.angular.z = a_scale_*angular_;
	    twist.linear.x = l_scale_*linear_;
	    if(dirty ==true)
	    {
	      twist_pub_.publish(twist);    
	      dirty=false;
	    }
	  }


	  return;
}
