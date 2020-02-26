#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <rsx_esc.h>

ros::NodeHandle  nh;

//int led = 13;
int count = 0;
int count_reached = 500;
boolean startCounting = false;

//ESC *P1;
//ESC *P2;
//ESC *P3;
//ESC *P4;
//ESC *P5;
//ESC *P6;

void messageCb(const geometry_msgs::Twist& msg)
{
  startCounting = true;
  count = 0;

  char resultAngular[8]; // Buffer big enough for 7-character float
  char resultLinear[8];
  dtostrf(msg.angular.z, 6, 2, resultAngular); // Leave room for too large numbers!
  dtostrf(msg.linear.x, 6, 2, resultLinear); // Leave room for too large numbers!

  nh.loginfo(resultAngular);
  nh.loginfo(resultLinear);
}

ros::Subscriber<geometry_msgs::Twist> sub("drive", &messageCb );

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
 //pinMode(led, OUTPUT);    
  //ESC ESC1(2340.0, 24, 25, 8, 22, 23, B0001001);
  //P1=&ESC1;
  /*ESC ESC2(2340.0, 26, 27, 9, 28, 29, B0001101);
  ESC ESC3(2340.0, 33, 32, 7, 31, 30, B0001000);
  ESC ESC4(2340.0, 35, 34, 4, 37, 36, B0001100);
  ESC ESC5(2340.0, 39, 38, 5, 41, 40, B0001110);
  ESC ESC6(2340.0, 45, 44, 6, 43, 42, B1001100);*/ 
  nh.initNode();
  nh.subscribe(sub);
  //Serial.begin(9600);
}

void stop(){
    nh.loginfo("STOP");
}

// the loop routine runs over and over again forever:
void loop() {
  nh.spinOnce();
  delay(1);
  if (startCounting)
  {
    count++; 
  }
  if (count == count_reached)
  {
    count = 0;
    stop();  
    startCounting = false;
  }
  static ESC ESC1(2340.0, 24, 25, 8, 22, 23, B0001001);
  ESC1.set_vel(5000.0);
  //(*P1).set_vel(10.0);
  /*ESC1.set_vel(10.0);
  (*P1).set_vel(10.0);
  ESC3.set_vel(10.0);
  ESC4.set_vel(10.0);
  ESC5.set_vel(10.0);
  ESC6.set_vel(10.0);*/
}
  