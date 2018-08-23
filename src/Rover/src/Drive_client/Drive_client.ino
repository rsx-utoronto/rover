/* 
 * rosserial Drive
 * Blinks an LED on callback
 */


#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

int speedPins[] = {6, 9, 11, 5, 3, 10 };
int directionPins[] = { 4, 2 };

int speedl;
int speedr;
boolean driveMode;
int speedF = 70;
int speedB = -70;
int num = 1.00;

void messageCb( const geometry_msgs::Twist& msg){
  
  nh.loginfo("msg received");
  char resultAngular[8]; // Buffer big enough for 7-character float
  char resultLinear[8];
  dtostrf(msg.angular.z, 6, 2, resultAngular); // Leave room for too large numbers!
  dtostrf(msg.linear.x, 6, 2, resultLinear); // Leave room for too large numbers!

  nh.loginfo(resultAngular);
  nh.loginfo(resultLinear);
  
  if (msg.angular.z == 0.00)
  {
    if (msg.linear.x == num)
    {
      setLeftSpd(speedF);
      setRightSpd(speedF);
      nh.loginfo("moving forward");
    }
    else if (msg.linear.x == -num)
    {
      setLeftSpd(speedB);
      setRightSpd(speedB);
      nh.loginfo("moving backward");
    }
    else
    {
      stop();
      nh.loginfo("stopping");
    }
  }
  else
  {
    if (msg.angular.z == num)
    {
      doPivot(speedF);
    }
    else if (msg.angular.z == -num)
    {
      doPivot(speedB);
    }
  }
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<geometry_msgs::Twist> sub("turtle1/cmd_vel", &messageCb );

void setLeftSpd(int spd) {
      if(spd < 0) {
          digitalWrite(directionPins[0], LOW);
          for(int i=0; i<3; i++) {
              analogWrite(speedPins[i], -spd);
          }
      }
      else {
          digitalWrite(directionPins[0], HIGH);
          for(int i=0; i<3; i++) {
              analogWrite(speedPins[i], spd);
          }
      }
}

void setRightSpd(int spd) {
    if(spd < 0) {
        digitalWrite(directionPins[1], LOW);

        for(int i=3; i<6; i++) {
            analogWrite(speedPins[i], -spd);
        }
    }
    else {
        digitalWrite(directionPins[1], HIGH);

        for(int i=3; i<6; i++) {
            analogWrite(speedPins[i], spd);
        }
    }
}

// Pivot either direction
void doPivot(int pivot){
    setLeftSpd(pivot);
    setRightSpd(-pivot);
}

// Drive forward or backward
void forward(int speedl, int speedr){
    setLeftSpd(speedl);
    setRightSpd(speedr);
}

// Stop the motor
void stop(){
    setLeftSpd(0);
    setRightSpd(0);
}

void setup()
{ 
  //Set pins as outputs
  for (int i=0; i<6; i++) {
    pinMode(speedPins[i], OUTPUT);
    pinMode(directionPins[i], OUTPUT);
  }
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();

  delay(1);
}

