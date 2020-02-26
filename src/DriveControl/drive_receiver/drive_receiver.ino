/* 
 * rosserial Drive
 * Blinks an LED on callback
 */


#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

int speedPins[] = {3, 5, 6, 9, 10, 11 };
int directionPins[] = {23, 25, 2, 4, 7, 8};
int rel1 = 12;
int rel2 = 13;
int rel3 = A5;

int speedl;
int speedr;
boolean driveMode;

int speedF = 100;
int speedB = -100;

int speedPrevF= 100; 
int speedPrevB= -100; 

int hack = 2;

int num = 1.00;
int count = 0;
int count_reached = 500;

boolean startCounting = false;

char result[100];// Charcter array for final loginfo display message
char fspd[50], bspd[50], pspd[50];
const char *message; 

void accel(int, int);
void stop();
void doPivot(int);

void messageCb(const geometry_msgs::Twist& msg)
{
  startCounting = true;
  count = 0;
  //nh.loginfo("msg received");
  char resultAngular[8]; // Buffer big enough for 7-character float
  char resultLinear[8];
  //dtostrf(msg.angular.z, 6, 2, resultAngular); // Leave room for too large numbers!
  //dtostrf(msg.linear.x, 6, 2, resultLinear); // Leave room for too large numbers!

  //nh.loginfo(resultAngular);
  //nh.loginfo(resultLinear);
  
  //nh.loginfo(resultAngular);
  //nh.loginfo(resultLinear);
  
  sprintf(fspd,"%d",speedF);;// Converts speedF to appropriate type for loginfo display
  sprintf(bspd, "%d", speedB);;// Converts speedF to appropriate type for loginfo display
  /*sprintf(pspd, "%d", speedP);*/;// Converts speedF to appropriate type for loginfo display
  
  if (msg.linear.x == 25.00 || msg.linear.x == 50.00 || msg.linear.x == 75.00||msg.linear.x == 100.00||msg.linear.x == 125.00 || msg.linear.x == -1.5 || msg.linear.x == 1.5)
  {
    speedPrevF = speedF;
    speedPrevB = speedB;
    
    if (msg.linear.x== 25.00)
    {   
      speedF = 25;
      speedB = -25; 
      /*speedP = 25;*/
      message = "25";
    } else if (msg.linear.x == 50.0)
    {
      speedF = 50;
      speedB = -50; 
      /*speedP = 50;*/
      message = "50";
    }else if (msg.linear.x == 75.0)
    {
      speedF = 75;
      speedB = -75; 
      /*speedP = 75;*/
      message = "75";
    } else if (msg.linear.x == 100.0)
    {
      speedF = 100;
      speedB = -100; 
      /*speedP = 100;*/
      message = "100";
    } else if (msg.linear.x == 125.0)
    {
      speedF = 125;
      speedB = -125; 
      /*speedP = 125;*/
      message = "125";
    } else if (msg.linear.x == 1.5)
    {
      if (speedF <= 175){
        speedF += 10;
        speedB +=-10; 
        /*speedP +=10;*/
        message = "+10";
      }
      else {
        message = "cannot increase more";
      }
    } 
    else {
      if (speedF >= 10) {
        speedF -= 10;
        speedB -=-10; 
        /*speedP -=10;*/
        message = "-10";
      }
      else {
        message = "cannot decrease more";
      }
    } 
    strcpy(result,message);
    nh.loginfo(result);
  }
  
  if (msg.angular.z == 0.00)
  {
    if (msg.linear.x == 1.00)
    {
      /*setLeftSpd(speedF);
      setRightSpd(speedF);*/
      accel(speedF, speedPrevF);
      //message = "forward @ ";
      //strcpy(result,message);// Combines the output message with the appropriate speed 
      //strcat(result,fspd);
      nh.loginfo(fspd);
    }
    else if (msg.linear.x == -1.00)
    {
      /*setLeftSpd(-speedF);
      setRightSpd(-speedF);*/
      accel(speedB, speedPrevB); 
      message = "backward @ ";
      //strcpy(result,message);// Combines the output message with the appropriate speed 
      //strcat(result,bspd);
      nh.loginfo(bspd);
    }
    else
    {
      stop();
    }
  }
  else
  {
    if (msg.angular.z == 1.00)
    {
      doPivot(speedF);
      //message = "left @ ";
      //strcpy(result,message);// Combines the output message with the appropriate speed 
      //strcat(result,fspd);
      nh.loginfo(fspd);
    }
    else if (msg.angular.z == -1.00)
    {
      doPivot(-speedF);
      //message = "right @ ";
      //strcpy(result,message);// Combines the output message with the appropriate speed 
      //strcat(result,fspd);
      nh.loginfo(bspd);
    }
  }
//  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<geometry_msgs::Twist> sub("drive", &messageCb );

void setLeftSpd(int spd) {
      if(spd < 0) {
          for(int i=0; i<3; i++) {
              digitalWrite(directionPins[i], HIGH);
              analogWrite(speedPins[i], -spd);
          }
      }
      else {
          for(int i=0; i<3; i++) {
              digitalWrite(directionPins[i], LOW);
              analogWrite(speedPins[i], spd);
          }
      }
}

void setRightSpd(int spd) {
    if(spd < 0) {
        digitalWrite(directionPins[3], LOW);
        analogWrite(speedPins[3], -spd);
        for(int i=4; i<6; i++) {
              digitalWrite(directionPins[i], HIGH);
              analogWrite(speedPins[i], -spd);
          }
    }
    else {
        digitalWrite(directionPins[3], HIGH);
        analogWrite(speedPins[3], spd);
        for(int i=4; i<6; i++) {
            digitalWrite(directionPins[i], LOW);
            analogWrite(speedPins[i], spd);
        }
    }
}

void accel(int spd, int speedPrev) {
    if (spd > speedPrev){
       speedPrev = speedPrev + 1; 
       setRightSpd(speedPrev);
       setLeftSpd(speedPrev); 
    } 
    else if (spd < speedPrev){
        speedPrev = speedPrev - 1; 
        setRightSpd(speedPrev);
        setLeftSpd(speedPrev); 
    } 
    else {
        setRightSpd(spd);
        setLeftSpd(spd); 
    }  
}

// Pivot either direction
void doPivot(int pivot){
    setLeftSpd(-pivot);
    setRightSpd(+pivot);
}

// Drive forward or backward
void forward(int speedl, int speedr){
    setLeftSpd(speedl);
    setRightSpd(speedr);
}

// Stop the motor
void stop(){
    nh.loginfo("STOP");
  //  speedF = 0;
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
//  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  
  pinMode(rel1, OUTPUT);
  pinMode(rel2, OUTPUT);
  pinMode(rel3, OUTPUT);
}

void loop()
{  
  digitalWrite(rel1, HIGH);
  digitalWrite(rel2, HIGH);
  digitalWrite(rel3, HIGH);
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
  
//  forward(200, 200);
  
}

