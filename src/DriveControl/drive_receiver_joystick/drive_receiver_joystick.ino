/* 
 * rosserial Drive
 * Blinks an LED on callback
 */


#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

ros::NodeHandle  nh;

int speedPins[] = {6, 9, 11, 5, 3, 10 };
int directionPins[] = { 4, 2 };

int speedl;
int speedr;
boolean driveMode;
int speedF = 0;
int speedB = 0;
int rdegree =0;
int ldegree =0; 
int speedPrevF= 50; 
int speedPrevB= -50; 
/*int speedP = 50;*/
int num = 1.00;
int count = 0;
int count_reached = 500;
boolean right_turn = true; 
boolean startCounting = false;
int t_ratio = 0; 
char result[100];// Charcter array for final loginfo display message
char fspd[50], bspd[50], pspd[50];
const char *message; 

void messageCb( const geometry_msgs::Twist& msg){
  startCounting = true;
  count = 0;
  //nh.loginfo("msg received");
  char resultAngular[8]; // Buffer big enough for 7-character float
  char resultLinear[8];
  dtostrf(msg.angular.z, 6, 2, resultAngular); // Leave room for too large numbers!
  dtostrf(msg.linear.x, 6, 2, resultLinear); // Leave room for too large numbers!
  
  //nh.loginfo(resultAngular);
  //nh.loginfo(resultLinear);
  
  sprintf(fspd,"%d",speedF);;// Converts speedF to appropriate type for loginfo display
  sprintf(bspd, "%d", speedB);;// Converts speedF to appropriate type for loginfo display
  /*sprintf(pspd, "%d", speedP);*/;// Converts speedF to appropriate type for loginfo display
  
  speedF = expConvert(msg.linear.x, 1, 25, 90); 
  speedB = -1*speedF;   
  
  if ((msg.angular.z) == 0 && msg.linear.x == 0) { 
    stop(); }
  else {
    if ((msg.angular.z)>-0.2 && (msg.angular.z)<0.2){
      if (msg.linear.x>0){
        forward(speedF, speedF); 
        message = "forward @ ";
        strcpy(result,message);// Combines the output message with the appropriate speed 
        strcat(result,fspd);
      }
      else if (msg.linear.x<0){
        forward(speedB, speedB); 
        message = "backward @ ";
        strcpy(result,message);// Combines the output message with the appropriate speed 
        strcat(result,bspd);
      }
      nh.loginfo(result);
    }
    else {
      speedF = 60;
      speedB = -60;
      if (msg.linear.x >= 0) 
      {
        doTurn(speedF, msg.angular.z);  
        message = "moving f and turn @ ";
        strcpy(result,message);// Combines the output message with the appropriate speed 
        strcat(result,fspd);
      }
      else if (msg.linear.x < 0) {
        doTurn(speedB, msg.angular.z);
        message = "moving b and turn @ ";
        strcpy(result,message);// Combines the output message with the appropriate speed 
        strcat(result,bspd);
      }
      nh.loginfo(result);
    }
  }  
}
ros::Subscriber<geometry_msgs::Twist> sub("drive", &messageCb );

int expConvert(float val, int max_in, int min_out, int max_out){
      // Precondtions: -(max_out) <= val <= max_out    ,    min_out <= max_out
      // ===Example Call===
      // speedF = expConvert(msg.linear.x, 1, 25, 90);
      int val_exp = 0;
      int fromHigh = 0;
      if (val!=0){
        val_exp = int(exp(abs(val))*1000);
        fromHigh = int(exp(max_in)*1000);
      } else { return 0; }
      return map(val_exp, 1000, fromHigh, min_out, max_out);
      //NOTE: multiplies by 1000 to avoid rounding error when converting to int
}

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
    setLeftSpd(pivot);
    setRightSpd(-pivot);
}

void doTurn(int turn, float degree){ 
    if (degree<0){
      t_ratio = 1.0 + degree; 
      setLeftSpd(turn);
      setRightSpd(turn*t_ratio);
    } 
    else if (degree>0){
       t_ratio = 1.0 - degree;
       setRightSpd(turn);
       setLeftSpd(turn*t_ratio);
    }
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
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
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
}

