// Slave

// The Addresses are relative, so A->B->C, then A being the master, then B must be 2 and C must be 3 and so on...


#include <Wire.h>

#define outputA3 11 // Most Left Port
#define outputB3 12
//#define address 2
int aState;
int aLastState;
int counter = 0;
byte counterbyte[2];

void setup() {
  Wire.begin(3);                // join i2c bus with address #2
  Wire.onRequest(requestEvent); // register event
  pinMode(outputA3,INPUT);
  pinMode(outputB3,INPUT);
  Serial.begin(9600);
  aLastState = digitalRead(outputA3);
}

 void loop() { 
   aState = digitalRead(outputA3); // Reads the "current" state of the outputA3
   
   //Serial.println(counter);
   
   // If the previous and the current state of the outputA3 are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB3 state is different to the outputA3 state, that means the encoder is rotating clockwise
     if (digitalRead(outputB3) != aState) { 
       counter ++;
     } else {
       counter --;
     }
   } 
   aLastState = aState; // Updates the previous state of the outputA3 with the current state
   Serial.println(counter);
   delay(10);
 }

 void requestEvent() {
  counterbyte[0]=(counter>>8)&0xFF;
  counterbyte[1]=counter&0xFF;
//  counterbyte[2]=address;
  Wire.write(counterbyte,2); // respond with message of 6 bytes
  // as expected by master

  // implement bit manipulation stuff so..
}
