// Master


// The Addresses are relative, so A->B->C, then A being the master, then B must be 2 and C must be 3 and so on...

#include <Wire.h>

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() {
  int counter=0;
  Wire.requestFrom(2, 2);    // request 6 bytes from slave device #8

  while (Wire.available()) { // slave may send less than requested
    byte a=Wire.read();
    byte b=Wire.read();
   //byte c = Wire.read(); // receive a byte as character
    counter=a;
    counter=(counter<<8)|b;
    Serial.println(counter);         // print the character
  }

  Wire.requestFrom(3, 2);    // request 6 bytes from slave device #2, adjust for the number of bits (int = 1, char = number of chars)

  while (Wire.available()) { // slave may send less than requested
    byte d=Wire.read();
    byte e=Wire.read();
    //byte c = Wire.read(); // receive a byte as character
    counter=d;
    counter=(counter<<8)|e;
    Serial.println(counter);         // print the character
  }
  Serial.println();
  delay(10);
}
