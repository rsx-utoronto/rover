int A = 12;
int B = 11;
int I = 4;

int countTick = 0;
int countIndex = 0;
char precTick = 0;
char precIndex = 0;
char tick = 0;
char tickB =0;
char index = 0;

float range = 180.0;
float angle=45;
float currAngle = 0;
float maxAngle = 90.0;

// Motor B
int panPwm = 3;
int dir2 = 5;

int input;

long int t = 0;
int x = 0;
double t1 = 0;
int t2 = 0;
int ticks;
int ticks2;

int maxTicks=428;
int change = 1;
int change2 = 1;
int state = 0;

char key;

int getAngle(){
  tick = digitalRead(A);
  tickB = digitalRead(B);
  index = digitalRead(I);
  //Serial.print("index");
  //Serial.println(index);
  if(tick != precTick)
  {
    if(tick != tickB)
    {
      countTick = countTick + tick;
      precTick = tick;
    }
    else
    {
      countTick = countTick - tick;
      precTick = tick;
    }
    //Serial.print("tick :");
    //Serial.println(countTick);
  }
  
  if(index != precIndex)
  {
    if(countTick > 0)
    {
      countIndex = countIndex + index;
      precIndex = index;
    }
    else
    {
      countIndex = countIndex - index;
      precIndex = index;
    }
    //Serial.print("tick :");
    //Serial.println(countTick);
    //countTick = 0;
    //Serial.print("turn :");
    //Serial.println(countIndex);
  }
  return countTick;
}

void callibrateLow(){
  x = getAngle();
  if (change == 1){
    ticks = x;
    t1 = millis();
    change = 0;
  }

  if (x!=ticks){
    change = 1;
  }

  if (millis()-t1 >5000){
    countTick = 0;
    state = state + 1;
  }else{
    digitalWrite(dir2, LOW);
    analogWrite(panPwm, 250);
  }
  //angle = ticks*(180.0/(-114));
}

void getMax(){
  x = getAngle();
  if (change2 == 1){
    ticks2 = x;
    t2 = millis();
    change2 = 0;
  }

  if (x!=ticks){
    change2 = 1;
  }

  if (millis()-t2 >5000){
    maxTicks = x;
    state = state + 1;
  }else{
    digitalWrite(dir2, HIGH);
    analogWrite(panPwm, 250);
  }
  //angle = ticks*(180.0/(-114));
}

void setAngle(int target){
  x = getAngle();
  currAngle = x*(maxAngle/(-maxTicks));
  if (abs(target - currAngle) >= 2){
    if (target-currAngle>=0){
      digitalWrite(dir2, HIGH);
      analogWrite(panPwm, 250);
    }else if (target - currAngle<0){
      digitalWrite(dir2, LOW);
      analogWrite(panPwm, 250);
    }
  }else{
    analogWrite(panPwm, 0);
    state = state+ 1;
    input = 45;//random(10,maxAngle);
  }
}
void setup()
{
  pinMode(A, INPUT);
  pinMode(B, INPUT);
  pinMode(I, INPUT);
  Serial.begin(9600);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.setTimeout(2); // Set the timeout to 2ms, otherwise parsing can hang for up to a second
  Serial.println("Serial initialized.");
  
}

void loop()
{
      
  
  
  /*if (Serial.available()){
      key = Serial.read();
      if (key == 'd'){
        digitalWrite(dir2, LOW);
        analogWrite(panPwm, 250);
      }else if(key == 'a'){
        digitalWrite(dir2, HIGH);
        analogWrite(panPwm, 250);
      }else{
        analogWrite(panPwm, 0);
      }
    }*/
    
  Serial.println(getAngle());
  if (state == 0){
      callibrateLow();
    }else if(state == 1){
      getMax();
    }else if(state >= 2){
      if (Serial.available() > 0) {
      angle = Serial.parseFloat();
      Serial.println(angle);
      }
      setAngle(input);
    }
    
   
}





































































































































































































































































































































































































































































































































































































































































































































































