#include <Servo.h>
#define sensor(A0)

Servo esc1; //ESC Stuff
Servo esc2;
Servo esc3;
Servo esc4;
int val;

long throtle; // Reciever Stuff
long pitch;
long roll;
long yaw;

// Movement Stuff
int moveNumber; //Determines how severly pitching, yawing, or rolling effects quadcopter.

void setup() {
  esc1.attach(5); // Callibrate first ESC
  esc1.write(180);
  delay(2000);
  esc1.write(0);
  delay(2000);
  esc1.write(90);
  delay(2000);

  esc2.attach(6); // Callibrate second ESC 
  esc2.write(180);
  delay(2000);
  esc2.write(0);
  delay(2000);
  esc2.write(90);
  delay(2000);

  esc3.attach(9); // Callibrate third ESC
  esc3.write(180);
  delay(2000);
  esc3.write(0);
  delay(2000);
  esc3.write(90);
  delay(2000);

  esc4.attach(10); // Calibrate fourth ESC.
  esc4.write(180);
  delay(2000);
  esc4.write(0);
  delay(2000);
  esc4.write(90);
  delay(2000);

  pinMode(4,INPUT); // Set up input pins for reciever
  pinMode(7,INPUT);
  pinMode(8,INPUT);
  pinMode(12,INPUT);
  Serial.begin(9600);
}

void loop() {
  throtle = escData(recevierReadingChecker(pulseIn(4, HIGH, 25000))); // Get information from remote controller
  pitch = pitching(recevierReadingChecker(pulseIn(7,HIGH,25000)),throtle);
  yaw = yawing(recevierReadingChecker(pulseIn(8,HIGH, 25000)),throtle);
  roll = rolling(recevierReadingChecker(pulseIn(12,HIGH, 25000)),throtle);

  esc1.write(90+throtle); //Write Throtle to ESCS.
  esc2.write(90-throtle);
  esc3.write(90+throtle);
  esc4.write(90-throtle);
  delay(5);

  Serial.print("ESC1: ");
  Serial.println(90+throtle);
  Serial.print("ESC2: ");
  Serial.println(90-throtle);
  Serial.print("ESC3: ");
  Serial.println(90+throtle);
  Serial.print("ESC4: ");
  Serial.println(90-throtle);

  

}

long recevierReadingChecker(long x){ // Clips Reviecer input to a certain range.
  if(x < 1070){
    return 1070;
  }
  else if(x > 1900){
    return 1900;
  }
  else{
    return x;
  }
}

long escData(long x){ // Get throtle data converted.
  return ((x-1070)*90/830);
}

// The pitching, yawing, and rolling functions still need work.

long pitching(long x, long th){ // Get Pitching Data conveted
  if (th > 45){
    if(x > 1485){               // Determine ratio based on throtle.
      return ((x-1070)*10/830);
    }
    else{
      return -((x-1070)*10/830);
    }
  }
  else {
    if(x > 45){
      return -((x-1070)*10/830);
    }
    else{
      return ((x-1070)*10/830);
    }
  }
}

long rolling(long x, long th){ // Get rolling data converted
    if (th > 45){
    return ((x-1070)*10/830);
  }
  else {
     return -((x-1070)*10/830);
  }
}

long yawing(long x, long th){  // Get yawing data converted
    if (th > 45){
    return ((x-1070)*10/830);
  }
  else {
     return -((x-1070)*10/830);
  }
}


