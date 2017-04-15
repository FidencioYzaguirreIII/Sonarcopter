#include <Servo.h>
#define sensor(A0)

Servo esc1; //ESC Stuff
Servo esc2;
Servo esc3;
Servo esc4;
int val;

int throtle; // Reciever Stuff
int pitch;
int roll;
int yaw;

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
  pitch = recevierReadingChecker(pulseIn(7,HIGH,25000));
  yaw = recevierReadingChecker(pulseIn(8,HIGH, 25000));
  roll = recevierReadingChecker(pulseIn(12,HIGH, 25000));

  esc1.write(90+throtle);
  delay(1000);
  esc2.write(90-throtle);
  delay(1000);
  esc3.write(90+throtle);
  delay(1000);
  esc4.write(90-throtle);
  delay(1000);

  Serial.print("ESC1: ");
  Serial.println(90+throtle);
  Serial.print("ESC2: ");
  Serial.println(90-throtle);
  Serial.print("ESC3: ");
  Serial.println(90+throtle);
  Serial.print("ESC4: ");
  Serial.println(90-throtle);

  

}

int recevierReadingChecker(int x){ // Clips Reviecer input to a certain range.
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

int escData(int x){ // Get throtle data converted.
  double data = (double)(((x-1070)/830)*90);//Multiply first then divide
  return (int) data;
}

int pitching(int x, int th){
  
}

int rolling(int x, int th){
  
}

int yawing(int x, int th){
  
}


