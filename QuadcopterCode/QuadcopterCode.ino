/*
 * Construction Notes
 * 
 *       ESC Layout
 *      (1)     (2)
 *         \   /
 *          ( )
 *         /   \
 *      (4)     (3)
 * 
 * Pins in use:
 * ESC Pins: 5, 6, 9, 10
 * RC Pins: 4, 7, 8, 12
 * Sonar Pins: A1, A2
 */

#include <Servo.h>
#include "Ultrasonic.h"
#define sensor(A0)

Ultrasonic ultrasonic(A1,A2); //Can change pins later if need be
Servo esc1; //ESC Stuff
Servo esc2;
Servo esc3;
Servo esc4;
int val;
//These values are input target values as recieved from the remote controller
long throtle; // Reciever Stuff
long pitch;
long roll;
long yaw;
int distance;
// Movement Stuff
int moveNumber; //Determines how severly pitching, yawing, or rolling effects quadcopter.
/*
 * General Structure for PID controller 
 * coefficient terms need to manually tuned to our drone
 * This structure is designed to work with the calculatePID method to get our adjustment factor for pitch and roll calculations
 * Since ptich and roll are determined by difference between front and back; left and right respectively, PID is used to calculate
 * adjustment to these values
 */

struct PID{
  unsigned long timeNow;  //time stored in PID for integral accumulator and derivative calculator;
  long target;
  long integralAccumulator;
  long derivative;
  long value;
  long proportionalCoefficient;
  long integralCoefficient;
  long derivativeCoefficient;
};
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

  //Initialize PID's here
  struct PID pitchPID;
  struct PID rollPID;
  //Initialize PID coefficients
  pitchPID.integralCoefficient = 0;
  pitchPID.derivativeCoefficient = 0;
  pitchPID.proportionalCoefficient = 0;
  rollPID.integralCoefficient = 0;
  rollPID.derivativeCoefficient = 0;
  rollPID.proportionalCoefficient = 0;
  //Initialize PID other values:
  pitchPID.target = pitching(recevierReadingChecker(pulseIn(7,HIGH,25000)),throtle);  //Need to decide if this value should be transformed into an angle
  pitchPID.timeNow = millis();
  pitchPID.integralAccumulator = 0;
  rollPID.target = rolling(recevierReadingChecker(pulseIn(12,HIGH, 25000)),throtle);
  rollPID.timeNow = millis();
  rollPID.integralAccumulator = 0;
  
}

void loop() {
  throtle = escData(recevierReadingChecker(pulseIn(4, HIGH, 25000))); // Get information from remote controller
  pitch = pitching(recevierReadingChecker(pulseIn(7,HIGH,25000)),throtle);
  yaw = yawing(recevierReadingChecker(pulseIn(8,HIGH, 25000)),throtle);
  roll = rolling(recevierReadingChecker(pulseIn(12,HIGH, 25000)),throtle);
  distance = ultrasonic.Ranging(1);//Reads distance in CM

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
//Updates values in PID struct, and returns PID adjustment
//This could be used for pitch and roll calculations in conjunction
//with accellerometer readings, and/or height adjustments with sonar readings
long calculatePID(struct PID data, long currentValue, long currentTarget)
{
  data.target = currentTarget;
  unsigned long timeUpdate = millis();
  data.derivative = (currentValue-data.value)/(timeUpdate - data.timeNow);
  data.integralAccumulator += (data.target-currentValue) * (timeUpdate - data.timeNow);
  data.value = currentValue;
  data.timeNow = timeUpdate;
  return (data.target - data.value) * data.proportionalCoefficient + data.integralAccumulator * data.integralCoefficient + data.derivative * data.derivativeCoefficient;
}


