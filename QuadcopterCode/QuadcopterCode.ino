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
#include <Wire.h>
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

//IMU(accelerometer stuff)
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
/*
 * General Structure for PID controller 
 * coefficient terms need to manually tuned to our drone
 * This structure is designed to work with the calculatePID method to get our adjustment factor for pitch and roll calculations
 * Since pitch and roll are determined by difference between front and back; left and right respectively, PID is used to calculate
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
  pitchPID.target = 0; //Need to decide if this value should be transformed into an angle
  pitchPID.timeNow = millis();
  pitchPID.integralAccumulator = 0;
  rollPID.target = 0;
  rollPID.timeNow = millis();
  rollPID.integralAccumulator = 0;
  //initialize mpu6050
  Wire.begin();                                                                                                                                          
  
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  
}

void loop() {
  mpu6050();
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

void mpu6050(){

  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  angle_pitch += gyro_x * 0.0000611;                                   
  angle_roll += gyro_y * 0.0000611;                                    
  
  
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                      
    angle_roll = angle_roll_acc;                                        
    set_gyro_angles = true;                                            
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x3B);                                                    
  Wire.endTransmission();                                              
  Wire.requestFrom(0x68,14);                                           
  while(Wire.available() < 14);                                        
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temperature = Wire.read()<<8|Wire.read();                            
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 

}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                    
  Wire.endTransmission();                                              
  //Configure the accelerometer
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1C);                                                    
  Wire.write(0x10);                                                    
  Wire.endTransmission();                                              
  //Configure the gyro
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                    
  Wire.write(0x08);                                                    
  Wire.endTransmission();                                              
}
