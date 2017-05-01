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
 * ESC Pins: 5, 10, 9, 6 (1,2,3,4)
 * RC Pins: 4, 7, 8, 12 (Throttle, Altitude, Elevation, Ruddor)
 * Sonar Pins: A1, A2 (Trig, Echo)
 * Gyroscope Pins: A4, A5 (SDA, SCL)
 */
#include <Wire.h>
#include <Servo.h>
#include "Ultrasonic.h"
#define sensor(A0)

Ultrasonic ultrasonic(A1,A2); //Can change pins later if need be
Servo escFrontLeft; //ESC Stuff
Servo escFrontRight;
Servo escBackLeft;
Servo escBackRight;
int val;

//These values are input target values as recieved from the remote controller
float throtle; // Reciever Stuff
float pitch;
float roll;
float yaw;

int distance; // Sonar

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

//PID variables
  struct PID pitchPID;
  struct PID rollPID;
  //Initialize calculated PID's
  float pitchAdjustment = 0;
  float rollAdjustment = 0;
  
void setup() {
  escFrontLeft.attach(5); // Callibrate first ESC
  escFrontRight.attach(10); // Callibrate second ESC 
  escBackLeft.attach(6); // Callibrate third ESC
  escBackRight.attach(9); // Calibrate fourth ESC.
  
  escFrontLeft.write(180);
  escFrontRight.write(180);
  escBackLeft.write(180);
  escBackRight.write(180);
  delay(2000);
  escFrontLeft.write(0);
  escFrontRight.write(0);
  escBackLeft.write(0);
  escBackRight.write(0);
  delay(2000);
  escFrontLeft.write(90);
  escFrontRight.write(90);
  escBackLeft.write(90);
  escBackRight.write(90);
  delay(5000);
  
  pinMode(4,INPUT); // Set up input pins for reciever
  pinMode(7,INPUT);
  pinMode(8,INPUT);
  pinMode(12,INPUT);
  Serial.begin(57600);

  //Initialize PID's here

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

  for (int cal_int = 0; cal_int < 3000 ; cal_int ++){                  //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          
  }
  gyro_x_cal /= 3000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 3000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 3000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  
}

void loop() {
  throtle = escData(recevierReadingChecker(pulseIn(4, HIGH, 25000))); // Get information from remote controller
  pitch = angle(recevierReadingChecker(pulseIn(7,HIGH,25000)));
  yaw = angle(recevierReadingChecker(pulseIn(8,HIGH, 25000)))/9*(-1);
  roll = angle(recevierReadingChecker(pulseIn(12,HIGH, 25000)));

  //Get data from sonar and accellerometer
  //distance = ultrasonic.Ranging(1);//Reads distance in CM
  
  mpu6050(); //updates accellerometer values
  pitchAdjustment = calculatePID(pitchPID, angle_pitch, pitch)/90;
  rollAdjustment = calculatePID(rollPID, angle_roll, roll)/90;
  delay(10);
  
  //Write throttle values
  escFrontLeft.write(safety((float)(90 + throtle + pitchAdjustment + rollAdjustment + yaw))); //+PIDPitch +PIDRoll
  escFrontRight.write(safety((float)(90 + throtle + pitchAdjustment - rollAdjustment - yaw)));
  escBackLeft.write(safety((float)(90 + throtle - pitchAdjustment + rollAdjustment - yaw)));
  escBackRight.write(safety((float)(90 + throtle - pitchAdjustment - rollAdjustment + yaw)));
  delay(20);
//  // Print troubleshooting data.
//  Serial.print("escFrontLeft: ");
//  Serial.println(safety((float)(90 + throtle + pitchAdjustment + rollAdjustment + yaw))); 
//  Serial.print("escFrontRight: ");
//  Serial.println(safety((float)(90 + throtle + pitchAdjustment - rollAdjustment - yaw))); 
//  Serial.print("escBackLeft: ");
//  Serial.println(safety((float)(90 + throtle - pitchAdjustment + rollAdjustment - yaw))); 
//  Serial.print("escBackRight: ");
//  Serial.println(safety((float)(90 + throtle - pitchAdjustment - rollAdjustment + yaw))); 


  // Print troubleshooting data.
  Serial.print("Pitch: " ); Serial.print(angle_pitch_output);
  Serial.print("| Roll: "); Serial.println(angle_roll_output);

}

float safety(float data){
  if(data > 130){
    return 130;
  }
  else if(data < 90){
    return 90;
  }
  else{
    return data;
  }

}

float recevierReadingChecker(float x){ // Clips Reviecer input to a certain range.
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

float escData(float x){ // Get throtle data converted.
  return ((x-1070)*20/830);
}


// Return values are place holders
// This function determines the desired angle for either pitching or yawing
float angle(float x){
  if ((1470 < x) ||(x <= 1520)){
    return 0;
  }
  else if(x < 1470){
    if ( (1230 < x) || (x <= 1310)){
      return -27;
    }
    else if (x <= 1230){
      if( x <= 1150){
        return -45;
      }
      else{
        return -36;
      }
    }
    else{
      if (x <= 1310){
        return -18;
      }
      else{
        return -9;
      }
    }
  }
  else{
    if ( (1680 < x) || (x <= 1760)){
      return 27;
    }
    else if (x <= 1680){
      if(1820 < x){
        return 45;
      }
      else{
        return 36;
      }
    }
    else{
      if (x <= 1600){
        return 9;
      }
      else{
        return 18;
      }
    }
    
  }
}


//Updates values in PID struct, and returns PID adjustment
//This could be used for pitch and roll calculations in conjunction
//with accellerometer readings, and/or height adjustments with sonar readings
float calculatePID(struct PID data, float currentValue, float currentTarget)
{
  data.target = currentTarget;
  unsigned long timeUpdate = millis();
  data.derivative = (long)((currentValue-data.value)/(timeUpdate - data.timeNow));
  data.integralAccumulator += (long)((data.target-currentValue) * (timeUpdate - data.timeNow));
  data.value = (long)currentValue;
  data.timeNow = timeUpdate;
  return float((data.target - data.value) * (data.proportionalCoefficient + data.integralAccumulator * data.integralCoefficient + data.derivative * data.derivativeCoefficient));
}

float constant = 0.0016;
void mpu6050(){

  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  angle_pitch += gyro_x * constant;                                  
  angle_roll += gyro_y * constant;     

  angle_pitch += angle_roll * sin(gyro_z * constant*(3.14159265358979/180));               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * constant*(3.14159265358979/180));               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.95 + angle_pitch_acc * 0.05;     
    angle_roll = angle_roll * 0.95 + angle_roll_acc * 0.05;        
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                      
    angle_roll = angle_roll_acc;                                        
    set_gyro_angles = true;                                            
  }
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.8 + angle_pitch * 0.2 ;   
  angle_roll_output = angle_roll_output * 0.8 + angle_roll * 0.2 ;     
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
