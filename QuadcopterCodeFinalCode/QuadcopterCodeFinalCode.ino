/*
 * File Name: QuadcopterCodeFinalCode 
 * 
 * Authors: Guy Maor
 *          Fidencio Yzaguire III
 *          Thomas Jarvinen
 *          Julian Barata
 *          
 * Description: This program is the controller for a quadcopter.
 *              It must read in receiver and accelerometer values and
 *              process the signals with PID controllers. The program then
 *              sends a Pulse Width Modulation signal to the electronic speed
 *              controllers of the drone.
 *              
 * Procedure names: setup() Sets up the initial values of the drone.
 *                  loop() Runs the entire code periodically.
 *                  shutdown() If the drone spins out of control, this program stops it.
 *                  safety() Makes sure the ESCs do not receive a damaging PWM signal.
 *                  recevierReadingChecker() Makes sure the receiver values are within the correct range.
 *                  escData() Converts the receiver values to an appropriate ESC PWM signal value.
 *                  angle() Convertes the receiver values for pitch and roll to angles.
 *                  calculatePID() Performs the PID calculations.
 *                  mpu6050() Converts raw Accelerometer values to usable values.
 *                  read_mpu_6050_data() Reads in the raw accelerometer values.
 *                  setup_mpu_6050_registers() Sets up the accelerometer.
 *                  
 * Construction Notes:
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
 * 
 * Outside references: Part of the code was referenced from outside sources;
 *                     however, it only belongs to part of the code and the entire
 *                     code was implemented orignially.
 */
#include <Wire.h>
#include <Servo.h>
#include "Ultrasonic.h"

//These are the interfaces for the ESCs.
Servo escFrontLeft; 
Servo escFrontRight;
Servo escBackLeft;
Servo escBackRight;
int val;

//These values are input target values as recieved from the remote controller
double throtle; 
float pitch;
float roll;
double yaw;



//Determines how severly pitching, yawing, or rolling effects quadcopter.
int moveNumber; 

//IMU(accelerometer stuff)
double gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
double gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float angle_pitch_output_cal, angle_roll_output_cal;

//Itterates between the Receiver channels to improve efficiency.
char sensorItterator;

/*
 * General Structure for PID controller 
 * coefficient terms need to manually tuned to our drone
 * This structure is designed to work with the calculatePID method to get our adjustment factor for pitch and roll calculations
 * Since pitch and roll are determined by difference between front and back; left and right respectively, PID is used to calculate
 * adjustment to these values
 */

struct PID{
  unsigned long timeNow;  //time stored in PID for integral accumulator and derivative calculator;
  double target;
  double integralAccumulator;
  double derivative;
  double value;
  double proportionalCoefficient;
  double integralCoefficient;
  double derivativeCoefficient;
};

//PID variables
  struct PID pitchPID;
  struct PID rollPID;
  //Initialize calculated PID's
  double pitchAdjustment;
  double rollAdjustment;

/*
 * Procedure name: setup()
 * Author:  Guy Maor
 *          Fidencio Yzaguire III
 *          Thomas Jarvinen
 *          Julian Barata
 * Description: This sets up all the initial values of the quadcopter.
 */
void setup() {
  pinMode(4,INPUT); // Set up input pins for reciever
  pinMode(7,INPUT);
  pinMode(8,INPUT);
  pinMode(12,INPUT);
  Serial.begin(57600);

  //Initialize PID's here

  //Initialize PID coefficients
  pitchPID.integralCoefficient = 0;
  pitchPID.derivativeCoefficient = 0;//0.2
  pitchPID.proportionalCoefficient = 1;
  rollPID.integralCoefficient = 0;//0.001
  rollPID.derivativeCoefficient = 0.2;//0.2;
  rollPID.proportionalCoefficient = 1;
  //Initialize PID other values:
  pitchPID.target = 0; //Need to decide if this value should be transformed into an angle
  pitchPID.timeNow = millis();
  pitchPID.integralAccumulator = 0;
  rollPID.target = 0;
  rollPID.timeNow = millis();
  rollPID.integralAccumulator = 0;

  //initialize mpu6050
  Wire.begin();                                                                                                                                          
  
  setup_mpu_6050_registers();                                       

  //This calibrates the gyroscope. The more itterations the more accurate it becomes.
  int itterations = 500;
  for (int cal_int = 0; cal_int < itterations ; cal_int ++){                  
    read_mpu_6050_data();                                              
    gyro_x_cal += gyro_x;                                              
    gyro_y_cal += gyro_y;                                             
    gyro_z_cal += gyro_z;                                              
    delay(3);
    Serial.println(itterations - cal_int);                                                          
  }
    gyro_x_cal /= itterations;
    gyro_y_cal /= itterations;
    gyro_z_cal /= itterations;

  //Sets the pins for the ESCs.
  escFrontLeft.attach(5); // Callibrate first ESC
  escFrontRight.attach(10); // Callibrate second ESC 
  escBackLeft.attach(6); // Callibrate third ESC
  escBackRight.attach(9); // Calibrate fourth ESC.

  //calibrates ESCs.
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

  //initial sensor values.
  pitch = angle(recevierReadingChecker(pulseIn(8,HIGH,25000)));
  yaw = angle(recevierReadingChecker(pulseIn(12,HIGH, 25000)))/20*(-1);
  roll = angle(recevierReadingChecker(pulseIn(7,HIGH, 25000)));
  throtle = escData(recevierReadingChecker(pulseIn(4,HIGH,25000)));

  //This is to make sure the receiver throttle is on low before starting the drone.
  if(throtle>0)
  {
      delay(100);
      throtle = escData(recevierReadingChecker(pulseIn(4,HIGH,25000)));
  }
  sensorItterator = 0;
}

double bias1 = 2.0;
double bias2 = 2.0;

/*
 * Procedure name: loop()
 * Author:  Guy Maor
 *          Fidencio Yzaguire III
 *          Thomas Jarvinen
 *          Julian Barata
 * Description: This performs all the drone operations periodically.
 */
void loop() {

  //This program itterates between each channel of the receiver so that it does not constantly read
  //from each receiver in the loop. This saves a significant amount of time.
  switch(sensorItterator)
  {
    case 0:
      pitch = angle(recevierReadingChecker(pulseIn(8,HIGH,25000))); 
      break;
    case 1:
      yaw = angle(recevierReadingChecker(pulseIn(12,HIGH, 25000)))/20*(-1);
      break;
    case 2:
      roll = angle(recevierReadingChecker(pulseIn(7,HIGH, 25000)));
      break;
    case 3:
      throtle = escData(recevierReadingChecker(pulseIn(4,HIGH,25000)));
      break;
  }
  sensorItterator++;
  sensorItterator%=4;

  //Updates accelerometer values.
  mpu6050();

  
   //The drone must stay within a certain angle range or else it will shut off.
  if((angle_pitch_output > 35) || (angle_pitch_output < -35) || (angle_roll_output > 35) || (angle_roll_output < -35)){
    shutdown();
  }
  

  //This is to make sure the PID doesn't work if the receiver is turned off.
  if(throtle > 1){ 
    //If it's on, it utilizes the PID.
    pitchAdjustment = calculatePID(pitchPID, angle_pitch_output-angle_pitch_output_cal, pitch);
    rollAdjustment = calculatePID(rollPID, angle_roll_output-angle_roll_output_cal, roll);

    //Updates the ESC PWM values.
    escFrontLeft.write(safety((double)(90 + throtle + pitchAdjustment + rollAdjustment + yaw))); //+PIDPitch +PIDRoll
    escFrontRight.write(safety((double)(90 + throtle + pitchAdjustment - rollAdjustment - yaw+bias1)));
    escBackLeft.write(safety((double)(90 + throtle - pitchAdjustment + rollAdjustment - yaw)));
    escBackRight.write(safety((double)(90 + throtle - pitchAdjustment - rollAdjustment + yaw+bias2)));

  }
  else{
    //If it is off, it takes in calibrates the angle for a smoother takeoff.
    angle_pitch_output_cal = angle_pitch_output;
    angle_roll_output_cal = angle_roll_output;

    //Updates the ESC PWM values.
    read_mpu_6050_data();                                              
    escFrontLeft.write(safety((double)(90 + throtle)));
    escFrontRight.write(safety((double)(90 + throtle+bias1)));
    escBackLeft.write(safety((double)(90 + throtle)));
    escBackRight.write(safety((double)(90 + throtle+bias2)));
    
  }

    Serial.print("Pitch: " ); Serial.print(angle_pitch_output-angle_pitch_output_cal);
    Serial.print("| Roll: "); Serial.println(angle_roll_output-angle_roll_output_cal);


}

/*
 * Procedure name: shutdown()
 * Author:  Guy Maor
 *          Fidencio Yzaguire III
 *          Thomas Jarvinen
 *          Julian Barata
 * Description: This turns off the drone when it goes out of control.
 */
void shutdown(){

  //Writes 90 to all the ESCs so they stop moving.
  escFrontLeft.write(90);
  escFrontRight.write(90);
  escBackLeft.write(90);
  escBackRight.write(90);
  int i = 0;
  //Goes into an infinite loop.
  while(i == 0){
    i =0;
  }
}

/*
 * Procedure name: safety()
 * Author:  Guy Maor
 *          Fidencio Yzaguire III
 *          Thomas Jarvinen
 *          Julian Barata
 * Description: This makes sure the ESCs do not exceed a certain PWM signal
 * @param data takes in the value that is to be corrected.
 * @return the safe ESC drone value.
 */
float safety(float data){
  //115 is a safe PWM signal value that the ESCs should not exceed.
  if(data > 115){
    return 115;
  }
  else if(data < 90){
    return 90;
  }
  else{
    return data;
  }

}

/*
 * Procedure name: recevierReadingChecker()
 * Author:  Guy Maor
 *          Fidencio Yzaguire III
 *          Thomas Jarvinen
 *          Julian Barata
 * Description: Modifies the receiver values to fit in a desirable range.
 * @param data the receiver value that is to be changed.
 * @return the resulting desirable receiver values.
 */
float recevierReadingChecker(float x){ // Clips Reviecer input to a certain range.

  //The value is limited from 1070 to 1900
  if(x < 1070){
    return 1070.0;
  }
 if(x > 1900){
    return 1900.0;
  }
  return x;
}

/*
 * Procedure name: escData()
 * Author:  Guy Maor
 *          Fidencio Yzaguire III
 *          Thomas Jarvinen
 *          Julian Barata
 * Description: Converts the receiver values to a usable ESC PWM signal.
 * @param data The receiver value.
 * @return The ESC PWM signal.
 */
float escData(float x){
  return ((x-1070)*10.0/830.0);
}

/*
 * Procedure name: angle()
 * Author:  Guy Maor
 *          Fidencio Yzaguire III
 *          Thomas Jarvinen
 *          Julian Barata
 * Description: Converts the receiver value of pitch and roll to an angle.
 * @param data The receiver value.
 * @return The desired angle.
 */
float angle(float x){
  if ((1470 < x) ||(x <= 1520)){
    return 0;
  }
  else if(x < 1470){

    //This code does a binary search to find the desired angle.
    //This is to increase efficiency.
    if ( (1230 < x) || (x <= 1310)){
      return 9;
    }
    else if (x <= 1230){
      if( x <= 1150){
        return 15;
      }
      else{
        return 12;
      }
    }
    else{
      if (x <= 1310){
        return 6;
      }
      else{
        return 3;
      }
    }
  }
  else{
    if ( (1680 < x) || (x <= 1760)){
      return -9;
    }
    else if (x <= 1680){
      if(1820 < x){
        return -15;
      }
      else{
        return -12;
      }
    }
    else{
      if (x <= 1600){
        return -3;
      }
      else{
        return -6;
      }
    }
    
  }
}

/*
 * Procedure name: calculatePID()
 * Author:  Guy Maor
 *          Fidencio Yzaguire III
 *          Thomas Jarvinen
 *          Julian Barata
 * Description: Calculates the PID values of the drone.
 * @param data The PID structure.
 * @param currentValue The sensor value from the accelerometer.
 * @param currentTarget The desired sensor value that we want to achieve.
 * @return The adjustment value produced by the PID.
 */
double calculatePID(struct PID data, float currentValue, float currentTarget)
{
  data.target = currentTarget;
  unsigned long timeUpdate = millis();
  data.derivative = (double)((currentValue-data.value)/(timeUpdate - data.timeNow));
  data.integralAccumulator += (double)((data.target-currentValue) * (timeUpdate - data.timeNow));
  data.value = (double)currentValue;
  data.timeNow = timeUpdate;
  return double((data.target - data.value) * (data.proportionalCoefficient + data.integralAccumulator * data.integralCoefficient + data.derivative * data.derivativeCoefficient));
}

/*
 * Procedure name: mpu6050()
 * Author:  Outside Source
 * Description: Converts the raw accelerometer values into usable accelerometer values.
 */
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
  angle_pitch_output = (angle_pitch_output * 0.996 + angle_pitch * 0.004);   
  angle_roll_output = (angle_roll_output * 0.996 + angle_roll * 0.004);     
}

/*
 * Procedure name: read_mpu_6050_data()
 * Author:  Outside Source
 * Description: Reads in raw accelerometer values. This
 *              procedure uses I2C to communicate with the
 *              accelerometer.
 */
void read_mpu_6050_data(
{
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x3B);                                                    
  Wire.endTransmission();                                              
  Wire.requestFrom(0x68,14);   

  //Waits for I2C to be available.
  while(Wire.available() < 14);

  //Reads in the raw values. 16 bits each.
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                                            
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}

/*
 * Procedure name: setup_mpu_6050_registers()
 * Author:  Outside Source
 * Description: Sets up the accelerometer
 */
void setup_mpu_6050_registers()
{
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
