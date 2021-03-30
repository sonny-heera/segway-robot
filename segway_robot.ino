
/**
 * Sandeep Heera
 * segway_robot.ino
 * This program attempts to actualize a self-balancing inverted pendulum 
 * robot with a PID controller.
 */

#include "quaternionFilters.h"
#include "MPU9250.h"
#include <PID_v1.h>
#include <BricktronicsMotor.h>
#include <BricktronicsMegashield.h>

BricktronicsMotor m1(BricktronicsMegashield::MOTOR_1);  //initialize motor 1 object
BricktronicsMotor m2(BricktronicsMegashield::MOTOR_2);  //initialize motor 2 object

MPU9250 myIMU;

const double KP_AGGRESSIVE = 141.66;  //aggressive tuning parameters
const double KI_AGGRESSIVE = 824.62;
const double KD_AGGRESSIVE = 6.08;
const double CONSERVATIVE_DIV = 2;
const double KP_CONSERVATIVE = KP_AGGRESSIVE / CONSERVATIVE_DIV;  //conservative tuning parameters
const double KI_CONSERVATIVE = KI_AGGRESSIVE / CONSERVATIVE_DIV;
const double KD_CONSERVATIVE = KD_AGGRESSIVE * CONSERVATIVE_DIV;
const double CONSERVATIVE_OFFSET = 5;
const int IMU_OFFSET = 180;

double setpoint = 91.5;
double input = 0, output = 0, angle = 0, kp = KP_AGGRESSIVE, ki = KI_AGGRESSIVE, kd = KD_AGGRESSIVE;
double senVal = 0;

PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup()
{

  Wire.begin();
  m1.begin();
  m2.begin();
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  calibrate();
  getRoll();
}

void loop()
{
  double difference;
  input = myIMU.roll + IMU_OFFSET;  //get the reading from the sensor
  applyFiltering();                 //apply the Madgwick quaternion filter
  getRoll();
  double  currAngle = input;
  if(input > setpoint){
    difference = input - setpoint;
    input = setpoint - difference;
  }

  //check to see whether we should be using conservative or aggressive
  //tuning parameters
  if(abs(input - setpoint) <= CONSERVATIVE_OFFSET){
    kp = KP_CONSERVATIVE;
    ki = KI_CONSERVATIVE;
    kd = KD_CONSERVATIVE;
  }
  else{
    kp = KP_AGGRESSIVE;
    ki = KI_AGGRESSIVE;
    kd = KD_AGGRESSIVE;
  }
  
  pid.Compute();

  //move backwards if the angle is less than the setpoint
  if(currAngle < setpoint){
    m1.setFixedDrive(-output);
    m2.setFixedDrive(-output);
  }
  else if(currAngle > setpoint){  //move backwards
    m1.setFixedDrive(output);
    m2.setFixedDrive(output);
  }
  else{ //move in the opposite direction to reduce angular momentum
    m1.setFixedDrive(-m1.getFixedDrive() / 10);
    m2.setFixedDrive(-m2.getFixedDrive() / 10);
  }
}

/**
 * This function utilizes the Madgwick filter to get the roll of 
 * the filter.
 */
void getRoll() {
  myIMU.updateTime();
  
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
  
  myIMU.roll  *= RAD_TO_DEG;

  int myRollVal = myIMU.roll + 180;
  myIMU.count = millis();
  myIMU.sumCount = 0;
  myIMU.sum = 0;
}

/**
 * This function calibrates the MPU 9250.
 */
void calibrate() {
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    myIMU.initAK8963(myIMU.magCalibration);
  } // if (c == 0x71)
  else
  {
    while(1) ;
  }
}

/**
 * This method filters out the noise from the IMU.
 */
void applyFiltering() {
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();
    
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    myIMU.magbias[0] = +470.;
    myIMU.magbias[1] = +120.;
    myIMU.magbias[2] = +125.;

    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } 
}

