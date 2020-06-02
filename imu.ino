#include "Wire.h"   
#include "MPU9250.h"
#include "Quaternion.hpp"

#define CALIBRATION_GYRO 0
#define CALIBRATION_MAG 0
#define SERIAL_DEBUG 0

uint8_t Gscale = GFS_250DPS, Ascale = AFS_4G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;         
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float motion = 0; // check on linear acceleration to determine motion
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
const float rad2deg = 180.0f / pi;
const float deg2rad = pi / 180.0f;

float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
bool wakeup;

// Pin definitions
int  intPin1 = 2;  //  MPU9250 1 interrupt
int  intPin2 = 3;  //  MPU9250 2 interrupt
int  myLed  = 13; // red led

volatile bool intFlag1 = false;
volatile bool intFlag2 = false;
bool newMagData = false;

int16_t MPU9250Data1[7], MPU9250Data2[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount1[3], magCount2[3];    // Stores the 16-bit signed magnetometer sensor output
float   magCalibration1[3] = {0, 0, 0}, magCalibration2[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float   temperature1, temperature2;    // Stores the MPU9250 internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// These can be measured once and entered here or can be calculated each time the device is powered on
int32_t   gyroBias1[3] = {414, -265, 20};
float accelBias1[3] = {-1.66613769f, -2.16113281f, -0.33935546f};

int32_t   gyroBias2[3] = {0, 0, 0};
float accelBias2[3] = {0.0f, 0.0f, 0.0f};

float   magBias1[3] = {415.33, -209.91, -229.41}, magScale1[3]  = {1.20, 0.91, 0.93}; // Bias corrections for gyro and accelerometer
float   magBias2[3] = {71.04, 122.43, -36.90}, magScale2[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer


uint32_t delt_t1, delt_t2 = 0;                      // used to control display output rate
uint32_t count1 = 0, sumCount1 = 0, count2 = 0, sumCount2 = 0;         // used to control display output rate
float pitch1, yaw1, roll1, pitch2, yaw2, roll2;                   // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float A12, A22, A31, A32, A33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat1 = 0.0f, sum1 = 0.0f, deltat2 = 0.0f, sum2 = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate1 = 0, lastUpdate2 = 0; // used to calculate integration interval
uint32_t Now1 = 0, Now2 = 0;                         // used to calculate integration interval

float ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1; // variables to hold latest sensor data values 
float g1[3],a1[3],m1[3];
Quaternion orien(1.0f,0.,0.,0.);
float ax2, ay2, az2, gx2, gy2, gz2, mx2, my2, mz2; // variables to hold latest sensor data values 
float lin_ax1, lin_ay1, lin_az1;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float lin_ax2, lin_ay2, lin_az2;             // linear acceleration (acceleration with gravity component subtracted)
float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

MPU9250 MPU9250(intPin1); // instantiate MPU9250 class
uint32_t last_send_time1 = 0; 
uint32_t delta_send_time1 = 0; 

void setup()
{
  Serial.begin(115200);
  delay(1000);
  
  Wire.begin(); // set master mode, default on SDA/SCL for Ladybug   
  Wire.setClock(400000); // I2C frequency at 400 kHz
  delay(1000);

  MPU9250.I2Cscan(); // should detect BME280 at 0x77, MPU9250 at 0x71 
  
  // Set up the interrupt pin, it's set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with orange led on (active HIGH)

  pinMode(intPin1, INPUT);
  pinMode(intPin2, INPUT);

   /* Configure the MPU9250 */
  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("MPU9250 9-axis motion sensor...");
  uint8_t c = MPU9250.getMPU9250ID(MPU1);
  Serial.print("MPU9250_1 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  uint8_t d = MPU9250.getMPU9250ID(MPU2);
  Serial.print("MPU9250_2 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  delay(1000);
  
  // if (c == 0x71 && d == 0x71 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
  if (c == 0x71)
  {  
    Serial.println("MPU9250s 1 is online...");
    
    delay(1000);

  // get sensor resolutions, only need to do this once, same for both MPU9250s for now
  aRes = MPU9250.getAres(Ascale);
  gRes = MPU9250.getGres(Gscale);
  mRes = MPU9250.getMres(Mscale);

#if CALIBRATION_GYRO
    MPU9250.calibrateMPU9250(MPU1, gyroBias1, NULL); // Calibrate gyro and accelerometers, load biases in bias registers
    Serial.println("MPU1 accel biases (g)"); Serial.println(accelBias1[0],10); Serial.println(accelBias1[1],8); Serial.println(accelBias1[2],8);
    Serial.println("MPU1 gyro biases (dps)"); Serial.println(gyroBias1[0]); Serial.println(gyroBias1[1]); Serial.println(gyroBias1[2]);
    Serial.println("Finished!! Please reset");
    while(1);
#endif
  MPU9250.GyroBiasWriteToReg(MPU1, gyroBias1);
  // MPU9250.calibrateMPU9250(MPU2, gyroBias2, accelBias2); // Calibrate gyro and accelerometers, load biases in bias registers
  // Serial.println("MPU2 accel biases (mg)"); Serial.println(1000.*accelBias2[0]); Serial.println(1000.*accelBias2[1]); Serial.println(1000.*accelBias2[2]);
  // Serial.println("MPU2 gyro biases (dps)"); Serial.println(gyroBias2[0]); Serial.println(gyroBias2[1]); Serial.println(gyroBias2[2]);
  // delay(1000); 
  
  MPU9250.initMPU9250(MPU1, Ascale, Gscale, sampleRate); 
  // MPU9250.initMPU9250(MPU2, Ascale, Gscale, sampleRate); 
  Serial.println("MPU9250s 1 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte e = MPU9250.getAK8963CID(MPU1);  // Read WHO_AM_I register for AK8963
  Serial.print("AK8963 1 "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
  // byte f = MPU9250.getAK8963CID(MPU2);  // Read WHO_AM_I register for AK8963
  // Serial.print("AK8963 2 "); Serial.print("I AM "); Serial.print(f, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
  delay(1000); 
  
  // Get magnetometer calibration from AK8963 ROM
  MPU9250.initAK8963Slave(MPU1, Mscale, Mmode, magCalibration1); Serial.println("AK8963 1 initialized for active data mode...."); // Initialize device 1 for active mode read of magnetometer
  Serial.println("Calibration values for mag 1: ");
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration1[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration1[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration1[2], 2);
  // MPU9250.initAK8963Slave(MPU2, Mscale, Mmode, magCalibration2); Serial.println("AK8963 2 initialized for active data mode...."); // Initialize device 2 for active mode read of magnetometer
  // Serial.println("Calibration values for mag 2: ");
  // Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration2[0], 2);
  // Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration2[1], 2);
  // Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration2[2], 2);
  
 // Comment out if using pre-measured, pre-stored offset biases
#if CALIBRATION_MAG
  MPU9250.magcalMPU9250(MPU1, magBias1, magScale1);
  Serial.println("AK8963 1 mag biases (mG)"); Serial.println(magBias1[0]); Serial.println(magBias1[1]); Serial.println(magBias1[2]); 
  Serial.println("AK8963 1 mag scale (mG)"); Serial.println(magScale1[0]); Serial.println(magScale1[1]); Serial.println(magScale1[2]); 
  while(1);
#endif
  // MPU9250.magcalMPU9250(MPU2, magBias2, magScale2);
  // Serial.println("AK8963 2 mag biases (mG)"); Serial.println(magBias2[0]); Serial.println(magBias2[1]); Serial.println(magBias2[2]); 
  // Serial.println("AK8963 2 mag scale (mG)"); Serial.println(magScale2[0]); Serial.println(magScale2[1]); Serial.println(magScale2[2]); 
  // delay(2000); // add delay to see results before serial spew of data
 
  
  attachInterrupt(digitalPinToInterrupt(intPin1), myinthandler1, RISING);  // define interrupt for intPin output of MPU9250 1
  attachInterrupt(digitalPinToInterrupt(intPin2), myinthandler2, RISING);  // define interrupt for intPin output of MPU9250 2

  
  }
  else
  {
    Serial.print("Could not connect to MPU9250 1: 0x"); Serial.println(c, HEX);
    Serial.print("Could not connect to MPU9250 2: 0x"); Serial.println(d, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }


   digitalWrite(myLed, LOW); // turn off led when using flash memory


  
}

void loop()
{
  // digitalWrite(myLed, HIGH);
   if(intFlag1 == true) {   
      intFlag1 = false; 

    MPU9250.readMPU9250Data(MPU1, MPU9250Data1); // INT cleared on any read
   
    // Now we'll calculate the accleration value into actual g's
     ax1 = (float)MPU9250Data1[0]*aRes - accelBias1[0];  // get actual g value, this depends on scale being set
     ay1 = (float)MPU9250Data1[1]*aRes - accelBias1[1];   
     az1 = (float)MPU9250Data1[2]*aRes - accelBias1[2];  

    // Calculate the gyro value into actual degrees per second
     gx1 = (float)MPU9250Data1[4]*gRes*deg2rad;  // get actual gyro value, this depends on scale being set
     gy1 = (float)MPU9250Data1[5]*gRes*deg2rad;  
     gz1 = (float)MPU9250Data1[6]*gRes*deg2rad; 
  
//    if( MPU9250.checkNewMagData() == true) { // wait for magnetometer data ready bit to be set
      MPU9250.readMagData(MPU1, magCount1);  // Read the x/y/z adc values
  
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
      mx1 = (float)magCount1[0]*mRes*magCalibration1[0] - magBias1[0];  // get actual magnetometer value, this depends on scale being set
      my1 = (float)magCount1[1]*mRes*magCalibration1[1] - magBias1[1];  
      mz1 = (float)magCount1[2]*mRes*magCalibration1[2] - magBias1[2];  
      mx1 *= magScale1[0];
      my1 *= magScale1[1];
      mz1 *= magScale1[2]; 
//    }
   
    Now1 = micros();
    deltat1 = ((Now1 - lastUpdate1)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate1 = Now1;

    Quaternion omega(0, gx1,gy1,gz1);
    Quaternion delta_q = orien * omega * 0.5f;
    orien = orien + (delta_q * deltat1);
    orien.normalize();

    orien.toEulerAngle(&roll1, &pitch1,&yaw1);
    roll1 *= rad2deg;
    pitch1 *= rad2deg;
    yaw1 *= rad2deg;
   
    sum1 += deltat1; // sum for averaging filter update rate
    sumCount1++;
    if(sum1 >= 1.0f) //1HZ 
    {
      if (SERIAL_DEBUG)
      {
        Serial.print("rate 1 = "); Serial.print((float)sumCount1/sum1, 2); Serial.println(" Hz");
        Serial.print("ax1 = "); Serial.print((int)1000*ax1);  
        Serial.print(" ay1 = "); Serial.print((int)1000*ay1); 
        Serial.print(" az1 = "); Serial.print((int)1000*az1); Serial.println(" mg");
        Serial.print("gx1 = "); Serial.print( gx1, 4); 
        Serial.print(" gy1 = "); Serial.print( gy1, 4); 
        Serial.print(" gz1 = "); Serial.print( gz1, 4); Serial.println(" deg/s");
        Serial.print("mx1 = "); Serial.print( (int)mx1 ); 
        Serial.print(" my1 = "); Serial.print( (int)my1 ); 
        Serial.print(" mz1 = "); Serial.print( (int)mz1 ); Serial.println(" mG");
        Serial.print("MPU9250 1 Yaw, Pitch, Roll: ");
        Serial.print(yaw1, 2);
        Serial.print(", ");
        Serial.print(pitch1, 2);
        Serial.print(", ");
        Serial.println(roll1, 2);
      }
      sum1 = 0;
      sumCount1 = 0;
    }

    /* end of MPU9250 1 interrupt handling */
   }

   delta_send_time1 = millis() - last_send_time1;
   if (!SERIAL_DEBUG && delta_send_time1 >= 10) //100HZ
   {
     last_send_time1 = millis();
     Serial.print(String("#;"));
     Serial.print(orien.w, 5);
     Serial.print(String(';'));
     Serial.print(orien.x, 5);
     Serial.print(String(';'));
     Serial.print(orien.y, 5);
     Serial.print(String(';'));
     Serial.print(orien.z, 5);
     Serial.print('\n');
   }
}

//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================

void myinthandler1()
{
  intFlag1 = true;
  
}

void myinthandler2()
{
  intFlag2 = true;
}

