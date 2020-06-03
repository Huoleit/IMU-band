#include "Wire.h"   
#include "MPU9250.h"
#include "Quaternion.hpp"

#define CALIBRATION_GYRO 0
#define CALIBRATION_MAG 0
#define SERIAL_DEBUG 0

const uint8_t Gscale = GFS_250DPS, Ascale = AFS_4G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;         
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors


const float pi = 3.141592653589793238462643383279502884f;
const float rad2deg = 180.0f / pi;
const float deg2rad = pi / 180.0f;

// Pin definitions
int  intPin1 = 2;  //  MPU9250 1 interrupt
int  intPin2 = 3;  //  MPU9250 2 interrupt
int  myLed  = 13; // red led

volatile bool intFlag1 = false;
volatile bool intFlag2 = false;

int16_t MPU9250Data1[7], MPU9250Data2[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount1[3], magCount2[3];    // Stores the 16-bit signed magnetometer sensor output
float   magCalibration1[3] = {0, 0, 0}, magCalibration2[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float   temperature1, temperature2;    // Stores the MPU9250 internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// These can be measured once and entered here or can be calculated each time the device is powered on
int32_t   gyroBias1[3] = {414, -265, 20};
float accelBias1[3] = {-1.66613769f, -2.16113281f, -0.33935546f};

int32_t   gyroBias2[3] = {388, 160, -2};
float accelBias2[3] = {0.0f, 0.0f, 0.0f};

float   magBias1[3] = {415.33, -209.91, -229.41}, magScale1[3]  = {1.20, 0.91, 0.93}; // Bias corrections for gyro and accelerometer
float   magBias2[3] = {71.04, 122.43, -36.90}, magScale2[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer


uint32_t delt_t1, delt_t2 = 0;                      // used to control display output rate
uint32_t count1 = 0, sumCount1 = 0, count2 = 0, sumCount2 = 0;         // used to control display output rate
float pitch1, yaw1, roll1, pitch2, yaw2, roll2;                   // absolute orientation

float deltat1 = 0.0f, sum1 = 0.0f, deltat2 = 0.0f, sum2 = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate1 = 0, lastUpdate2 = 0; // used to calculate integration interval
uint32_t Now1 = 0, Now2 = 0;                         // used to calculate integration interval

float ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1; // variables to hold latest sensor data values 
float ax2, ay2, az2, gx2, gy2, gz2, mx2, my2, mz2; // variables to hold latest sensor data values 
Quaternion orien_1(1.0f,0.,0.,0.);
Quaternion orien_2(1.0f,0.,0.,0.);

MPU9250 MPU9250(intPin1); // instantiate MPU9250 class
uint32_t last_send_time = 0; 
uint32_t delta_send_time = 0; 
bool isOnline_1 = false;
bool isOnline_2 = false;

void setup()
{
  Serial.begin(115200);
  delay(200);
  
  Wire.begin(); 
  Wire.setClock(400000); 
  delay(200);

  // MPU9250.I2Cscan();
  
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); 

  pinMode(intPin1, INPUT); // interrupt
  pinMode(intPin2, INPUT);

  Serial.println("MPU9250 9-axis motion sensor...");
  uint8_t c = MPU9250.getMPU9250ID(MPU1);
  uint8_t d = MPU9250.getMPU9250ID(MPU2);
  if (c == 0x71)
  {
    Serial.println("MPU 1 is online");
    isOnline_1 = true;
  }
  if (d == 0x71)
  {
    Serial.println("MPU 2 is online");
    isOnline_2 = true;
  }
  if(!isOnline_1 && !isOnline_2)
  {
    Serial.print("Could not connect to MPU9250"); Serial.println(c, HEX);
    while(1) ; 
  }
  // get sensor resolutions, only need to do this once, same for both MPU9250s for now
  aRes = MPU9250.getAres(Ascale);
  gRes = MPU9250.getGres(Gscale);
  mRes = MPU9250.getMres(Mscale);

#if CALIBRATION_GYRO
    MPU9250.calibrateMPU9250(MPU1, gyroBias1, accelBias1); // Calibrate gyro and accelerometers, load biases in bias registers
    Serial.println("MPU accel biases (g)"); Serial.println(accelBias1[0],10); Serial.println(accelBias1[1],10); Serial.println(accelBias1[2],10);
    Serial.println("MPU gyro biases"); Serial.println(gyroBias1[0]); Serial.println(gyroBias1[1]); Serial.println(gyroBias1[2]);
    Serial.println("Finished!! Please reset");
    while(1);
#endif
//update gyro bias reg
  if(isOnline_1)
  {
    MPU9250.GyroBiasWriteToReg(MPU1, gyroBias1);
    delay(200);
    MPU9250.initMPU9250(MPU1, Ascale, Gscale, sampleRate); 
    byte e = MPU9250.getAK8963CID(MPU1);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 1 "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    MPU9250.initAK8963Slave(MPU1, Mscale, Mmode, magCalibration1); Serial.println("AK8963 1 initialized for active data mode...."); // Initialize device 1 for active mode read of magnetometer
  }
  if(isOnline_2)
  {
    MPU9250.GyroBiasWriteToReg(MPU2, gyroBias2);
    delay(200);
    MPU9250.initMPU9250(MPU2, Ascale, Gscale, sampleRate); 
    byte e = MPU9250.getAK8963CID(MPU2);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 2 "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    MPU9250.initAK8963Slave(MPU2, Mscale, Mmode, magCalibration1); Serial.println("AK8963 2 initialized for active data mode...."); // Initialize device 2 for active mode read of magnetometer
  }
    
#if CALIBRATION_MAG
  MPU9250.magcalMPU9250(MPU1, magBias1, magScale1);
  Serial.println("AK8963 1 mag biases (mG)"); Serial.println(magBias1[0]); Serial.println(magBias1[1]); Serial.println(magBias1[2]); 
  Serial.println("AK8963 1 mag scale (mG)"); Serial.println(magScale1[0]); Serial.println(magScale1[1]); Serial.println(magScale1[2]); 
  while(1);
#endif
  
  attachInterrupt(digitalPinToInterrupt(intPin1), myinthandler1, RISING);  // define interrupt for intPin output of MPU9250 1
  attachInterrupt(digitalPinToInterrupt(intPin2), myinthandler2, RISING);  // define interrupt for intPin output of MPU9250 2

  digitalWrite(myLed, LOW); // turn off led when using flash memory
}

void loop()
{
   if(intFlag1 == true && isOnline_1) {   
      intFlag1 = false; 

    MPU9250.readMPU9250Data(MPU1, MPU9250Data1); // INT cleared on any read
   
    ax1 = (float)MPU9250Data1[0]*aRes - accelBias1[0];  
    ay1 = (float)MPU9250Data1[1]*aRes - accelBias1[1];   
    az1 = (float)MPU9250Data1[2]*aRes - accelBias1[2];  
    gx1 = (float)MPU9250Data1[4]*gRes*deg2rad; 
    gy1 = (float)MPU9250Data1[5]*gRes*deg2rad;  
    gz1 = (float)MPU9250Data1[6]*gRes*deg2rad; 
    
    // MPU9250.readMagData(MPU1, magCount1); 
    // mx1 = (float)magCount1[0]*mRes*magCalibration1[0] - magBias1[0];  
    // my1 = (float)magCount1[1]*mRes*magCalibration1[1] - magBias1[1];  
    // mz1 = (float)magCount1[2]*mRes*magCalibration1[2] - magBias1[2];  
    // mx1 *= magScale1[0];
    // my1 *= magScale1[1];
    // mz1 *= magScale1[2]; 
   
    Now1 = micros();
    deltat1 = ((Now1 - lastUpdate1)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate1 = Now1;

    Quaternion omega(0, gx1,gy1,gz1);
    Quaternion delta_q = orien_1 * omega * 0.5f;
    orien_1 = orien_1 + (delta_q * deltat1);
    orien_1.normalize();
   
    sum1 += deltat1; // sum for averaging filter update rate
    sumCount1++;
   
    /* end of MPU9250 1 interrupt handling */
   }

   if(intFlag2 == true && isOnline_2) {   
      intFlag2 = false; 
    // Serial.print("M");
    delayMicroseconds(10);
    MPU9250.readMPU9250Data(MPU2, MPU9250Data2); // INT cleared on any read
    //Serial.print("read");
   
    ax2 = (float)MPU9250Data2[0]*aRes - accelBias2[0];  
    ay2 = (float)MPU9250Data2[1]*aRes - accelBias2[1];   
    az2 = (float)MPU9250Data2[2]*aRes - accelBias2[2];  
    gx2 = (float)MPU9250Data2[4]*gRes*deg2rad; 
    gy2 = (float)MPU9250Data2[5]*gRes*deg2rad;  
    gz2 = (float)MPU9250Data2[6]*gRes*deg2rad; 
    
    // MPU9250.readMagData(MPU2, magCount2); 
    // mx2 = (float)magCount2[0]*mRes*magCalibration2[0] - magBias2[0];  
    // my2 = (float)magCount2[1]*mRes*magCalibration2[1] - magBias2[1];  
    // mz2 = (float)magCount2[2]*mRes*magCalibration2[2] - magBias2[2];  
    // mx2 *= magScale2[0];
    // my2 *= magScale2[1];
    // mz2 *= magScale2[2]; 
   
    Now2 = micros();
    deltat2 = ((Now2 - lastUpdate2)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate2 = Now2;

    Quaternion omega(0, gx2,gy2,gz2);
    Quaternion delta_q = orien_2 * omega * 0.5f;
    orien_2 = orien_2 + (delta_q * deltat2);
    orien_2.normalize();
   
    sum2 += deltat2; // sum for averaging filter update rate
    sumCount2++;
    // Serial.println("AAAAAAA");
    /* end of MPU9250 2 interrupt handling */
   }

    if(SERIAL_DEBUG && isOnline_1 && sum1 >= 1.0f) //1HZ 
    {
      orien_1.toEulerAngle(&roll1, &pitch1,&yaw1);
      roll1 *= rad2deg;
      pitch1 *= rad2deg;
      yaw1 *= rad2deg;

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

      sum1 = 0;
      sumCount1 = 0;
    }
  
  if(SERIAL_DEBUG && isOnline_2 && sum2 >= 1.0f) //1HZ 
    {
      orien_2.toEulerAngle(&roll2, &pitch2,&yaw2);
      roll2 *= rad2deg;
      pitch2 *= rad2deg;
      yaw2 *= rad2deg;

      Serial.print("rate 2 = "); Serial.print((float)sumCount2/sum2, 2); Serial.println(" Hz");
      Serial.print("ax2 = "); Serial.print((int)1000*ax2);  
      Serial.print(" ay2 = "); Serial.print((int)1000*ay2); 
      Serial.print(" az2 = "); Serial.print((int)1000*az2); Serial.println(" mg");
      Serial.print("gx2 = "); Serial.print( gx2, 4); 
      Serial.print(" gy2 = "); Serial.print( gy2, 4); 
      Serial.print(" gz2 = "); Serial.print( gz2, 4); Serial.println(" deg/s");
      Serial.print("mx2 = "); Serial.print( (int)mx2 ); 
      Serial.print(" my2 = "); Serial.print( (int)my2 ); 
      Serial.print(" mz2 = "); Serial.print( (int)mz2 ); Serial.println(" mG");
      Serial.print("MPU9250 2 Yaw, Pitch, Roll: ");
      Serial.print(yaw2, 2);
      Serial.print(", ");
      Serial.print(pitch2, 2);
      Serial.print(", ");
      Serial.println(roll2, 2);

      sum2 = 0;
      sumCount2 = 0;
    }

   delta_send_time = millis() - last_send_time;
   if (!SERIAL_DEBUG && delta_send_time >= 10) //100HZ
   {
     last_send_time = millis();
     Serial.print(String("#;"));
     Serial.print(orien_1.w, 5);
     Serial.print(String(';'));
     Serial.print(orien_1.x, 5);
     Serial.print(String(';'));
     Serial.print(orien_1.y, 5);
     Serial.print(String(';'));
     Serial.print(orien_1.z, 5);
     Serial.print(String(';'));
     
     Serial.print(orien_2.w, 5);
     Serial.print(String(';'));
     Serial.print(orien_2.x, 5);
     Serial.print(String(';'));
     Serial.print(orien_2.y, 5);
     Serial.print(String(';'));
     Serial.print(orien_2.z, 5);
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

