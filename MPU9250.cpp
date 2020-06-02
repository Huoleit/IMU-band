/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. 
 Addition of 9 DoF sensor fusion using open source Madgwick filter algorithm. 
 Sketch runs on the 3.3 V Dragonfly STM32L476 Breakout Board.
 
 Library may be used freely and without limit with attribution.
 
*/

#include "MPU9250.h"
#include "Wire.h"

MPU9250::MPU9250(uint8_t intPin)
{
  uint8_t _intPin = intPin;
}


uint8_t MPU9250::getMPU9250ID(uint8_t MPUnum)
{
  uint8_t c = readByte(MPUnum, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  return c;
}

  uint8_t MPU9250::getAK8963CID(uint8_t MPUnum)
{
//  uint8_t c = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for MPU-9250
  writeByte(MPUnum, USER_CTRL, 0x20);    // Enable I2C Master mode  
  writeByte(MPUnum, I2C_MST_CTRL, 0x0D); // I2C configuration multi-master I2C 400KHz

  writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  writeByte(MPUnum, I2C_SLV0_REG, WHO_AM_I_AK8963);           // I2C slave 0 register address from where to begin data transfer
  writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
  delay(10);
  uint8_t c = readByte(MPUnum, EXT_SENS_DATA_00);             // Read the WHO_AM_I byte
  return c;
}


float MPU9250::getMres(uint8_t Mscale) {
  switch (Mscale)
  {
   // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          _mRes = 10.0f*4912.0f/8190.0f; // Proper scale to return milliGauss
          return _mRes;
          break;
    case MFS_16BITS:
          _mRes = 10.0f*4912.0f/32760.0f; // Proper scale to return milliGauss
          return _mRes;
          break;
  }
}

float MPU9250::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_250DPS:
          _gRes = 250.0/32768.0;
          return _gRes;
          break;
    case GFS_500DPS:
          _gRes = 500.0/32768.0;
          return _gRes;
          break;
    case GFS_1000DPS:
         _gRes = 1000.0/32768.0;
         return _gRes;
         break;
    case GFS_2000DPS:
          _gRes = 2000.0/32768.0;
         return _gRes;
         break;
  }
}

float MPU9250::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
         _aRes = 2.0f/32768.0f;
         return _aRes;
         break;
    case AFS_4G:
         _aRes = 4.0f/32768.0f;
         return _aRes;
         break;
    case AFS_8G:
         _aRes = 8.0f/32768.0f;
         return _aRes;
         break;
    case AFS_16G:
         _aRes = 16.0f/32768.0f;
         return _aRes;
         break;
  }
}



void MPU9250::accelWakeOnMotion(uint8_t MPUnum)
{
  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  uint8_t c = readByte(MPUnum, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x01;  // Set accelerometer rate to 1 kHz and bandwidth to 184 Hz
  writeByte(MPUnum, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master 
   writeByte(MPUnum, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear  
   writeByte(MPUnum, INT_ENABLE, 0x41);   // Enable data ready (bit 0) and wake on motion (bit 6)  interrupt

   // enable wake on motion detection logic (bit 7) and compare current sample to previous sample (bit 6)
   writeByte(MPUnum, MOT_DETECT_CTRL, 0xC0);  

   // set accel threshold for wake up at  mG per LSB, 1 - 255 LSBs == 0 - 1020 mg), pic 0x19 for 25 mg
   writeByte(MPUnum, WOM_THR, 0x19);

  // set sample rate in low power mode
  /* choices are 0 == 0.24 Hz, 1 == 0.49 Hz, 2 == 0.98 Hz, 3 == 1.958 Hz, 4 == 3.91 Hz, 5 == 7.81 Hz
   *             6 == 15.63 Hz, 7 == 31.25 Hz, 8 == 62.50 Hz, 9 = 125 Hz, 10 == 250 Hz, and 11 == 500 Hz
   */
  writeByte(MPUnum, LP_ACCEL_ODR, 0x02);

  c = readByte(MPUnum, PWR_MGMT_1);
  writeByte(MPUnum, PWR_MGMT_1, c | 0x20);     // Write bit 5 to enable accel cycling

  gyromagSleep(MPUnum);
  delay(100); // Wait for all registers to reset 

}


void MPU9250::gyromagSleep(uint8_t MPUnum)
{
  uint8_t temp = 0;
  writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);        // Set the I2C slave address of AK8963 and set for read.
  writeByte(MPUnum, I2C_SLV0_REG,  AK8963_CNTL);                  // I2C slave 0 register address from where to begin data transfer
  writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                         // Enable I2C and transfer 1 byte
  delay(10);
  temp = readByte(MPUnum, EXT_SENS_DATA_00);
  
  writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS);               // Set the I2C slave address of AK8963 and set for write.
  writeByte(MPUnum, I2C_SLV0_REG,  AK8963_CNTL);                  // I2C slave 0 register address from where to begin data transfer
  writeByte(MPUnum, I2C_SLV0_DO, temp & ~(0x0F));                 // Power down AK8963
  writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                         // Enable I2C and transfer 1 byte
  delay(10);
  
  temp = readByte(MPUnum, PWR_MGMT_1);
  writeByte(MPUnum, PWR_MGMT_1, temp | 0x10);     // Write bit 4 to enable gyro standby
  delay(10); // Wait for all registers to reset 
}


void MPU9250::gyromagWake(uint8_t MPUnum, uint8_t Mmode)
{
  uint8_t temp = 0;
  writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);        // Set the I2C slave address of AK8963 and set for read.
  writeByte(MPUnum, I2C_SLV0_REG,  AK8963_CNTL);                  // I2C slave 0 register address from where to begin data transfer
  writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                         // Enable I2C and transfer 1 byte
  delay(10);
  temp = readByte(MPUnum, EXT_SENS_DATA_00);
  
  writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS);               // Set the I2C slave address of AK8963 and set for write.
  writeByte(MPUnum, I2C_SLV0_REG,  AK8963_CNTL);                  // I2C slave 0 register address from where to begin data transfer
  writeByte(MPUnum, I2C_SLV0_DO, temp | Mmode);                   // Reset normal mode for  magnetometer 
  writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                         // Enable I2C and transfer 1 byte
  delay(10);
  temp = readByte(MPUnum, PWR_MGMT_1);
  writeByte(MPUnum, PWR_MGMT_1, 0x01);                            // return gyro and accel normal mode
  delay(10); // Wait for all registers to reset 
}


void MPU9250::resetMPU9250(uint8_t MPUnum)
{
  // reset device
  writeByte(MPUnum, PWR_MGMT_1, 0x80); // Set bit 7 to reset MPU9250
  delay(100); // Wait for all registers to reset 
}

void MPU9250::readMPU9250Data(uint8_t MPUnum, int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(MPUnum, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;   
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;  
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;  
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 
}

void MPU9250::readAccelData(uint8_t MPUnum, int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPUnum, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void MPU9250::readGyroData(uint8_t MPUnum, int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPUnum, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

bool MPU9250::checkNewMagData(uint8_t MPUnum)
{
  bool test;
//  test = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
  writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);     // Set the I2C slave address of AK8963 and set for read.
  writeByte(MPUnum, I2C_SLV0_REG, AK8963_ST1);                 // I2C slave 0 register address from where to begin data transfer
  writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
  delay(2);
  test = (readByte(MPUnum, EXT_SENS_DATA_00) & 0x01); // Check data ready status byte
  return test;
}

bool MPU9250::checkNewAccelGyroData(uint8_t MPUnum)
{
  bool test;
  test = (readByte(MPUnum, INT_STATUS) & 0x01);
  return test;
}

bool MPU9250::checkWakeOnMotion(uint8_t MPUnum)
{
  bool test;
  test = (readByte(MPUnum, INT_STATUS) & 0x40);
  return test;
}


void MPU9250::readMagData(uint8_t MPUnum, int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
//  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
   writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV0_REG, AK8963_XOUT_L);             // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV0_CTRL, 0x87);                     // Enable I2C and read 7 bytes
//   delay(10);
   readBytes(MPUnum, EXT_SENS_DATA_00, 7, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
   uint8_t c = rawData[6]; // End data read by reading ST2 register
   if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
   destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
   destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
   destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
   }
}

int16_t MPU9250::readGyroTempData(uint8_t MPUnum)
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPUnum, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}
       

void MPU9250::initAK8963Slave(uint8_t MPUnum, uint8_t Mscale, uint8_t Mmode, float * magCalibration)
{
   // First extract the factory calibration for each magnetometer axis
   uint8_t rawData[3];  // x/y/z gyro calibration data stored here
   _Mmode = Mmode;

   writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL2);              // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV0_DO, 0x01);                       // Reset AK8963
   writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
   delay(50);
   writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV0_DO, 0x00);                       // Power down magnetometer  
   writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
   delay(50);
   writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV0_DO, 0x0F);                       // Enter fuze mode
   writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
   delay(50);
   
   writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(MPUnum, I2C_SLV0_REG, AK8963_ASAX);               // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV0_CTRL, 0x83);                     // Enable I2C and read 3 bytes
   delay(50);
   readBytes(MPUnum, EXT_SENS_DATA_00, 3, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
   magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;        // Return x-axis sensitivity adjustment values, etc.
   magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
   magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
   
   writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(MPUnum, I2C_SLV0_DO, 0x00);                       // Power down magnetometer  
   writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
   delay(50);

   writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(MPUnum, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer 
   // Configure the magnetometer for continuous read and highest resolution
   // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
   // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
   writeByte(MPUnum, I2C_SLV0_DO, Mscale << 4 | Mmode);        // Set magnetometer data resolution and sample ODR
   writeByte(MPUnum, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
   delay(50);
}


void MPU9250::initMPU9250(uint8_t MPUnum, uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate)
{  
 // wake up device
  writeByte(MPUnum, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPUnum, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPUnum, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPUnum, SMPLRT_DIV, sampleRate);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                                       // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPUnum, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x02; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPUnum, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  c = readByte(MPUnum, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  writeByte(MPUnum, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPUnum, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPUnum, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPUnum, INT_PIN_CFG, 0x10);  // INT is 50 microsecond pulse and any read to clear  
   writeByte(MPUnum, INT_ENABLE, 0x01);   // Enable data ready (bit 0) interrupt
   delay(100);

  writeByte(MPUnum, USER_CTRL, 0x20);          // Enable I2C Master mode  
  writeByte(MPUnum, I2C_MST_CTRL, 0x1D);       // I2C configuration STOP after each transaction, master I2C bus at 400 KHz
  writeByte(MPUnum, I2C_MST_DELAY_CTRL, 0x81); // Use blocking data retreival and enable delay for mag sample rate mismatch
  writeByte(MPUnum, I2C_SLV4_CTRL, 0x01);      // Delay mag data retrieval to once every other accel/gyro data sample
}


void MPU9250::magcalMPU9250(uint8_t MPUnum, float * dest1, float * dest2) 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  uint8_t rawData[7] = {0, 0, 0, 0, 0, 0, 0}, magCalibration[3] = {0, 0, 0};

  Serial.print("Mag Calibration for: 0x"); Serial.println(MPUnum, HEX); Serial.println("Wave device in a figure eight until done!");
  delay(4000);
  
// shoot for ~fifteen seconds of mag data
  if(_Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
  if(_Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
  for(ii = 0; ii < sample_count; ii++) {
    writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
    writeByte(MPUnum, I2C_SLV0_REG, AK8963_XOUT_L);             // I2C slave 0 register address from where to begin data transfer
    writeByte(MPUnum, I2C_SLV0_CTRL, 0x87);                     // Enable I2C and read 7 bytes
    if(_Mmode == 0x02) delay(125);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(_Mmode == 0x06) delay(10);   // at 100 Hz ODR, new mag data is available every 10 ms
    readBytes(MPUnum, EXT_SENS_DATA_00, 7, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
    mag_temp[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;     // Turn the MSB and LSB into a signed 16-bit value
    mag_temp[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;     // Data stored as little Endian
    mag_temp[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
    
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    Serial.println(ii);
 }

    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    writeByte(MPUnum, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
    writeByte(MPUnum, I2C_SLV0_REG, AK8963_ASAX);               // I2C slave 0 register address from where to begin data transfer
    writeByte(MPUnum, I2C_SLV0_CTRL, 0x83);                     // Enable I2C and read 3 bytes
    delay(50);
    readBytes(MPUnum, EXT_SENS_DATA_00, 3, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
    magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;        // Return x-axis sensitivity adjustment values, etc.
    magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
    magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 

    dest1[0] = (float) mag_bias[0]*_mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*_mRes*magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*_mRes*magCalibration[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("Mag Calibration done!");
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9250::calibrateMPU9250(uint8_t MPUnum, int32_t * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // reset device
  writeByte(MPUnum, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPUnum, PWR_MGMT_1, 0x01);  
  writeByte(MPUnum, PWR_MGMT_2, 0x00);
  delay(200);                                    

// Configure device for bias calculation
  writeByte(MPUnum, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPUnum, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPUnum, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPUnum, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPUnum, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPUnum, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPUnum, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPUnum, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPUnum, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPUnum, ACCEL_CONFIG, 0x08); // Set accelerometer full-scale to 4 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 8192;  // = 8192 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPUnum, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPUnum, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPUnum, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPUnum, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPUnum, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
  // Output scaled gyro biases for display in the main program
  if (dest1)
  {
    dest1[0] = gyro_bias[0];
    dest1[1] = gyro_bias[1];
    dest1[2] = gyro_bias[2];
  }
  // Output scaled accelerometer biases for display in the main program
  if (dest2)
  {
    dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
    dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
    dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
  }
}
void MPU9250::GyroBiasWriteToReg(uint8_t MPUnum, int32_t *gyro_bias)
{
  uint8_t data[12];
  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4) & 0xFF;
  data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4) & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPUnum, XG_OFFSET_H, data[0]);
  writeByte(MPUnum, XG_OFFSET_L, data[1]);
  writeByte(MPUnum, YG_OFFSET_H, data[2]);
  writeByte(MPUnum, YG_OFFSET_L, data[3]);
  writeByte(MPUnum, ZG_OFFSET_H, data[4]);
  writeByte(MPUnum, ZG_OFFSET_L, data[5]);
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::SelfTest(uint8_t MPUnum, float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;
   
  writeByte(MPUnum, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPUnum, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPUnum, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  writeByte(MPUnum, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPUnum, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
  readBytes(MPUnum, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPUnum, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   writeByte(MPUnum, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(MPUnum, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  readBytes(MPUnum, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPUnum, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
   writeByte(MPUnum, ACCEL_CONFIG, 0x00);  
   writeByte(MPUnum, GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(MPUnum, SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(MPUnum, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(MPUnum, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(MPUnum, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(MPUnum, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(MPUnum, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0f*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0f;   // Report percent differences
     destination[i+3] = 100.0f*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0f; // Report percent differences
   }
   
}


// simple function to scan for I2C devices on the bus
void MPU9250::I2Cscan() 
{
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
      

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


// I2C read/write functions for the MPU9250 sensors

  void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

  uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data = 0;                        // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t)1);            // Read two bytes from slave register address on MPU9250 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

  void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
