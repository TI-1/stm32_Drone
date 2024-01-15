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
#include "math.h"
#include "stdio.h"
#include "debug.h"

MPU9250::MPU9250(uint8_t intPin)
{
  _intPin = intPin;
}



MPU9250::MPU9250()
{

}


/**
 * Gets the ID of the MPU-9250.
 *
 * @return The ID of the MPU-9250.
 */
uint8_t MPU9250::getMPU9250ID()
{
  uint8_t c;
  I2Cdev_readByte(MPU9250_1_ADDRESS, WHO_AM_I_MPU9250, &c, I2CDEV_DEFAULT_READ_TIMEOUT); // Read WHO_AM_I register for MPU-9250
  return c;
}

/**
 * Calculates the Gyroscope scaling necessary to convert raw data to degrees/s.
 * @param Gscale the selected gyroscope resolution.
 * @return The scale factor.
 */
float MPU9250::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GYRO_FS_SEL_250DPS:
          _gRes = 250.0/32768.0; // @suppress("Avoid magic numbers")
          return _gRes;
          break;
    case GYRO_FS_SEL_500DPS:
          _gRes = 500.0/32768.0; // @suppress("Avoid magic numbers")
          return _gRes;
          break;
    case GYRO_FS_SEL_1000DPS:
         _gRes = 1000.0/32768.0; // @suppress("Avoid magic numbers")
         return _gRes;
         break;
    case GYRO_FS_SEL_2000DPS:
          _gRes = 2000.0/32768.0; // @suppress("Avoid magic numbers")
         return _gRes;
         break;
    default:
    	_gRes = 1000.0/32768.0; // @suppress("Avoid magic numbers")
    	return _gRes;
    	break;
  }
}

/**
 * Calculates the Accelerometer scaling necessary to convert raw accelerometer readings to G's.
 *
 * @param Ascale the selected accelerometer resolution.
 * @return The scale factor.
 */
float MPU9250::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
    case ACCEL_FS_SEL_2G:
         _aRes = 2.0f/32768.0f; // @suppress("Avoid magic numbers")
         return _aRes;
         break;
    case ACCEL_FS_SEL_4G:
         _aRes = 4.0f/32768.0f; // @suppress("Avoid magic numbers")
         return _aRes;
         break;
    case ACCEL_FS_SEL_8G:
         _aRes = 8.0f/32768.0f; // @suppress("Avoid magic numbers")
         return _aRes;
         break;
    case ACCEL_FS_SEL_16G:
         _aRes = 16.0f/32768.0f; // @suppress("Avoid magic numbers")
         return _aRes;
         break;
    default:
    	_aRes = 8.0f/32768.0f; // @suppress("Avoid magic numbers")
    	return _aRes;
    	break;

  }
}


/**
 * Resets the MPU
 */
void MPU9250::resetMPU9250()
{
  // reset device
	I2Cdev_writeByte(MPU9250_1_ADDRESS, PWR_MGMT_1, 0X80); // Set bit 7 to reset MPU9250
	HAL_Delay(100); // Wait for all registers to reset
}

/**
 * Reads the MCU raw accelerometer and gyroscope data.
 * @param destination a pointer to an array to store the raw data.
 * pointer to array to hold raw data.
 */
void MPU9250::readMPU9250Data(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  I2Cdev_readBytes(MPU9250_1_ADDRESS, ACCEL_XOUT_H, 14, rawData, I2CDEV_DEFAULT_READ_TIMEOUT);// Read the 14 raw data registers into data array
  destination[0] = rawData[0] << 8 | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = rawData[2] << 8 | rawData[3] ;
  destination[2] = rawData[4] << 8 | rawData[5] ;
  destination[3] = rawData[6] << 8 | rawData[7] ;
  destination[4] = rawData[8] << 8 | rawData[9] ;
  destination[5] = rawData[10] << 8 | rawData[11] ;
  destination[6] = rawData[12] << 8 | rawData[13] ;
}

/**
 * Reads the MCU raw accelerometer data.
 * @param destination a pointer to an array to store the raw data.
 * pointer to array to hold raw data.
 */
void MPU9250::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  I2Cdev_readBytes(MPU9250_1_ADDRESS, ACCEL_XOUT_H, 6, rawData, I2CDEV_DEFAULT_READ_TIMEOUT); // Read the six raw data registers into data array
  destination[0] = rawData[0] << 8 | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = rawData[2] << 8 | rawData[3] ;
  destination[2] = rawData[4] << 8 | rawData[5] ;
}


/**
 * Reads the MCU raw gyroscope data.
 * @param destination a pointer to an array to store the raw data.
 * pointer to array to hold raw data.
 */
void MPU9250::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  I2Cdev_readBytes(MPU9250_1_ADDRESS, GYRO_XOUT_H, 6, rawData, I2CDEV_DEFAULT_READ_TIMEOUT); // Read the six raw data registers sequentially into data array
  destination[0] = rawData[0] << 8 | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = rawData[2] << 8 | rawData[3] ;
  destination[2] = rawData[4] << 8 | rawData[5] ;
}



bool MPU9250::checkNewAccelGyroData()
{
  bool test;
  uint8_t data;
  I2Cdev_readByte(MPU9250_1_ADDRESS, INT_STATUS, &data, I2CDEV_DEFAULT_READ_TIMEOUT);
  test = (data & 0x01);
  return test;
}




int16_t MPU9250::readGyroTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  I2Cdev_readBytes(MPU9250_1_ADDRESS, TEMP_OUT_H, 2, rawData, I2CDEV_DEFAULT_READ_TIMEOUT); // Read the two raw data registers sequentially into data array
  return rawData[0] << 8 | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

int16_t MPU9250::getXAccelOffset() {
	uint8_t rawData[2];
	I2Cdev_readBytes(MPU9250_1_ADDRESS, XA_OFFSET_H, 2, rawData, I2CDEV_DEFAULT_READ_TIMEOUT);
	return rawData[0] << 8 | rawData[1];
}

int16_t MPU9250::getYAccelOffset() {
	uint8_t rawData[2];
	I2Cdev_readBytes(MPU9250_1_ADDRESS, YA_OFFSET_H, 2, rawData, I2CDEV_DEFAULT_READ_TIMEOUT);
	return rawData[0] << 8 | rawData[1];
}

int16_t MPU9250::getZAccelOffset() {
	uint8_t rawData[2];
	I2Cdev_readBytes(MPU9250_1_ADDRESS, ZA_OFFSET_H, 2, rawData, I2CDEV_DEFAULT_READ_TIMEOUT);
	return rawData[0] << 8 | rawData[1];
}

void MPU9250::setXAccelOffset(int16_t offset) {
	uint8_t upper = (offset >> 8) & 0xFF;
	uint8_t lower = offset & 0xFF;
	uint8_t ts_data[2] = {upper , lower};
	I2Cdev_writeBytes(MPU9250_1_ADDRESS, XA_OFFSET_H, 2, ts_data);
}

void MPU9250::setYAccelOffset(int16_t offset) {
	uint8_t upper = (offset >> 8) & 0xFF;
	uint8_t lower = offset & 0xFF;
	uint8_t ts_data[2] = {upper , lower};
	I2Cdev_writeBytes(MPU9250_1_ADDRESS, YA_OFFSET_H, 2, ts_data);
}

void MPU9250::setZAccelOffset(int16_t offset) {
	uint8_t upper = (offset >> 8) & 0xFF;
	uint8_t lower = offset & 0xFF;
	uint8_t ts_data[2] = {upper , lower};
	I2Cdev_writeBytes(MPU9250_1_ADDRESS, ZA_OFFSET_H, 2, ts_data);
}

int16_t MPU9250::getXGyroOffset() {
	uint8_t rawData[2];
	I2Cdev_readBytes(MPU9250_1_ADDRESS, XG_OFFSET_H, 2, rawData, I2CDEV_DEFAULT_READ_TIMEOUT);
	return rawData[0] << 8 | rawData[1];
}

int16_t MPU9250::getYGyroOffset() {
	uint8_t rawData[2];
	I2Cdev_readBytes(MPU9250_1_ADDRESS, YG_OFFSET_H, 2, rawData, I2CDEV_DEFAULT_READ_TIMEOUT);
	return rawData[0] << 8 | rawData[1];
}

int16_t MPU9250::getZGyroOffset() {
	uint8_t rawData[2];
	I2Cdev_readBytes(MPU9250_1_ADDRESS, ZG_OFFSET_H, 2, rawData, I2CDEV_DEFAULT_READ_TIMEOUT);
	return rawData[0] << 8 | rawData[1];
}

void MPU9250::setXGyroOffset(int16_t offset) {
	uint8_t upper = (offset >> 8) & 0xFF;
	uint8_t lower = offset & 0xFF;
	uint8_t ts_data[2] = {upper , lower};
	I2Cdev_writeBytes(MPU9250_1_ADDRESS, XG_OFFSET_H, 2, ts_data);
}

void MPU9250::setYGyroOffset(int16_t offset) {
	uint8_t upper = (offset >> 8) & 0xFF;
	uint8_t lower = offset & 0xFF;
	uint8_t ts_data[2] = {upper , lower};
	I2Cdev_writeBytes(MPU9250_1_ADDRESS, YG_OFFSET_H, 2, ts_data);
}

void MPU9250::setZGyroOffset(int16_t offset) {
	uint8_t upper = (offset >> 8) & 0xFF;
	uint8_t lower = offset & 0xFF;
	uint8_t ts_data[2] = {upper , lower};
	I2Cdev_writeBytes(MPU9250_1_ADDRESS, ZG_OFFSET_H, 2, ts_data);
}

/**
 * Reads raw accelerometer data and scales it based on sensitivity setting set in initMPU9250 to produce output in g's. Output is stored in destination.
 * @param destination
 */
void MPU9250::readAccelDataScaled(float *destination) {
	int16_t a_xyz[3];
	readAccelData(a_xyz);
	destination[0] =  a_xyz[0] * _aRes;
	destination[1] =  a_xyz[1] * _aRes;
	destination[2] =  a_xyz[2] * _aRes;

}

/**
 * Reads raw gyroscope data and scales it based on sensitivity setting set in initMPU9250 to produce output in degrees/s. Output is stored in destination.
 * @param destination
 */
void MPU9250::readGyroDataScaled(float *destination)
{
	int16_t g_xyz[3];
	readGyroData(g_xyz);
    destination[0] =  g_xyz[0] * _gRes;
    destination[1] =  g_xyz[1] * _gRes;
    destination[2] =  g_xyz[2] * _gRes;

}

uint8_t MPU9250::initMPU9250(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate)
{  	uint8_t c;
	 // Initialise MPU9250 device
	 // wake up device
    if(getMPU9250ID() != 0x70)
        {
          return incorrect_ID;
        }

	I2Cdev_writeByte(MPU9250_1_ADDRESS, PWR_MGMT_1, 0x00);// Clear sleep mode bit (6), enable all sensors
	HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	 // get stable time source
	  I2Cdev_writeByte(MPU9250_1_ADDRESS, PWR_MGMT_1, 0x01);// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    I2Cdev_readByte(MPU9250_1_ADDRESS,PWR_MGMT_1, &c, I2CDEV_DEFAULT_READ_TIMEOUT);
    if(c == 0x01){
      Debug(printf("Clock source set.\r\n"));
    }
    else
    {
      return clock_set_failed;
    }

    //enable accelerometer and gyroscope
    I2Cdev_writeByte(MPU9250_1_ADDRESS,PWR_MGMT_2,0x00);
    I2Cdev_readByte(MPU9250_1_ADDRESS, PWR_MGMT_2, &c, I2CDEV_DEFAULT_READ_TIMEOUT);
    if(c == 0x00){
      Debug(printf("Accelerometer and Gyroscope Enabled.\r\n"));
    } 
    else 
    {
      return accel_and_gyro_set_failed;
    }

	 // Configure Gyroscope and Accelerometer
	 // Disable FSYNC and set accelerometer and gyroscope bandwidth to 44 and 42 Hz, respectively;

// Set accelerometer full-scale range configuration
	  I2Cdev_writeByte(MPU9250_1_ADDRESS, ACCEL_CONFIG, Ascale); // Write new ACCEL_CONFIG register value
    I2Cdev_readByte(MPU9250_1_ADDRESS, ACCEL_CONFIG, &c, I2CDEV_DEFAULT_READ_TIMEOUT);
    if(c == Ascale){
      getAres(Ascale);
      Debug(printf("Accelerometer sensitivity set.\r\n"));
    } 
    else
    {
      return accel_sensitivty_set_failed;
    }

    // Set gyroscope full scale range configuration
	  I2Cdev_writeByte(MPU9250_1_ADDRESS, GYRO_CONFIG, Gscale); // Write new GYRO_CONFIG value to register
    I2Cdev_readByte(MPU9250_1_ADDRESS, GYRO_CONFIG, &c, I2CDEV_DEFAULT_READ_TIMEOUT);
    if(c == Gscale){
      getGres(Gscale);
      Debug(printf("Gyro sensitivity set.\r\n"));
    } 
    else
    {
      return gyro_sensitivty_set_failed;
    }

 // Set accelerometer sample rate configuration
	  I2Cdev_writeByte(MPU9250_1_ADDRESS, ACCEL_CONFIG2, ACCEL_DLPF_41); // Write new ACCEL_CONFIG2 register value
    I2Cdev_readByte(MPU9250_1_ADDRESS, ACCEL_CONFIG2, &c, I2CDEV_DEFAULT_READ_TIMEOUT);
    if(c == ACCEL_DLPF_41){
      Debug(printf("Acceleration filter set.\r\n"));
    } 
    else
    {
      return accel_filter_set_failed;
    }

 // Set gyro sample rate configuration
	  I2Cdev_writeByte(MPU9250_1_ADDRESS, CONFIG, GYRO_DLPF_41); // Write new CONFIG register value
    I2Cdev_readByte(MPU9250_1_ADDRESS, CONFIG, &c, I2CDEV_DEFAULT_READ_TIMEOUT);
    if(c == GYRO_DLPF_41){
      Debug(printf("Gyro filter set.\r\n"));
    } 
    else
    {
      return gyro_filter_set_failed;
    }

	 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	  I2Cdev_writeByte(MPU9250_1_ADDRESS, SMPLRT_DIV, sampleRate);  // Use a 200Hz rate; the same rate set in CONFIG above
    I2Cdev_readByte(MPU9250_1_ADDRESS, SMPLRT_DIV, &c, I2CDEV_DEFAULT_READ_TIMEOUT);
     if (c == sampleRate)
    {
      Debug(printf("Sample Rate set.\r\n"));
    }
    else
    {
      return sample_rate_set_failed;
    }
	 // The accelerometer, gyroscope, and thermometer are set to 1 kHz sample rates,
	 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	  // Configure Interrupts and Bypass Enable
	  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	  // can join the I2C bus and all can be controlled by the Arduino as master
	  I2Cdev_writeByte(MPU9250_1_ADDRESS, INT_PIN_CFG, 0x22);
      I2Cdev_readByte(MPU9250_1_ADDRESS, INT_PIN_CFG, &c, I2CDEV_DEFAULT_READ_TIMEOUT);
     if (c == 0x22)
    {
      Debug(printf("I2C Bypass Enabled.\r\n"));
    }
    else
    {
      return I2C_bypass_failed;
    }
	  I2Cdev_writeByte(MPU9250_1_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    I2Cdev_readByte(MPU9250_1_ADDRESS, INT_ENABLE, &c, I2CDEV_DEFAULT_READ_TIMEOUT);
     if (c == 0x01)
    {
      Debug(printf("Data Ready Interrupt Enabled.\r\n"));
    }
    else
    {
      return data_interrupt_set_failed;
    }

    return ok;
}


// Function which accumulates gyroscope and accelerometer data after device initialisation. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyroscope bias registers.
void MPU9250::calibrateMPU9250(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyroscope x, y, z, data
  uint16_t ii;
  uint16_t packet_count;
  uint16_t fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0};
  int32_t accel_bias[3] = {0, 0, 0};
  
 // reset device
  I2Cdev_writeByte(MPU9250_1_ADDRESS, PWR_MGMT_1, 0x80);// Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  I2Cdev_writeByte(MPU9250_1_ADDRESS, PWR_MGMT_1, 0x01);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, PWR_MGMT_2, 0x00);
  HAL_Delay(200);

// Configure device for bias calculation
  I2Cdev_writeByte(MPU9250_1_ADDRESS, INT_ENABLE, 0x00);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, FIFO_EN, 0x00);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, PWR_MGMT_1, 0x00);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, I2C_MST_CTRL, 0x00);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, USER_CTRL, 0x00);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, USER_CTRL, 0x0C);

  HAL_Delay(15);
  
// Configure MPU6050 gyroscope and accelerometer for bias calculation
  I2Cdev_writeByte(MPU9250_1_ADDRESS, CONFIG, 0x01); // Set low-pass filter to 188 Hz
  I2Cdev_writeByte(MPU9250_1_ADDRESS, SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
  I2Cdev_writeByte(MPU9250_1_ADDRESS, GYRO_CONFIG, 0x00);// Set gyroscope full-scale to 250 degrees per second, maximum sensitivity
  I2Cdev_writeByte(MPU9250_1_ADDRESS, ACCEL_CONFIG, 0x00);// Set accelerometer full-scale to 2 g, maximum sensitivity


  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyroscope data for bias calculation
  I2Cdev_writeByte(MPU9250_1_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  I2Cdev_writeByte(MPU9250_1_ADDRESS, FIFO_EN, 0x78);     // Enable gyroscope and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  HAL_Delay(40);// accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  I2Cdev_writeByte(MPU9250_1_ADDRESS, FIFO_EN, 0x00);        // Disable gyroscope and accelerometer sensors for FIFO
  I2Cdev_readBytes(MPU9250_1_ADDRESS, FIFO_COUNTH, 2, data, I2CDEV_DEFAULT_READ_TIMEOUT); // read FIFO sample count
  fifo_count = data[0] << 8 | data[1];
  packet_count = fifo_count/12;// How many sets of full gyroscope and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0};
    int16_t gyro_temp[3] = {0, 0, 0};
    I2Cdev_readBytes(MPU9250_1_ADDRESS, FIFO_R_W, 12, data, I2CDEV_DEFAULT_READ_TIMEOUT);// read data for averaging
    accel_temp[0] = data[0] << 8 | data[1];  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = data[2] << 8 | data[3];
    accel_temp[2] = data[4] << 8 | data[5];
    gyro_temp[0]  = data[6] << 8 | data[7];
    gyro_temp[1]  = data[8] << 8 | data[9];
    gyro_temp[2]  = data[10] << 8 | data[1];

    accel_bias[0] += accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += accel_temp[1];
    accel_bias[2] += accel_temp[2];
    gyro_bias[0]  += gyro_temp[0];
    gyro_bias[1]  += gyro_temp[1];
    gyro_bias[2]  += gyro_temp[2];

}
    accel_bias[0] /= packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= packet_count;
    accel_bias[2] /= packet_count;
    gyro_bias[0]  /= packet_count;
    gyro_bias[1]  /= packet_count;
    gyro_bias[2]  /= packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -=  accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  I2Cdev_writeByte(MPU9250_1_ADDRESS, XG_OFFSET_H, data[0]);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, XG_OFFSET_L, data[1]);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, YG_OFFSET_H, data[2]);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, YG_OFFSET_L, data[3]);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, ZG_OFFSET_H, data[4]);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  dest1[0] =  gyro_bias[0] / gyrosensitivity;
  dest1[1] =  gyro_bias[1] / gyrosensitivity;
  dest1[2] =  gyro_bias[2] / gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  I2Cdev_readBytes(MPU9250_1_ADDRESS, XA_OFFSET_H, 2, data, I2CDEV_DEFAULT_READ_TIMEOUT);// Read factory accelerometer trim values
  accel_bias_reg[0] = data[0] << 8 | data[1];
  I2Cdev_readBytes(MPU9250_1_ADDRESS, YA_OFFSET_H, 2, data, I2CDEV_DEFAULT_READ_TIMEOUT);
  accel_bias_reg[1] = data[0] << 8 | data[1];
  I2Cdev_readBytes(MPU9250_1_ADDRESS, ZA_OFFSET_H, 2, data, I2CDEV_DEFAULT_READ_TIMEOUT);
  accel_bias_reg[2] = data[0] << 8 | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
 I2Cdev_writeByte(MPU9250_1_ADDRESS, XA_OFFSET_H, data[0]);
 I2Cdev_writeByte(MPU9250_1_ADDRESS, XA_OFFSET_L, data[1]);
 I2Cdev_writeByte(MPU9250_1_ADDRESS, YA_OFFSET_H, data[2]);
 I2Cdev_writeByte(MPU9250_1_ADDRESS, YA_OFFSET_L, data[3]);
 I2Cdev_writeByte(MPU9250_1_ADDRESS, ZA_OFFSET_H, data[4]);
 I2Cdev_writeByte(MPU9250_1_ADDRESS, ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
   dest2[0] = accel_bias[0] / accelsensitivity;
   dest2[1] = accel_bias[1] / accelsensitivity;
   dest2[2] = accel_bias[2] / accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0};
   int32_t aAvg[3] = {0};
   int32_t aSTAvg[3] = {0};
   int32_t gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;


  I2Cdev_writeByte(MPU9250_1_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  I2Cdev_writeByte(MPU9250_1_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  I2Cdev_writeByte(MPU9250_1_ADDRESS, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  I2Cdev_writeByte(MPU9250_1_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  I2Cdev_writeByte(MPU9250_1_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
	  I2Cdev_readBytes(MPU9250_1_ADDRESS, ACCEL_XOUT_H, 6, rawData, I2CDEV_DEFAULT_READ_TIMEOUT); // Read the six raw data registers into data array
  aAvg[0] += rawData[0] << 8 | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += rawData[2] << 8 | rawData[3] ;
  aAvg[2] += rawData[4] << 8 | rawData[5] ;

  	I2Cdev_readBytes(MPU9250_1_ADDRESS, GYRO_XOUT_H, 6, rawData, I2CDEV_DEFAULT_READ_TIMEOUT); // Read the six raw data registers sequentially into data array
  gAvg[0] += rawData[0] << 8 | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += rawData[2] << 8 | rawData[3];
  gAvg[2] += rawData[4] << 8 | rawData[5];
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
  I2Cdev_writeByte(MPU9250_1_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  I2Cdev_writeByte(MPU9250_1_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  HAL_Delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  I2Cdev_readBytes(MPU9250_1_ADDRESS, ACCEL_XOUT_H, 6, rawData, I2CDEV_DEFAULT_READ_TIMEOUT);// Read the six raw data registers into data array
  aSTAvg[0] += rawData[0] << 8| rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += rawData[2] << 8| rawData[3] ;
  aSTAvg[2] += rawData[4] << 8| rawData[5] ;
  
  I2Cdev_readBytes(MPU9250_1_ADDRESS, GYRO_XOUT_H, 6, rawData, I2CDEV_DEFAULT_READ_TIMEOUT);// Read the six raw data registers sequentially into data array
  gSTAvg[0] += rawData[0] << 8 | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += rawData[2] << 8 | rawData[3] ;
  gSTAvg[2] += rawData[4] << 8 | rawData[5] ;
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
  I2Cdev_writeByte(MPU9250_1_ADDRESS, ACCEL_CONFIG, 0x00);
  I2Cdev_writeByte(MPU9250_1_ADDRESS, GYRO_CONFIG,  0x00);
  HAL_Delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   I2Cdev_readByte(MPU9250_1_ADDRESS, SELF_TEST_X_ACCEL, &selfTest[0], I2CDEV_DEFAULT_READ_TIMEOUT);// X-axis accel self-test results
   I2Cdev_readByte(MPU9250_1_ADDRESS, SELF_TEST_Y_ACCEL, &selfTest[1], I2CDEV_DEFAULT_READ_TIMEOUT); // Y-axis accel self-test results
   I2Cdev_readByte(MPU9250_1_ADDRESS, SELF_TEST_Z_ACCEL, &selfTest[2], I2CDEV_DEFAULT_READ_TIMEOUT); // Z-axis accel self-test results
   I2Cdev_readByte(MPU9250_1_ADDRESS, SELF_TEST_X_GYRO, &selfTest[3], I2CDEV_DEFAULT_READ_TIMEOUT); // X-axis gyro self-test results
   I2Cdev_readByte(MPU9250_1_ADDRESS, SELF_TEST_Y_GYRO, &selfTest[4], I2CDEV_DEFAULT_READ_TIMEOUT); // Y-axis gyro self-test results
   I2Cdev_readByte(MPU9250_1_ADDRESS, SELF_TEST_Z_GYRO, &selfTest[5], I2CDEV_DEFAULT_READ_TIMEOUT);// Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (2620/1<<FS)*(pow( 1.01 , (selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (2620/1<<FS)*(pow( 1.01 , (selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (2620/1<<FS)*(pow( 1.01 , (selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (2620/1<<FS)*(pow( 1.01 , (selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (2620/1<<FS)*(pow( 1.01 , (selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (2620/1<<FS)*(pow( 1.01 , (selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0f*((aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0f;   // Report percent differences
     destination[i+3] = 100.0f*((gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0f; // Report percent differences
   }
   

}



