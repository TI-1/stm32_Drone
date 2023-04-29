/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. 
 Addition of 9 DoF sensor fusion using open source Madgwick filter algorithm. 
 Sketch runs on the 3.3 V Ladybug STM32L432 Breakout Board.
 
 Library may be used freely and without limit with attribution.
 
*/
  
#ifndef MPU9250_h
#define MPU9250_h

 #include <stdint.h>
#include "I2Cdev.h"

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//


#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

#define ACCEL_FS_SEL_2G  0x00
#define ACCEL_FS_SEL_4G  0x08
#define ACCEL_FS_SEL_8G  0x10
#define ACCEL_FS_SEL_16G 0x18
#define GYRO_CONFIG 	 0x1B
#define GYRO_FS_SEL_250DPS  0x00
#define GYRO_FS_SEL_500DPS  0x08
#define GYRO_FS_SEL_1000DPS 0x10
#define GYRO_FS_SEL_2000DPS 0x18
#define ACCEL_CONFIG2  0x1D
#define ACCEL_DLPF_184 0x01
#define ACCEL_DLPF_92  0x02
#define ACCEL_DLPF_41  0x03
#define ACCEL_DLPF_20  0x04
#define ACCEL_DLPF_10  0x05
#define ACCEL_DLPF_5   0x06
#define CONFIG 		   0x1A
#define GYRO_DLPF_184  0x01
#define GYRO_DLPF_92   0x02
#define GYRO_DLPF_41   0x03
#define GYRO_DLPF_20   0x04
#define GYRO_DLPF_10   0x05
#define GYRO_DLPF_5    0x06

// Set initial input parameters
#define  AFS_2G  0
#define  AFS_4G  1
#define  AFS_8G  2
#define  AFS_16G 3

#define  GFS_250DPS  0
#define  GFS_500DPS  1
#define  GFS_1000DPS 2
#define  GFS_2000DPS 3

#define  MFS_14BITS  0 // 0.6 mG per LSB
#define  MFS_16BITS  1    // 0.15 mG per LSB

#define M_8Hz   0x02
#define M_100Hz 0x06

// Define I2C addresses of the two MPU9250
#define MPU9250_1_ADDRESS 0x68   // Device address when ADO = 0
#define MPU1              0x68
#define MPU9250_2_ADDRESS 0x69   // Device address when ADO = 1
#define MPU2              0x69
#define AK8963_ADDRESS    0x0C   //  Address of magnetometer

// Define Calibrated offset values for MPU
#define USER_XA_OFFSET -4156
#define USER_YA_OFFSET 4680
#define USER_ZA_OFFSET 8869
#define USER_XG_OFFSET 114
#define USER_YG_OFFSET -178
#define USER_ZG_OFFSET -46


class MPU9250
{
  public: 
	  MPU9250(uint8_t intPin);
	  MPU9250();
	  uint8_t getMPU9250ID();
	  void resetMPU9250();
	  int8_t initMPU9250 (uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate);
	  float getAres(uint8_t Ascale);
	  float getGres(uint8_t Gscale);
	  void calibrateMPU9250(float * dest1, float * dest2);
	  void SelfTest(float * destination);
	  void readMPU9250Data(int16_t * destination);
	  void readAccelData(int16_t * destination);
	  void readGyroData(int16_t * destination);
	  bool checkNewAccelGyroData();
	  bool checkNewMagData();
	  int16_t readGyroTempData();
	  void gyromagSleep();
	  void gyromagWake(uint8_t Mmode);
	  void accelWakeOnMotion();
	  bool checkWakeOnMotion();
	  int16_t getXAccelOffset();
	  int16_t getYAccelOffset();
	  int16_t getZAccelOffset();
	  void setXAccelOffset(int16_t offset);
	  void setYAccelOffset(int16_t offset);
	  void setZAccelOffset(int16_t offset);
	  int16_t getXGyroOffset();
	  int16_t getYGyroOffset();
	  int16_t getZGyroOffset();
	  void setXGyroOffset(int16_t offset);
	  void setYGyroOffset(int16_t offset);
	  void setZGyroOffset(int16_t offset);


	  void readGyroDataScaled(float * destination);
	  void readAccelDataScaled(float * destination);
	  

  private:
	  uint8_t _intPin = 0;
	  /** The accelerometer scale factor*/
	  float _aRes = 0;
	  /** The gyroscope scale factor*/
	  float _gRes = 0;
	  uint8_t _Mmode = 0;
};

#endif
