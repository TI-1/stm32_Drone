/*
 * imu.h
 *
 *  Created on: Apr 19, 2023
 *      Author: tobii
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stdint.h"
#include "MPU9250.h"
#include <math.h>

enum axis {
	X, Y, Z
};

class IMU {
public:
	IMU(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate,
			I2C_HandleTypeDef *hi2c);
	IMU(const IMU &obj) = delete;
	IMU& operator=(const IMU &obj) = delete;
	float Yaw();
	float Pitch();
	float Roll();
	float Accel(axis axis);
	float Gyro(axis axis);
	void calibrateImu();
	void readIMU();
	bool dataReady();
	bool checkStatus();
	void debugGyro();
	void debugYPR();

public:
	uint8_t init_status = ok;

private:
	void initialise(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate);
	void calculateMadgwickQuaternion();

private:
	MPU9250 _mpu;
	float _ypr[3] = { 0, 0, 0 };
	float _a_xyz[3];
	float _g_xyz[3];
	float m_q_madg[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
	float _deltat = 0.0f;
	uint32_t _lastUpdate = HAL_GetTick();
	float _GyroMeasError = M_PI * (60.0f / 180.0f); // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	float _beta = sqrt(3.0f / 4.0f) * _GyroMeasError;  // compute beta
	float _GyroMeasDrift = M_PI * (1.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	float _zeta = sqrt(3.0f / 4.0f) * _GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

};

#endif /* INC_IMU_H_ */
