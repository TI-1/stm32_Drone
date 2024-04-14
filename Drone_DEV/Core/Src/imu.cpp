/*
 * imu.cpp
 *
 *  Created on: Apr 19, 2023
 *      Author: tobii
 */
#include "imu.h"
#include "debug.h"
#include "stdio.h"

//TODO Create virtual class for IMU

/**
 * IMU class constructor
 * @param calibration set calibration flag as true or false
 */
IMU::IMU(uint8_t Ascale = ACCEL_FS_SEL_4G, uint8_t Gscale = GYRO_FS_SEL_1000DPS, uint8_t sampleRate = 4, bool calibration = false) {
	initialise(Ascale, Gscale, sampleRate);
}

/**
 * IMU class default constructor
 */
IMU::IMU(uint8_t Ascale = ACCEL_FS_SEL_4G, uint8_t Gscale = GYRO_FS_SEL_1000DPS, uint8_t sampleRate = 4) {
	initialise(Ascale, Gscale, sampleRate);
}

/**
 * Initialise MPU
 * @param Ascale Accleration scale factor
 * @param Gscale Gyroscope scale factor
 * @param sampleRate Data sample rate
 * @return
 */
uint8_t IMU::initialise(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate) {
	init_status = initMPU9250(Ascale, Gscale, sampleRate);
	//TODO possibly add check to make sure offsets set correctly
	 setXAccelOffset(USER_XA_OFFSET);
	 setYAccelOffset(USER_YA_OFFSET);
	 setZAccelOffset(USER_ZA_OFFSET);
	 setXGyroOffset(USER_XG_OFFSET);
	 setYGyroOffset(USER_YG_OFFSET);
	 setZGyroOffset(USER_ZG_OFFSET);
	return init_status;
}

float IMU::Yaw() {
	return _ypr[0];
}

float IMU::Pitch() {
	return _ypr[1];
}

float IMU::Roll() {
	return _ypr[2];
}

float IMU::Accel(axis axis) {
	switch (axis){
		case X:
			return _a_xyz[0];
			break;
		case Y:
			return _a_xyz[1];
			break;
		case Z:
			return _a_xyz[2];
			break;
		default:
			return 0.0f;
	}
}

float IMU::Gyro(axis axis) {
	switch (axis){
		case X:
			return _g_xyz[0];
			break;
		case Y:
			return _g_xyz[1];
			break;
		case Z:
			return _g_xyz[2];
			break;
		default:
			return 0.0f;
	}
}

void IMU::readIMU() {
	uint32_t Now;
	readAccelDataScaled(_a_xyz);
	readGyroDataScaled(_g_xyz);
	Now = HAL_GetTick();
	_deltat = static_cast<float>(((Now - _lastUpdate) / 1000.0f));
	calculateMadgwickQuaternion();
	_lastUpdate = Now;
}

bool IMU::checkStatus(){
	return (getMPU9250ID() == MPU1);
}

bool IMU::dataReady() {
	return checkNewAccelGyroData();
}

void IMU::calculateMadgwickQuaternion() {
	float ax = _a_xyz[0];
    float ay = _a_xyz[1];
    float az = _a_xyz[2];
	float gx = _g_xyz[0]*M_PI/180.0f;
	float gy = _g_xyz[1]*M_PI/180.0f;
	float gz = _g_xyz[2]*M_PI/180.0f;
	float q1 = m_q_madg[0];
	float q2 = m_q_madg[1];
	float q3 = m_q_madg[2];
	float q4 = m_q_madg[3];         // short name local variable for readability
	float norm;                                               // vector norm
	float f1;
	float f2;
	float f3;                                         // objective funcyion elements
	float J_11or24;
	float J_12or23;
	float J_13or22;
	float J_14or21;
	float J_32;
	float J_33; // objective function Jacobian elements
	float qDot1;
	float qDot2;
	float qDot3;
	float qDot4;
	float hatDot1;
	float hatDot2;
	float hatDot3;
	float hatDot4;
	float gerrx;
	float gerry;
	float gerrz;
	float gbiasx = 0;
	float gbiasy = 0;
	float gbiasz = 0;  // gyro bias error

	// Auxiliary variables to avoid repeated arithmetic
	float _halfq1 = 0.5f * q1;
	float _halfq2 = 0.5f * q2;
	float _halfq3 = 0.5f * q3;
	float _halfq4 = 0.5f * q4;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	//            float _2q1q3 = 2.0f * q1 * q3;
	//            float _2q3q4 = 2.0f * q3 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm <= 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Compute the objective function and Jacobian
	f1 = _2q2 * q4 - _2q1 * q3 - ax;
	f2 = _2q1 * q2 + _2q3 * q4 - ay;
	f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
	J_11or24 = _2q3;
	J_12or23 = _2q4;
	J_13or22 = _2q1;
	J_14or21 = _2q2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	hatDot1 = J_14or21 * f2 - J_11or24 * f1;
	hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
	hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
	hatDot4 = J_14or21 * f1 + J_11or24 * f2;

	// Normalize the gradient
	norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
	hatDot1 /= norm;
	hatDot2 /= norm;
	hatDot3 /= norm;
	hatDot4 /= norm;

	// Compute estimated gyroscope biases
	gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
	gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
	gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

	// Compute and remove gyroscope biases
	gbiasx += gerrx * _deltat * _zeta;
	gbiasy += gerry * _deltat * _zeta;
	gbiasz += gerrz * _deltat * _zeta;
	//           gx -= gbiasx;
	//           gy -= gbiasy;
	//           gz -= gbiasz;

	// Compute the quaternion derivative
	qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
	qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
	qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
	qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

	// Compute then integrate estimated quaternion derivative
	q1 += (qDot1 -(_beta * hatDot1)) * _deltat;
	q2 += (qDot2 -(_beta * hatDot2)) * _deltat;
	q3 += (qDot3 -(_beta * hatDot3)) * _deltat;
	q4 += (qDot4 -(_beta * hatDot4)) * _deltat;

	// Normalize the quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	m_q_madg[0] = q1 * norm;
	m_q_madg[1] = q2 * norm;
	m_q_madg[2] = q3 * norm;
	m_q_madg[3] = q4 * norm;

	_ypr[0] = (atan2(2.0f * (m_q_madg[1] * m_q_madg[2] + m_q_madg[0] * m_q_madg[3]), m_q_madg[0] * m_q_madg[0] + m_q_madg[1] * m_q_madg[1] - m_q_madg[2] * m_q_madg[2] - m_q_madg[3] * m_q_madg[3])) * 180.f/M_PI;
	_ypr[1] = (-asin(2.0f * (m_q_madg[1] * m_q_madg[3] - m_q_madg[0] * m_q_madg[2]))) * 180.f/M_PI;
	_ypr[2] = (atan2(2.0f * (m_q_madg[0] * m_q_madg[1] + m_q_madg[2] * m_q_madg[3]), m_q_madg[0] * m_q_madg[0] - m_q_madg[1] * m_q_madg[1] - m_q_madg[2] * m_q_madg[2] + m_q_madg[3] * m_q_madg[3])) * 180.f/M_PI;
}

void IMU::debugGyro() {
	printf("gyro_X:%f\tgyro_Y:%f\tgyro_Z:%f\r\n",_g_xyz[0],_g_xyz[1], _g_xyz[2]);
}

void IMU::debugYPR() {
	printf("Yaw:%3.3f\tPitch:%3.3f\tRoll:%3.3f\tDelta:%3.3f\r\n",_ypr[0],_ypr[1],_ypr[2],_deltat);
}

void IMU::calibrateImu() {
	float dest1[3];
	float dest2[3];
	calibrateMPU9250(dest1, dest2);
}
