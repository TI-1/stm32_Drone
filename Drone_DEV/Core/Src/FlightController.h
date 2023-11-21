/*
 * FlightController.h
 *
 *  Created on: 23 Jul 2023
 *      Author: tobii
 */

#ifndef SRC_FLIGHTCONTROLLER_H_
#define SRC_FLIGHTCONTROLLER_H_

#include "remote.h"
#include "pid.h"
#include "imu.h"
#include "motors.h"

#define SAFETY_POSITION_MIN 1100 //TODO think of better name
#define SAFETY_POSITION_MAX	1950
#define SAFE_ANGLE 60


typedef enum {
  FC_RATE_MODE = 0,
  FC_ANGLE_MODE
} flight_mode;

typedef enum {
	ARMED = 0,
	DISARMED
} arm_status;


class FlightController {
public:

	FlightController(IMU *imu, Motors *motors, Remote *remote);
	FlightController(IMU *imu, Motors *motors, Remote *remote, flight_mode Fm);
	void setPid(PID *pid, float kp, float ki, float kd, float imax);
	uint8_t initialise();
	void loop();
	void process();



private:
	PID *_rateX;
	PID *_rateY;
	PID *_rateZ;
	PID *_angleX;
	PID *_angleY;
	IMU *_imu;
	Motors *_motors;
	Remote *_remote;
	flight_mode _Fm = FC_RATE_MODE;
	arm_status _As = DISARMED;
	bool _onGround;
	uint16_t _gyroFreezeCounter = 0;
	float _lastGyroValue = 0.0;
	bool _emergencyStopped = false;

	void armController();
	void disarmController();
	bool isArmed();
	void safetyChecks();
	void armCheck();
	void gyroLockCheck();
	void extremeAngleCheck();
	void emergencyStop(const char *reason);
};

#endif /* SRC_FLIGHTCONTROLLER_H_ */
