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
#include "debug.h"
#include "tim.h"

#define SAFETY_POSITION_MIN 1100 //TODO think of better name
#define SAFETY_POSITION_MAX	1950
#define SAFE_ANGLE 60
#define Throttle_Max 1700
#define Throttle_Min 1070
#define ANGLE_MAX 45
#define ANGLE_MIN -1 * ANGLE_MAX
#define ROLL_RATE_MAX     250
#define ROLL_RATE_MIN     -1 * ROLL_RATE_MAX
#define YAW_RATE_MAX      100
#define YAW_RATE_MIN      -1 * YAW_RATE_MAX
#define PITCH_RATE_MAX    250
#define PITCH_RATE_MIN	  -1 * PITCH_RATE_MAX


typedef enum {
  FC_RATE_MODE = 0,
  FC_ANGLE_MODE
} flight_mode;

typedef enum {
	ARMED = 0,
	DISARMED
} arm_status;

typedef enum {
	ALL = 0,
	GYRO,
	YPR,
	ANGLES,
	PIDS,
	MOTORS,
	FC_STATUS

}debug_group;

class FlightController {
public:

	FlightController(IMU *imu, Motors *motors, Remote *remote);
	FlightController(IMU *imu, Motors *motors, Remote *remote, flight_mode Fm);
	FlightController();
	void setPid(PID *pid, float kp, float ki, float kd, float imax);
	void initialise();
	void loop();
	void process();
	void watchdog_enable();
	void watchdog_reset();
	void watchdog_stop();
	void fcDebug(debug_group debug);
	void debugloop(int frequency, debug_group debug);
	void telemetryDataLoop(int frequency);


protected:
	void armController();
	void disarmController();
	void gyroLockCheck();
	void extremeAngleCheck();

	uint16_t _gyroFreezeCounter = 0;

private:
	PID *_rateX = nullptr;
	PID *_rateY = nullptr;
	PID *_rateZ = nullptr;
	PID *_angleX = nullptr;
	PID *_angleY = nullptr;
	IMU *_imu = nullptr;
	Motors *_motors = nullptr;
	Remote *_remote= nullptr;
	flight_mode _Fm = FC_RATE_MODE;
	arm_status _As = DISARMED;
	bool _onGround = true;

	float _lastGyroValue = 0.0;
	bool _emergencyStopped = false;
	bool watchdog_was_reset = true;
	bool watchdog_stop_var = false;


	bool isArmed();
	void safetyChecks();
	void armCheck();
	void emergencyStop(emergency_stop reason);
	void resetPIDIntegrals();
	void computePIDs();
	void computeMotorOutputs();
	bool minThrottle();
	void debugArmStatus();
	void initialisePIDs();
	void sendTelemetryData();
	void sendTelemetryMessage(uint8_t message_code, uint8_t value);
	void watchdog_verify();
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

};

#endif /* SRC_FLIGHTCONTROLLER_H_ */
