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
#include "tim.h"
#include "usart.h"
#include "telemetry.h"
#include <array>

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


#ifdef DEBUG
#define LOG_DEBUG(msg) printf("DEBUG: " msg "\n")
#else
#define LOG_DEBUG(msg)
#endif

enum FlightState {
    BOOTING,
    STANDBY,
    ARMED,
	DISARMED,
    IN_FLIGHT,
    EMERGENCY_STOP
};

typedef enum {
	FC_RATE_MODE = 0, FC_ANGLE_MODE
} flight_mode;


typedef enum {
	ALL = 0, GYRO, YPR, ANGLES, PIDS, MOTORS, FC_STATUS

} debug_group;

typedef enum {
	GYRO_FREEZE = 0, EXTREME_ANGLE, IMU_INIT
} emergency_stop;

enum {
	ROLL,
	PITCH,
	YAW,
	ANGLE_ROLL,
	ANGLE_PITCH,
	NUM_PIDS
};

class FlightController {
public:

	FlightController(IMU *imu, Motors *motors, Remote *remote);
	FlightController(IMU *imu, Motors *motors, Remote *remote, flight_mode Fm);
	FlightController();
	FlightController(const FlightController &obj) = delete;
	FlightController& operator=(const FlightController &obj) = delete;

	void initialise();
	void loop();


private:

	FlightState _currentState;

	IMU *_imu = nullptr;
	Motors *_motors = nullptr;
	Remote *_remote = nullptr;
	flight_mode _Fm = FC_RATE_MODE;

	uint16_t _gyroFreezeCounter = 0;
	std::array<PID, NUM_PIDS> pidControllers;
	float _lastGyroValue = 0.0;
	const int GYRO_FREEZE_THRESHOLD = 25000;


	void setState(FlightState newState);
	void onEnterState(FlightState state);
	void onExitState(FlightState state);
	void process();
	void gyroLockCheck();
	void extremeAngleCheck();
	void fcDebug(debug_group debug);
	void safetyChecks();
	void resetPIDIntegrals();
	void computePIDs();
	void computeMotorOutputs();
	void initialisePIDs();
	void indicateEmergency(emergency_stop reason);
	void emergencyBlink(int delayMs);

};

#endif /* SRC_FLIGHTCONTROLLER_H_ */
