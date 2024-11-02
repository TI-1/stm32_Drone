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


typedef enum {
	FC_RATE_MODE = 0, FC_ANGLE_MODE
} flight_mode;

typedef enum {
	ARMED = 0, DISARMED
} arm_status;

typedef enum {
	ALL = 0, GYRO, YPR, ANGLES, PIDS, MOTORS, FC_STATUS

} debug_group;

typedef enum {
	GYRO_FREEZE = 0, EXTREME_ANGLE, IMU_INIT, WATCHDOG_RESET
} emergency_stop;

class FlightController {
public:

	FlightController(IMU *imu, Motors *motors, Remote *remote,
			UART_HandleTypeDef *telem);
	FlightController(IMU *imu, Motors *motors, Remote *remote, flight_mode Fm,
			UART_HandleTypeDef *telem);
	FlightController();
	FlightController(const FlightController &obj) = delete;
	FlightController& operator=(const FlightController &obj) = delete;
	void initialise();
	void loop();
	void process();
	void fcDebug(debug_group debug);
	void debugloop(int frequency, debug_group debug);

protected:
	void armController();
	void disarmController();
	void gyroLockCheck();
	void extremeAngleCheck();

	uint16_t _gyroFreezeCounter = 0;

private:
	static FlightController *_instance;
	PID _rateX = PID(0, 0, 0, 0);
	PID _rateY = PID(0, 0, 0, 0);
	PID _rateZ = PID(0, 0, 0, 0);
	PID _angleX = PID(0, 0, 0, 0);
	PID _angleY = PID(0, 0, 0, 0);
	Telemetry telemetry{_telemHuart, 1};
	IMU *_imu = nullptr;
	Motors *_motors = nullptr;
	Remote *_remote = nullptr;
	flight_mode _Fm = FC_RATE_MODE;
	arm_status _As = DISARMED;
	bool _onGround = true;
	UART_HandleTypeDef *_telemHuart = nullptr;
	float _lastGyroValue = 0.0;
	bool _emergencyStopped = false;
	bool _watchdogReset = true;
	bool _watchdogStop = false;
	bool _telemetryFlag = false;
	TIM_HandleTypeDef *_frequencyTimer = nullptr;
	const int GYRO_FREEZE_THRESHOLD = 25000;
	const uint8_t START_BYTE = 66;
	const uint8_t STOP_BYTE = 66;

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
	void indicateEmergency(emergency_stop reason);
	void emergencyBlink(int delayMs);

};

#endif /* SRC_FLIGHTCONTROLLER_H_ */
