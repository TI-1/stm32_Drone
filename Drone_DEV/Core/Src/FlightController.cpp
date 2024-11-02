/*
 * FlightController.cpp
 *
 *  Created on: 23 Jul 2023
 *      Author: tobii
 */

#include "FlightController.h"
#include "helperfns.h"

//TODO set up watchdog timer

FlightController::FlightController(IMU *imu, Motors *motors, Remote *remote,
		UART_HandleTypeDef *telem) :
		_imu(imu), _motors(motors), _remote(remote), _telemHuart(telem) {
	HAL_GPIO_WritePin(RedLED_GPIO_Port, RedLED_Pin, GPIO_PIN_SET);
	initialise();

}

FlightController::FlightController(IMU *imu, Motors *motors, Remote *remote,
		flight_mode Fm, UART_HandleTypeDef *telem) :
		_imu(imu), _motors(motors), _remote(remote), _Fm(Fm), _telemHuart(
				telem){
	HAL_GPIO_WritePin(RedLED_GPIO_Port, RedLED_Pin, GPIO_PIN_SET);
	initialise();
}

FlightController::FlightController() {
	HAL_GPIO_WritePin(RedLED_GPIO_Port, RedLED_Pin, GPIO_PIN_SET);
	initialise();
}



void FlightController::initialise() {
	if ( !_imu -> checkStatus()){
		telemetry.sendMessage(IMU_INIT, 1);
	}
	else{
		HAL_GPIO_WritePin(RedLED_GPIO_Port, RedLED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(YellowLED_GPIO_Port, YellowLED_Pin, GPIO_PIN_SET);
		telemetry.sendMessage(IMU_INIT, 0);
	}
}

void FlightController::loop() { //Loop rate currently determined by IMU dataReady rate = 200hz
	if (_imu->dataReady()) {
		_imu->readIMU();
		telemetry.gatherData(_imu);
		telemetry.update();
		process();
	}
}

void FlightController::process() {
	safetyChecks();
	if (_onGround) {
		resetPIDIntegrals();
	}
	computePIDs();
	computeMotorOutputs();
	if (isArmed() && minThrottle()) {
		_onGround = false;
		_motors->command(On);
	} else {
		_onGround = true;
		_motors->command(Off);
	}
}

void FlightController::armController() {
	_As = ARMED;
	//TODO sort out telemetry message numbers
	HAL_GPIO_WritePin(YellowLED_GPIO_Port, YellowLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_SET);
	telemetry.sendMessage(11, 1);
}

void FlightController::disarmController() {
	_As = DISARMED;
	//TODO sort out telemetry message numbers
	HAL_GPIO_WritePin(YellowLED_GPIO_Port, YellowLED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_RESET);
	telemetry.sendMessage(11, 0);
}

bool FlightController::isArmed() {
	return _As == ARMED;
}

void FlightController::armCheck() {
	//TODO Add logic to check for signal lost and handle maybe throw an emergency stop
	if (_remote->getRemoteData(Arm) == 1000 && _As != DISARMED) {
			disarmController();
	} else if (_remote->getRemoteData(Arm) == 2000 && _As != ARMED) {
		armController();
	}
}

void FlightController::safetyChecks() {
	armCheck();
	if (isArmed()) {
		gyroLockCheck();
		extremeAngleCheck();
	}
}

void FlightController::gyroLockCheck() {
	if (floatComparison(_imu->Gyro(Y), _lastGyroValue) && !_onGround) {
		_gyroFreezeCounter++;
	}
	if (_gyroFreezeCounter >= GYRO_FREEZE_THRESHOLD) {
		telemetry.sendMessage(GYRO_FREEZE, 1);
		emergencyStop(GYRO_FREEZE);
	} else {
		_gyroFreezeCounter = 0;
		_lastGyroValue = _imu->Gyro(Y);
	}
}

void FlightController::extremeAngleCheck() {

	if (_imu->Pitch() > SAFE_ANGLE || _imu->Pitch() < -SAFE_ANGLE
			|| _imu->Roll() > SAFE_ANGLE || _imu->Roll() < -SAFE_ANGLE) {
		telemetry.sendMessage(EXTREME_ANGLE, 1);
		emergencyStop(EXTREME_ANGLE);
	}
}

void FlightController::emergencyStop(emergency_stop reason) {
	_emergencyStopped = true;
	_motors->command(Off);
	indicateEmergency(reason);

}

void FlightController::resetPIDIntegrals() {
	_rateX.reset_i();
	_rateY.reset_i();
	_rateZ.reset_i();
	_angleX.reset_i();
	_angleY.reset_i();
}

void FlightController::computePIDs() {
	if (_Fm == FC_ANGLE_MODE) {
		_angleX.controllerUpdate(
				map<int16_t>(_remote->getRemoteData(Roll), REMOTE_MIN,
						REMOTE_MAX, ANGLE_MIN, ANGLE_MAX), _imu->Roll());
		_angleY.controllerUpdate(
				map<int16_t>(_remote->getRemoteData(Pitch), REMOTE_MIN,
						REMOTE_MAX, ANGLE_MIN, ANGLE_MAX), _imu->Pitch());
		_rateX.controllerUpdate(_angleX.getOutput(), _imu->Gyro(X));
		_rateY.controllerUpdate(_angleY.getOutput(), _imu->Gyro(Y));
	} else {
		_rateX.controllerUpdate(
				map<int16_t>(_remote->getRemoteData(Roll), REMOTE_MIN,
						REMOTE_MAX, ROLL_RATE_MIN, ROLL_RATE_MAX),
				_imu->Gyro(X));
		_rateY.controllerUpdate(
				map<int16_t>(_remote->getRemoteData(Pitch), REMOTE_MIN,
						REMOTE_MAX, PITCH_RATE_MIN, PITCH_RATE_MAX),
				_imu->Gyro(Y));
	}
	_rateZ.controllerUpdate(
			map<int16_t>(_remote->getRemoteData(Yaw), REMOTE_MIN, REMOTE_MAX,
					YAW_RATE_MIN, YAW_RATE_MAX), _imu->Gyro(Z));
}

void FlightController::computeMotorOutputs() {
	int16_t throttle = map<int16_t>(_remote->getRemoteData(Throttle),
			REMOTE_MIN, REMOTE_MAX, Throttle_Min, Throttle_Max);
	_motors->setOutput(FrontRight,
			throttle - _rateX.getOutput() - _rateY.getOutput()
					+ _rateZ.getOutput());
	_motors->setOutput(RearLeft,
			throttle + _rateX.getOutput() + _rateY.getOutput()
					+ _rateZ.getOutput());
	_motors->setOutput(FrontLeft,
			throttle + _rateX.getOutput() - _rateY.getOutput()
					- _rateZ.getOutput());
	_motors->setOutput(RearRight,
			throttle - _rateX.getOutput() + _rateY.getOutput()
					- _rateZ.getOutput());
}

bool FlightController::minThrottle() {
	return _remote->getRemoteData(Throttle) >= Throttle_Min;
}

void FlightController::fcDebug(debug_group debug) {
	switch (debug) {
	case ALL:
		debugArmStatus();
		_imu->debugGyro();
		_imu->debugYPR();
		_rateX.debugPID("X");
		_rateY.debugPID("Y");
		_rateZ.debugPID("Z");
		if (_Fm == FC_ANGLE_MODE) {
			_angleX.debugPID("Roll Angle");
			_angleY.debugPID("Pitch Angle");
		}
		_motors->DebugMotors();
		break;
	case GYRO:
		_imu->debugGyro();
		break;
	case YPR:
		_imu->debugYPR();
		break;
	case ANGLES:
		_angleX.debugPID("Roll Angle");
		_angleY.debugPID("Pitch Angle");
		break;
	case PIDS:
		_rateX.debugPID("X");
		_rateY.debugPID("Y");
		_rateZ.debugPID("Z");
		break;
	case MOTORS:
		_motors->DebugMotors();
		break;
	case FC_STATUS:
		debugArmStatus();
		break;
	default:
		break;
	}
}


void FlightController::debugArmStatus() {
	Debug(printf("Arm status is:%s\r\n", isArmed() ? "true" : "false"));
}


void FlightController::debugloop(int frequency, debug_group debug) {
	static long unsigned int start = HAL_GetTick();
	if (HAL_GetTick() - start
			> static_cast<long unsigned int>(1000 / frequency)) {
		fcDebug(debug);
		start = HAL_GetTick();
	}
}
void FlightController::indicateEmergency(emergency_stop reason) {
	switch (reason) {
	case GYRO_FREEZE:
		while (1) {
			LOG_DEBUG("EMERGENCY STOP: GYRO FREEZE");
			emergencyBlink(500);
		}
		break;
	case EXTREME_ANGLE:
		while (1) {
			LOG_DEBUG("EMERGENCY STOP: ANGLE EXCEEDS LIMIT");
			emergencyBlink(2000);
		}
		break;
	case IMU_INIT:
		while (1) {
			LOG_DEBUG("EMERGENCY STOP: IMU INITIALISATION FAILED");
			emergencyBlink(3000);
		}
		break;
	case WATCHDOG_RESET:
		while (1) {
			LOG_DEBUG("EMERGENCY STOP: FAILED TO RESET WATCHDOG");
			emergencyBlink(5000);
		}
		break;
	default:
		while (1) {
			LOG_DEBUG("EMERGENCY STOP");
		}

	}
}


void FlightController::emergencyBlink(int delayMs) {
	while(1){
		HAL_GPIO_TogglePin(RedLED_GPIO_Port, RedLED_Pin);
					HAL_Delay(delayMs);
	}
}
