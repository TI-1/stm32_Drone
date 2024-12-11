/*
 * FlightController.cpp
 *
 *  Created on: 23 Jul 2023
 *      Author: tobii
 */

#include "FlightController.h"
#include "helperfns.h"

//TODO set up watchdog timer

FlightController::FlightController(IMU *imu, Motors *motors, Remote *remote) :
		_imu(imu), _motors(motors), _remote(remote) {
	setState(BOOTING);
	initialise();

}

FlightController::FlightController(IMU *imu, Motors *motors, Remote *remote,
		flight_mode Fm) :
		_imu(imu), _motors(motors), _remote(remote), _Fm(Fm){
	setState(BOOTING);
	initialise();
}

FlightController::FlightController() {
	setState(BOOTING);
	initialise();
}



void FlightController::initialise() {
	if ( !_imu -> checkStatus()){
		setState(EMERGENCY_STOP);
		indicateEmergency(IMU_INIT);
	}
	else{
		setState(STANDBY);
	}
}

void FlightController::loop() { //Loop rate currently determined by IMU dataReady rate = 200hz
	switch(_currentState){
	case BOOTING:
		break;

	case STANDBY:
		if(_remote -> getRemoteData(Arm) == 2000){
			setState(ARMED);
		}
		break;

	case ARMED:
	case IN_FLIGHT:
		if(_remote -> getRemoteData(Arm) != 2000){
			setState(DISARMED);
		}
		if (_imu->dataReady()) {
				_imu->readIMU();
				process();
			}
		break;

	case DISARMED:
		if(_remote -> getRemoteData(Arm) == 2000){
			setState(ARMED);
		}
		break;

	case EMERGENCY_STOP:
		break;

	default:
		break;
	}
}

void FlightController::process() {
	safetyChecks();

	if(_currentState == EMERGENCY_STOP){
		return;
	}

	if (_currentState == ARMED || _currentState == IN_FLIGHT){
		computePIDs();
		computeMotorOutputs();
		if (_currentState == ARMED){
			setState(IN_FLIGHT);
		}
	}
}


void FlightController::safetyChecks() {
		gyroLockCheck();
		extremeAngleCheck();
}

void FlightController::gyroLockCheck() {
	if (floatComparison(_imu->Gyro(Y), _lastGyroValue) && _currentState == IN_FLIGHT) {
		_gyroFreezeCounter++;
	}
	if (_gyroFreezeCounter >= GYRO_FREEZE_THRESHOLD) {
		setState(EMERGENCY_STOP);
		indicateEmergency(GYRO_FREEZE);
	} else {
		_gyroFreezeCounter = 0;
		_lastGyroValue = _imu->Gyro(Y);
	}
}

void FlightController::extremeAngleCheck() {

	if (_imu->Pitch() > SAFE_ANGLE || _imu->Pitch() < -SAFE_ANGLE
			|| _imu->Roll() > SAFE_ANGLE || _imu->Roll() < -SAFE_ANGLE) {
		setState(EMERGENCY_STOP);
		indicateEmergency(EXTREME_ANGLE);
	}
}

void FlightController::resetPIDIntegrals() {
	pidControllers[ROLL].reset_i();
	pidControllers[PITCH].reset_i();
	pidControllers[YAW].reset_i();
	pidControllers[ANGLE_ROLL].reset_i();
	pidControllers[ANGLE_PITCH].reset_i();
}

void FlightController::computePIDs() {
	if (_Fm == FC_ANGLE_MODE) {
		pidControllers[ANGLE_ROLL].controllerUpdate(
				map<int16_t>(_remote->getRemoteData(Roll), REMOTE_MIN,
						REMOTE_MAX, ANGLE_MIN, ANGLE_MAX), _imu->Roll());
		pidControllers[ANGLE_PITCH].controllerUpdate(
				map<int16_t>(_remote->getRemoteData(Pitch), REMOTE_MIN,
						REMOTE_MAX, ANGLE_MIN, ANGLE_MAX), _imu->Pitch());
		pidControllers[ROLL].controllerUpdate(pidControllers[ANGLE_ROLL].getOutput(), _imu->Gyro(X));
		pidControllers[PITCH].controllerUpdate(pidControllers[ANGLE_PITCH].getOutput(), _imu->Gyro(Y));
	} else {
		pidControllers[ROLL].controllerUpdate(
				map<int16_t>(_remote->getRemoteData(Roll), REMOTE_MIN,
						REMOTE_MAX, ROLL_RATE_MIN, ROLL_RATE_MAX),
				_imu->Gyro(X));
		pidControllers[PITCH].controllerUpdate(
				map<int16_t>(_remote->getRemoteData(Pitch), REMOTE_MIN,
						REMOTE_MAX, PITCH_RATE_MIN, PITCH_RATE_MAX),
				_imu->Gyro(Y));
	}
	pidControllers[YAW].controllerUpdate(
			map<int16_t>(_remote->getRemoteData(Yaw), REMOTE_MIN, REMOTE_MAX,
					YAW_RATE_MIN, YAW_RATE_MAX), _imu->Gyro(Z));
}

void FlightController::computeMotorOutputs() {
	int16_t throttle = map<int16_t>(_remote->getRemoteData(Throttle),
			REMOTE_MIN, REMOTE_MAX, Throttle_Min, Throttle_Max);
	_motors->setOutput(FrontRight,
			throttle - pidControllers[ROLL].getOutput() - pidControllers[PITCH].getOutput()
					+ pidControllers[YAW].getOutput());
	_motors->setOutput(RearLeft,
			throttle + pidControllers[ROLL].getOutput() + pidControllers[PITCH].getOutput()
					+ pidControllers[YAW].getOutput());
	_motors->setOutput(FrontLeft,
			throttle + pidControllers[ROLL].getOutput() - pidControllers[PITCH].getOutput()
					- pidControllers[YAW].getOutput());
	_motors->setOutput(RearRight,
			throttle - pidControllers[ROLL].getOutput() + pidControllers[PITCH].getOutput()
					- pidControllers[YAW].getOutput());
}

void FlightController::setState(FlightState newState) {
	if (_currentState != newState) {
	        onExitState(_currentState);
	        _currentState = newState;
	        onEnterState(newState);
	    }
}

void FlightController::onEnterState(FlightState state) {
	switch (state) {
	case BOOTING:
	    HAL_GPIO_WritePin(RedLED_GPIO_Port, RedLED_Pin, GPIO_PIN_SET);
	    break;

	case STANDBY:
	    HAL_GPIO_WritePin(YellowLED_GPIO_Port, YellowLED_Pin, GPIO_PIN_SET);
	    break;

	case ARMED:
	    HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_SET);
	    break;

	case DISARMED:
		_motors->command(Off);
		resetPIDIntegrals();
		break;

	case IN_FLIGHT:
	    _motors->command(On);
	    break;

	case EMERGENCY_STOP:
	    _motors->command(Off);
	    break;

	default:
	    break;
	}
}

void FlightController::onExitState(FlightState state) {
	 switch (state) {
	 case BOOTING:
	     HAL_GPIO_WritePin(RedLED_GPIO_Port, RedLED_Pin, GPIO_PIN_RESET);
	     break;

	 case STANDBY:
	     HAL_GPIO_WritePin(YellowLED_GPIO_Port, YellowLED_Pin, GPIO_PIN_RESET);
	     break;

	 case ARMED:
	     HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_RESET);
	     break;

	 default:
	     break;
	 }
}

void FlightController::initialisePIDs(){
	pidControllers[ROLL] = PID(1.0, 0.1, 0.05, 100.0);
	pidControllers[PITCH] = PID(1.0, 0.1, 0.05, 100.0);
	pidControllers[YAW] = PID(1.0, 0.1, 0.05, 100.0);
	pidControllers[ANGLE_ROLL] = PID(1.5, 0.2, 0.1, 100.0);
	pidControllers[ANGLE_PITCH] = PID(1.5, 0.2, 0.1, 100.0);
}

void FlightController::fcDebug(debug_group debug) {
	switch (debug) {
	case ALL:
		_imu->debugGyro();
		_imu->debugYPR();
		pidControllers[ROLL].debugPID("X");
		pidControllers[PITCH].debugPID("Y");
		pidControllers[YAW].debugPID("Z");
		if (_Fm == FC_ANGLE_MODE) {
			pidControllers[ANGLE_ROLL].debugPID("Roll Angle");
			pidControllers[ANGLE_PITCH].debugPID("Pitch Angle");
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
		pidControllers[ANGLE_ROLL].debugPID("Roll Angle");
		pidControllers[ANGLE_PITCH].debugPID("Pitch Angle");
		break;
	case PIDS:
		pidControllers[ROLL].debugPID("X");
		pidControllers[PITCH].debugPID("Y");
		pidControllers[YAW].debugPID("Z");
		break;
	case MOTORS:
		_motors->DebugMotors();
		break;
	case FC_STATUS:
		break;
	default:
		break;
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

