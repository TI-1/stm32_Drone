/*
 * FlightController.cpp
 *
 *  Created on: 23 Jul 2023
 *      Author: tobii
 */

#include "FlightController.h"
#include "helperfns.h"




FlightController::FlightController(IMU *imu, Motors *motors, Remote *remote):_imu(imu),_motors(motors),_remote(remote){
	_rateX = new PID(0,0,0,0);
	_rateY = new PID(0,0,0,0);
	_rateZ = new PID(0,0,0,0);
	_angleX = new PID(0,0,0,0);
	_angleY = new PID(0,0,0,0);


}

FlightController::FlightController(IMU *imu, Motors *motors, Remote *remote,
		flight_mode Fm):_imu(imu),_motors(motors),_remote(remote),_Fm(Fm){
	_rateX = new PID(0,0,0,0);
	_rateY = new PID(0,0,0,0);
	_rateZ = new PID(0,0,0,0);
	_angleX = new PID(0,0,0,0);
	_angleY = new PID(0,0,0,0);
}

void FlightController::setPid(PID *pid, float kp, float ki, float kd,
		float imax) {
	pid -> setkp(kp);
	pid -> setki(ki);
	pid -> setkd(kd);
}

uint8_t FlightController::initialise() {
	uint8_t check_val = 0;
	if (!_imu -> checkStatus()){
		check_val += 1;
		return check_val;
	}
//	else if{
//
//	}

	return 0;
}

void FlightController::loop() {
}

void FlightController::process() {
}

void FlightController::armController() {
	_As = ARMED;
}

void FlightController::disarmController() {
	_As = DISARMED;
}

bool FlightController::isArmed() {
	return _As == ARMED;
}

void FlightController::armCheck(){
	if(_remote -> getRemoteData(Throttle) < SAFETY_POSITION_MIN && _remote -> getRemoteData(Yaw) < SAFETY_POSITION_MIN){
			disarmController();
		}
		if(_remote -> getRemoteData(Throttle) < SAFETY_POSITION_MIN && _remote -> getRemoteData(Yaw) > SAFETY_POSITION_MAX){
				armController();
		}
}
void FlightController::safetyChecks() {
		armCheck();
		gyroLockCheck();
		extremeAngleCheck();



}

void FlightController::gyroLockCheck() {
	if (floatComparison(_imu -> Gyro(Y), _lastGyroValue) && !_onGround){
		_gyroFreezeCounter++;
		if(_gyroFreezeCounter == 25000){
			emergencyStop("gyro freeze");
		}
		else{
			_gyroFreezeCounter = 0;
			_lastGyroValue = _imu -> Gyro(Y);
		}
	}
}

void FlightController::extremeAngleCheck() {

	if(_imu -> Pitch()> SAFE_ANGLE || _imu -> Pitch() < -SAFE_ANGLE || _imu -> Roll() > SAFE_ANGLE || _imu -> Roll() < -SAFE_ANGLE ){
		emergencyStop("angles too high");
	}
}

void FlightController::emergencyStop(const char *reason){
    _emergencyStopped = true;
    _motors -> command(Off);
    for(;;){
    	//Todo implement led indicator / debug message
//        debugger_indicate_emergency(reason);
//        HAL_GPIO_TogglePin(STATUS_GPIO_Port, STATUS_Pin);
//    	HAL_Delay(2000);
    }
}
