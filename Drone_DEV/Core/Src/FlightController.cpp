/*
 * FlightController.cpp
 *
 *  Created on: 23 Jul 2023
 *      Author: tobii
 */

#include "FlightController.h"
#include "helperfns.h"
#include "telemetry.h"




FlightController::FlightController(IMU *imu, Motors *motors, Remote *remote):_imu(imu),_motors(motors),_remote(remote){
	initialisePIDs();
	HAL_GPIO_WritePin(ILED1_GPIO_Port, ILED1_Pin, GPIO_PIN_SET);
	initialise();
}

FlightController::FlightController(IMU *imu, Motors *motors, Remote *remote,
		flight_mode Fm):_imu(imu),_motors(motors),_remote(remote),_Fm(Fm){
	initialisePIDs();
	HAL_GPIO_WritePin(ILED1_GPIO_Port, ILED1_Pin, GPIO_PIN_SET);
	initialise();
}

FlightController::FlightController(){
	HAL_GPIO_WritePin(ILED1_GPIO_Port, ILED1_Pin, GPIO_PIN_SET);
}

void FlightController::setPid(PID *pid, float kp, float ki, float kd,
		float imax) {
	pid -> setkp(kp);
	pid -> setki(ki);
	pid -> setkd(kd);
}

void FlightController::initialise() {
	if (_imu == nullptr || !_imu -> checkStatus() || _imu -> init_status != ok){
		sendTelemetryMessage(IMU_INIT, 1);
		emergencyStop(IMU_INIT);
	}
	else{
		HAL_GPIO_WritePin(ILED1_GPIO_Port, ILED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ILED2_GPIO_Port, ILED2_Pin, GPIO_PIN_SET);
		sendTelemetryMessage(IMU_INIT, 0);
	}
}

void FlightController::loop() { //Loop rate currently determined by IMU dataReady rate = 200hz
	if (_imu -> dataReady()){
		_imu -> readIMU();
		HAL_GPIO_TogglePin(ILED3_GPIO_Port, ILED3_Pin);
		process();
	}
}

void FlightController::process() {
	safetyChecks();
	if(_onGround){
		resetPIDIntegrals();
	}
	computePIDs();
	computeMotorOutputs();
	if (isArmed() && minThrottle()){
		_onGround = false;
		_motors -> command(On);
	}
	else{
		_onGround = true;
		_motors -> command(Off);
	}
}

void FlightController::armController() {
	_As = ARMED;
	//TODO sort out telemetry message numbers
	HAL_GPIO_WritePin(ILED2_GPIO_Port, ILED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ILED3_GPIO_Port, ILED3_Pin, GPIO_PIN_SET);
	sendTelemetryMessage(11, 0);
}

void FlightController::disarmController() {
	_As = DISARMED;
	//TODO sort out telemetry message numbers
	HAL_GPIO_WritePin(ILED2_GPIO_Port, ILED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ILED3_GPIO_Port, ILED3_Pin, GPIO_PIN_RESET);
	sendTelemetryMessage(11, 1);
}

bool FlightController::isArmed() {
	return _As == ARMED;
}

void FlightController::armCheck(){
	//TODO Add logic to check for signal lost and handle maybe throw an emergency stop
	if(_remote -> getRemoteData(Arm) == 1000){
			disarmController();
		}
	else if(_remote -> getRemoteData(Arm) == 2000){
				armController();
		}
}

void FlightController::safetyChecks() {
		armCheck();
		if(isArmed()){
			gyroLockCheck();
			extremeAngleCheck();
		}
}

void FlightController::gyroLockCheck() {
	if (floatComparison(_imu -> Gyro(Y), _lastGyroValue) && !_onGround){
		_gyroFreezeCounter++;
	}
	if(_gyroFreezeCounter >= 25000){
		sendTelemetryMessage(GYRO_FREEZE, 1);
		emergencyStop(GYRO_FREEZE);
	}
	else{
		_gyroFreezeCounter = 0;
		_lastGyroValue = _imu -> Gyro(Y);
	}
}

void FlightController::extremeAngleCheck() {

	if(_imu -> Pitch()> SAFE_ANGLE || _imu -> Pitch() < -SAFE_ANGLE || _imu -> Roll() > SAFE_ANGLE || _imu -> Roll() < -SAFE_ANGLE ){
		sendTelemetryMessage(EXTREME_ANGLE, 1);
		emergencyStop(EXTREME_ANGLE);
	}
}

void FlightController::emergencyStop(emergency_stop reason){
    _emergencyStopped = true;
    _motors -> command(Off);
    indicateEmergency(reason);

}

void FlightController::resetPIDIntegrals() {
	_rateX -> reset_i();
	_rateY -> reset_i();
	_rateZ -> reset_i();
	_angleX -> reset_i();
	_angleY -> reset_i();
}

void FlightController::computePIDs() {
	if (_Fm == FC_ANGLE_MODE){
		_angleX -> controllerUpdate(map<int16_t>(_remote->getRemoteData(Roll),REMOTE_MIN,REMOTE_MAX,ANGLE_MIN, ANGLE_MAX), _imu ->Roll());
		_angleY -> controllerUpdate(map<int16_t>(_remote->getRemoteData(Pitch),REMOTE_MIN,REMOTE_MAX,ANGLE_MIN, ANGLE_MAX), _imu ->Pitch());
		_rateX -> controllerUpdate(_angleX -> getOutput(),_imu ->Gyro(X));
		_rateY -> controllerUpdate(_angleY -> getOutput(),_imu ->Gyro(Y));
	}
	else{
		_rateX -> controllerUpdate(map<int16_t>(_remote->getRemoteData(Roll),REMOTE_MIN,REMOTE_MAX,ROLL_RATE_MIN, ROLL_RATE_MAX),_imu ->Gyro(X));
		_rateY -> controllerUpdate(map<int16_t>(_remote->getRemoteData(Pitch),REMOTE_MIN,REMOTE_MAX,PITCH_RATE_MIN, PITCH_RATE_MAX),_imu ->Gyro(Y));
	}
	_rateZ -> controllerUpdate(map<int16_t>(_remote->getRemoteData(Yaw),REMOTE_MIN,REMOTE_MAX,YAW_RATE_MIN, YAW_RATE_MAX),_imu ->Gyro(Z));
}

void FlightController::computeMotorOutputs() {
	int16_t throttle = map<int16_t>(_remote -> getRemoteData(Throttle),REMOTE_MIN,REMOTE_MAX,Throttle_Min,Throttle_Max);
	_motors -> setOutput(FrontRight, throttle - _rateX -> getOutput() - _rateY -> getOutput() + _rateZ -> getOutput());
	_motors -> setOutput(RearLeft, throttle + _rateX -> getOutput() + _rateY -> getOutput() + _rateZ -> getOutput());
	_motors -> setOutput(FrontLeft, throttle + _rateX -> getOutput() - _rateY -> getOutput() - _rateZ -> getOutput());
	_motors -> setOutput(RearRight, throttle - _rateX -> getOutput() + _rateY -> getOutput() - _rateZ -> getOutput());
}

bool FlightController::minThrottle() {
	return _remote ->getRemoteData(Throttle) >= Throttle_Min;
}

void FlightController::fcDebug(debug_group debug) {
	switch(debug){
	case ALL:
		debugArmStatus();
		_imu -> debugGyro();
		_imu -> debugYPR();
		_rateX -> debugPID("X");
		_rateY -> debugPID("Y");
		_rateZ -> debugPID("Z");
		if (_Fm == FC_ANGLE_MODE){
			_angleX -> debugPID("Roll Angle");
			_angleY -> debugPID("Pitch Angle");
		}
		_motors -> DebugMotors();
		break;
	case GYRO:
		_imu -> debugGyro();
		break;
	case YPR:
		_imu -> debugYPR();
		break;
	case ANGLES:
		_angleX -> debugPID("Roll Angle");
		_angleY -> debugPID("Pitch Angle");
		break;
	case PIDS:
		_rateX -> debugPID("X");
		_rateY -> debugPID("Y");
		_rateZ -> debugPID("Z");
		break;
	case MOTORS:
		_motors -> DebugMotors();
		break;
	case FC_STATUS:
		debugArmStatus();
		break;
	default:
		break;
	}
}

void FlightController::sendTelemetryData() {
	Telemetry* telemetry = Telemetry::getInstance();
	uint8_t gyrobuf [] = {77,static_cast<uint8_t>( _imu -> Gyro(X)),static_cast<uint8_t>(_imu -> Gyro(Y)),static_cast<uint8_t>(_imu -> Gyro(Z)),static_cast<uint8_t>(_imu -> Yaw()),static_cast<uint8_t>(_imu -> Pitch()),static_cast<uint8_t>(_imu -> Roll()),77};
	telemetry -> sendData(gyrobuf);
}

void FlightController::debugArmStatus() {
	Debug(printf("Arm status is:%s\r\n", isArmed()?"true": "false"));
}

void FlightController::initialisePIDs() {
	_rateX = new PID(0,0,0,0);
	_rateY = new PID(0,0,0,0);
	_rateZ = new PID(0,0,0,0);
	_angleX = new PID(0,0,0,0);
	_angleY = new PID(0,0,0,0);
}

void FlightController::sendTelemetryMessage(uint8_t message_code,
		uint8_t value) {
	Telemetry* telemetry = Telemetry::getInstance();
	uint8_t message_buf [] = {66,message_code,value,0,0,0,0 ,66};
	telemetry -> sendData(message_buf);
}

void FlightController::watchdog_enable(){
	HAL_TIM_Base_Start_IT(&htim3);
}

void FlightController::debugloop(int frequency, debug_group debug) {
	static long unsigned int start = HAL_GetTick();
	if(HAL_GetTick() - start > static_cast<long unsigned int>(1000/frequency)){
		fcDebug(debug);
		start = HAL_GetTick();
	}
}

void FlightController::telemetryDataLoop(int frequency) {
	static int start = HAL_GetTick();
	if(HAL_GetTick() - start > static_cast<long unsigned int>(1000/frequency)){
			sendTelemetryData();
			start = HAL_GetTick();
		}
}

void FlightController::watchdog_verify() {
  if (!watchdog_was_reset) {
	  HAL_TIM_Base_Stop_IT(&htim3);
	  watchdog_stop_var = true;
  }
  watchdog_was_reset = false;
}

void FlightController::watchdog_stop(){
	if (watchdog_stop_var){
	indicateEmergency(WATCHDOG_RESET);
	}
}

void FlightController::watchdog_reset(){
	watchdog_was_reset = true;
}

void FlightController::HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim3 )
  {
    watchdog_verify();
  }
}
