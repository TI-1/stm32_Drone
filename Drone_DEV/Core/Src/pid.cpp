/*
 * pid.cpp
 *
 *  Created on: 8 May 2023
 *      Author: tobii
 */
#include "pid.h"
#include "stm32f1xx_hal.h"

/**
 * Constructor for PID call
 * @param kp Proportional constant
 * @param ki Integral constant
 * @param kd Derivative constant
 * @param imax Integral max value
 */
PID::PID(float kp, float ki, float kd, float imax) :
		_kp(kp), _ki(ki), _kd(kd), _imax(imax) {
	_lastCompute = HAL_GetTick();
}

PID::PID(){
	_kp = 0.0;
	_ki = 0.0;
	_kd = 0.0;
	_imax = 0.0;
	_lastCompute = HAL_GetTick();
}



/**
 * Method to calculate PID controller output
 * @param setpoint Set value
 * @param input Actual value measured from sensors
 */
void PID::controllerUpdate(float setpoint, float input) {
	_setpoint = setpoint;
	_input = input;
	float dt = (HAL_GetTick() - _lastCompute) / 1000.0f;
	_dt = dt;
	float error = setpoint - input;
	_integrator += _ki * error * dt;
	_integrator = (
			(_integrator) < (-_imax) ?
					(-_imax) :
					((_integrator) > (_imax) ? (_imax) : (_integrator)));
	float p_out = error * _kp;
	float i_out = _integrator;

	if (derivative_mode == doe) {
		float d_out = _kd * (error - _lastError) / dt;
		_controllerOutput = p_out + i_out + d_out;
	}

	else {
		float d_out = _kd * (input - _lastInput) / dt;
		_controllerOutput = p_out + i_out - d_out;
	}

	_lastError = error;
	_lastInput = input;
	_lastCompute = HAL_GetTick();

}

/**
 * Method to rest integrated value
 */
void PID::reset_i() {
	_integrator = 0.0;
}

/**
 * Method to set derivative mode to derivative on measurement
 */
void PID::setDOM() {
	derivative_mode = dom;
}

/**
 * Method to set derivative mode to derivative on error
 */
void PID::setDOE() {
	derivative_mode = doe;
}

/**
 * Method to get PID controller output
 * @return output from PID calculation
 */
float PID::getOutput() {
	return _controllerOutput;
}

/**
 * Method to set proportional constant
 * @param kp proportional constant
 */
void PID::setkp(float kp) {
	_kp = kp;

}

/**
 * Method to set integral constant
 * @param ki integral constant
 */
void PID::setki(float ki) {
	_ki = ki;
}

/**
 * Method to set derivative constant
 * @param kd derivative constant
 */
void PID::setkd(float kd) {
	_kd = kd;
}

/**
 * Method to get current proportional constant
 * @return proportional constant
 */
float PID::getkp() {
	return _kp;
}

/**
 * Method to get current integral constant
 * @return integral constant
 */
float PID::getki() {
	return _ki;
}

/**
 * Method to get current derivative constant
 * @return derivative constant
 */
float PID::getkd() {
	return _kd;
}

void PID::debugPID(const char *axis) {
	printf("%s-set:%3.3f\tinput:%3.3f\terror:%3.3f\tout:%3.3f\tdt:%3.3f\r\n",
			axis, _setpoint, _input, _setpoint - _input, _controllerOutput,
			_dt);
}
