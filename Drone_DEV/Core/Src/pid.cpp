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
PID::PID(float kp, float ki, float kd, float imax):_kp(kp),_ki(ki),_kd(kd),_imax(imax)
{
	_lastCompute = HAL_GetTick();
}

/**
 * Method to calculate PID controller output
 * @param setpoint Set value
 * @param input Actual value measured from sensors
 */
void PID::controllerUpdate(float setpoint, float input) {
	float dt = (HAL_GetTick()-_lastCompute)/1000.0f;
	float error = setpoint - input;
	_integrator += _ki * error * dt;
	_integrator = ((_integrator)<(-_imax)?(-_imax):((_integrator)>(_imax)?(_imax):(_integrator)));
	float p_out = error * _kp;
	float i_out = _integrator;

	if (derivative_mode == doe){
		float d_out = _kd * (error - _lastError) / dt;
		_controllerOutput = p_out + i_out + d_out;

		}

	else {
		float d_out = _kd * (input - _lastInput) / dt;
		_controllerOutput = p_out + i_out - d_out;
	}

	_lastError = error;
	_lastInput = input;

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
