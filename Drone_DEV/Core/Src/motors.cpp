/*
 * motors.cpp
 *
 *  Created on: 5 May 2023
 *      Author: tobii
 */

#include "motors.h"

/**
 * Constructor for Motor class
 * @param pwm_timer set which timer motors use
 */
Motors::Motors(TIM_HandleTypeDef *pwm_timer):_pwmTimer(pwm_timer),_counterPeriod( _pwmTimer ? _pwmTimer -> Instance -> ARR : 0) {
	if (!_pwmTimer) {
		printf("Error: PWM Timer is null\n");
	}
}

void Motors::initialise(){
	HAL_TIM_PWM_Start(_pwmTimer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(_pwmTimer, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(_pwmTimer, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(_pwmTimer, TIM_CHANNEL_4);
}

/**
 * set Motor output level
 * @param pos The position of the motor on the drone
 * @param output The motor output level
 */
void Motors::setOutput(motorPosition pos, uint16_t output) {
	if (pos >= 0 && pos < 4) {
		_output[pos] = output;
	}
	else {
		printf("Error: Invaleid motor position\n");
	}	
}

/**
 * get Motor output level
 * @param pos The position of the motor on the drone
 * @return The motor output level
 */
uint16_t Motors::getOutput(motorPosition pos) {
	return (pos >= 0 && pos < 4) ? _output[pos] : 0;
}

/**
 * Pushes the set motor output to the motors
 * @param state Turns the motors on or off
 */
void Motors::command(state state) {
	if (state == state::On){
		set_registers();
	}
	else {
		zeroOutput();
		set_registers();

	}
}

//TODO figure out motor calibration
void Motors::callibrate() {
	// Calibration logic not yet implemented
}

/**
 * Sets timer registers to produce correct output PWM
 */
void Motors::set_registers() {
	_pwmTimer -> Instance -> CCR1 = mapMotors(_output[FrontRight]);
	_pwmTimer -> Instance -> CCR2 = mapMotors(_output[RearLeft]);
	_pwmTimer -> Instance -> CCR3 = mapMotors(_output[FrontLeft]);
	_pwmTimer -> Instance -> CCR4 = mapMotors(_output[RearRight]);
}

/**
 * Utility function to map RC values to correct values for motor PWM
 * @param value input value to be converted (should be between 2000 and 1000)
 * @return Mapped output
 */
uint16_t Motors::mapMotors(uint16_t value) {
	// Clamp the input value to the valid range
	if (value > _maxRCInput) {
		return _counterPeriod * _maxDutyCycle;
	}
	if (value < _minRCInput) {
		return _counterPeriod * _minDutyCycle;
	}

	// Map the input value from RC input range to duty cycle range
	const uint16_t rcRange = _maxRCInput - _minRCInput;
	const float dutyCycleRange = _maxDutyCycle - _minDutyCycle;

	return ((value - _minRCInput) * (_counterPeriod * dutyCycleRange) / rcRange) +
		   (_counterPeriod * _minDutyCycle);
}

/**
 * Set minimum PWM duty cycle
 * @param dutyCycle
 */
void Motors::setMinDutyCycle(float dutyCycle) {
	_minDutyCycle = dutyCycle;
}

/**
 * set maximum PWM duty cycle
 * @param dutyCycle
 */
void Motors::setMaxDutyCycle(float dutyCycle) {
	_maxDutyCycle = dutyCycle;
}

void Motors::DebugMotors() {
	printf("m1_out:%d\tm2_out:%d\tm3_out:%d\tm4_out:%d\r\n",getOutput(FrontLeft),getOutput(FrontRight),getOutput(RearLeft),getOutput(RearRight));
}

/**
 * Zero motor outputs
 */
void Motors::zeroOutput() {
	for (uint8_t i = 0; i < sizeof(_output)/  sizeof(_output[0]); i++){
	        _output[i] = _minRCInput ;
	    }
}
