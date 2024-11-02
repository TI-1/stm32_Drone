/*
 * motors.h
 *
 *  Created on: 5 May 2023
 *      Author: tobii
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stdint.h"
#include "stm32f1xx.h"
#include "usart.h"
#include <stdio.h>

enum motorPosition {
	FrontLeft, FrontRight, RearLeft, RearRight
};

enum state {
	Off, On
};
class Motors {
public:
	Motors(TIM_HandleTypeDef *pwm_timer);
	Motors(const Motors &obj) = delete;
	Motors() = default;
	Motors& operator=(const Motors &obj) = delete;
	virtual ~Motors() {
		_pwmTimer = nullptr;
	}
	virtual void initialise();
	virtual void setOutput(motorPosition pos, uint16_t output);
	virtual uint16_t getOutput(motorPosition pos);
	virtual void command(state state);
	void callibrate();
	void setMinDutyCycle(float dutyCycle);
	void setMaxDutyCycle(float dutyCycle);
	void DebugMotors();

private:
	uint16_t _output[4];
	TIM_HandleTypeDef *_pwmTimer = nullptr;
	uint16_t _counterPeriod = 0;
	float _maxDutyCycle = 0.1;
	float _minDutyCycle = 0.05;
	uint16_t _maxRCInput = 2000;
	uint16_t _minRCInput = 1000;

	void set_registers();
	uint16_t mapMotors(uint16_t value);
	void zeroOutput();

};



#endif /* INC_MOTORS_H_ */
