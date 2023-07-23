/*
 * pid.h
 *
 *  Created on: 8 May 2023
 *      Author: tobii
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stdint.h"

enum derivative
{
	dom,
	doe
};

class PID {

public:
	PID(float kp, float ki, float kd, float imax);
	void controllerUpdate(float setpoint, float input);
	void reset_i();
	void setDOM();
	void setDOE();
	float getOutput();


private:
	float _kp;
	float _ki;
	float _kd;
	float _integrator = 0.0;
	float _imax;
	float _lastError=0.0;
	float _lastInput = 0.0;
	uint32_t _lastCompute;
	float _delta = 0.0;
	float _controllerOutput = 0.0;
	derivative derivative_mode = doe;

};



#endif /* INC_PID_H_ */
