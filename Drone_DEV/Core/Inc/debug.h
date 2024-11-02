/*
 * debug.h
 *
 *  Created on: Apr 19, 2023
 *      Author: tobii
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include "usart.h"
#include <stdio.h>
#include "gpio.h"


typedef enum {
	GYRO_FREEZE = 0,
	EXTREME_ANGLE,
	IMU_INIT,
	WATCHDOG_RESET
}emergency_stop;



void indicateEmergency(const char *reason);

void indicateEmergency(emergency_stop reason);

#endif /* INC_DEBUG_H_ */
