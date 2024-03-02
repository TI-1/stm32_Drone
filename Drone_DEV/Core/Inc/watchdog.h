/*
 * watchdog.h
 *
 *  Created on: Feb 13, 2023
 *      Author: tobii
 */

#ifndef INC_WATCHDOG_H_
#define INC_WATCHDOG_H_
#include "tim.h"

void watchdog_enable();
void watchdog_reset();
void watchdog_stop();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);



#endif /* INC_WATCHDOG_H_ */
