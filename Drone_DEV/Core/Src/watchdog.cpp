/*
 * watchdog.cpp
 *
 *  Created on: Feb 13, 2023
 *      Author: tobii
 */

#include "debug.h"
#include "watchdog.h"

//TODO rewrite methods accounting for camelcase
volatile bool watchdog_was_reset = true;
volatile bool watchdog_stop_var = false;


void watchdog_enable(){
	HAL_TIM_Base_Start_IT(&htim3);
}

void watchdog_verify() {
  if (!watchdog_was_reset) {
	  HAL_TIM_Base_Stop_IT(&htim3);
	  watchdog_stop_var = true;
  }
  watchdog_was_reset = false;
}

void watchdog_stop(){
	if (watchdog_stop_var){
	indicateEmergency(WATCHDOG_RESET);
	}
}

void watchdog_reset(){
	watchdog_was_reset = true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim3 )
  {
    watchdog_verify();
  }
}







