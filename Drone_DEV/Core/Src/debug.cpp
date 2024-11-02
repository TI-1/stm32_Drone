/*
 * debug.cpp
 *
 *  Created on: Jan 6, 2024
 *      Author: tobii
 */
#include "debug.h"

void indicateEmergency(const char *reason){
	printf("EMERGENCY STOP: %s\n", reason);
}

void indicateEmergency(emergency_stop reason){
	switch(reason){
	case GYRO_FREEZE:
		while(1){
#ifdef DEBUG
			printf("EMERGENCY STOP: GYRO FREEZE\n");
#endif
			HAL_GPIO_TogglePin(RedLED_GPIO_Port, RedLED_Pin);
			HAL_Delay(500);
		}
		break;
	case EXTREME_ANGLE:
			while(1){
#ifdef DEBUG
				printf("EMERGENCY STOP: ANGLE EXCEEDS LIMIT\n");
#endif
				HAL_GPIO_TogglePin(RedLED_GPIO_Port, RedLED_Pin);
				HAL_Delay(2000);
			}
			break;
	case IMU_INIT:
			while(1){
#ifdef DEBUG
				printf("EMERGENCY STOP: IMU INITIALISATION FAILED\n");
#endif
				HAL_GPIO_WritePin(RedLED_GPIO_Port, RedLED_Pin, GPIO_PIN_SET);
				HAL_Delay(3000);
			}
			break;
	case WATCHDOG_RESET:
				while(1){
#ifdef DEBUG
					printf("EMERGENCY STOP: FAILED TO RESET WATCHDOG\n");
#endif
					HAL_GPIO_TogglePin(RedLED_GPIO_Port, RedLED_Pin);
					HAL_Delay(5000);
				}
				break;
	default:
		while(1){
#ifdef DEBUG
			printf("EMERGENCY STOP\n");
#endif
		}

	}
}



