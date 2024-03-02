/*
 * telemetry.cpp
 *
 *  Created on: Jan 4, 2024
 *      Author: tobii
 */
#include "telemetry.h"


Telemetry* Telemetry::_telemetry = nullptr;
UART_HandleTypeDef* Telemetry::defaultUART = &huart2;

Telemetry::Telemetry():huart(*defaultUART) {}

Telemetry* Telemetry::getInstance() {
	if (_telemetry == nullptr){
		_telemetry = new Telemetry();
	}
	return _telemetry;

}


void Telemetry::sendData(uint8_t databuf[]) {
	HAL_UART_Transmit(defaultUART, databuf, 8, 1000);
}



