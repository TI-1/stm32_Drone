/*
 * telemetry.cpp
 *
 *  Created on: Jan 4, 2024
 *      Author: tobii
 */
#include "telemetry.h"

Telemetry::Telemetry(UART_HandleTypeDef *telem, uint32_t frequency) {
	_telemHuart = telem;
	_sendInterval = 1000 / frequency;
	_lastSendTime = HAL_GetTick();
}

void Telemetry::gatherData(IMU *imu) {
	uint32_t dataTime = (HAL_GetTick() - _startTime) / 1000;
	sprintf(gyrobuf,"*%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d*"
			,imu->Gyro(X),imu->Gyro(Y),imu->Gyro(Z),imu->Yaw(),imu->Pitch(),imu->Roll(), (int)dataTime);
}

void Telemetry::sendMessage(uint8_t message_code, uint8_t value){
	char message_buf[40];
	sprintf(message_buf,"#%d,%d,%d,%d,%d,%d#",message_code, value, 0, 0, 0,0);
		HAL_UART_Transmit(_telemHuart, (uint8_t *)message_buf, sizeof(message_buf), 1000);
}

void Telemetry::update() {
	uint32_t currentTime = HAL_GetTick();
	if (currentTime - _lastSendTime >= _sendInterval){
		sendData();
		_lastSendTime = currentTime;
	}
}

void Telemetry::sendData(){
	HAL_UART_Transmit(_telemHuart, (uint8_t *)gyrobuf, sizeof(gyrobuf), 1000);
}
