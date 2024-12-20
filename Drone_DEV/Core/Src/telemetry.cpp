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
	_startTime = HAL_GetTick();
	_lastSendTime = _startTime;
	HAL_UART_Receive_IT(telem, &_tempbuffer[0], 1);
}

void Telemetry::sendData(IMU *imu) {
	uint32_t currentTime = HAL_GetTick();
	if (currentTime - _lastSendTime >= _sendInterval){
		uint32_t dataTime = (HAL_GetTick() - _startTime);
		mavlink_message_t msg;
		mavlink_msg_attitude_pack(
			1, 200, &msg, dataTime,
			imu->Roll() * M_PI / 180, imu->Pitch() * M_PI / 180, imu->Yaw() * M_PI / 180,
			imu->Gyro(X) * M_PI / 180, imu->Gyro(Y) * M_PI / 180, imu->Gyro(Z) * M_PI / 180);
		SendMavLinkMessage(&msg);
		_lastSendTime = currentTime;
		}
}

void Telemetry::sendStatusText(const char* text, uint8_t severity) {
	mavlink_message_t msg;
	mavlink_msg_statustext_pack(1, 1, &msg, severity, text,1,0);
	SendMavLinkMessage(&msg);
}

void Telemetry::sendHeartbeat() {
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(1, 1, &msg,
		MAV_TYPE_QUADROTOR,  // Set type as needed
		MAV_AUTOPILOT_GENERIC,
		MAV_MODE_MANUAL_ARMED,
		0,                   // Custom mode, can define modes here
		_state);   // System state
	SendMavLinkMessage(&msg);
}


void Telemetry::SendMavLinkMessage(mavlink_message_t *msg) {
	uint16_t len = mavlink_msg_to_send_buffer(_mavlink_buffer, msg);

	if (_txBusy) {
		for (uint16_t i = 0; i < len; i++) {
			_txBuffer[_txBufferHead] = _mavlink_buffer[i];
			_txBufferHead = (_txBufferHead + 1) % TELEMETRY_TX_BUFFER_SIZE;

			if (_txBufferHead == _txBufferTail) {
				_txBufferTail = (_txBufferTail + 1) % TELEMETRY_TX_BUFFER_SIZE; // Handle overflow
			}
		}
		return;
	}
	_txBusy = true;
	HAL_UART_Transmit_DMA(_telemHuart, _mavlink_buffer, len);
}

void Telemetry::onTxComplete() {
	if (_txBufferTail != _txBufferHead) {
		uint16_t len = 0;
		while (_txBufferTail != _txBufferHead && len < MAVLINK_MAX_PACKET_LEN) {
			_mavlink_buffer[len++] = _txBuffer[_txBufferTail];
			_txBufferTail = (_txBufferTail + 1) % TELEMETRY_TX_BUFFER_SIZE;
		}
		HAL_UART_Transmit_DMA(_telemHuart, _mavlink_buffer, len);
	}
	else {
		_txBusy = false;
		HAL_UART_DMAStop(_telemHuart);
	}
}



void Telemetry::setState(MAV_STATE state) {
	_state = state;
}

UART_HandleTypeDef* Telemetry::getUartHandle() {
	return _telemHuart;
}

void Telemetry::onReceiveComplete() {
	_recievebuffer[_writeIndex] = _tempbuffer[0];
	_writeIndex = (_writeIndex + 1) % (MAVLINK_MAX_PACKET_LEN);

	if(_writeIndex == _readIndex){
		_readIndex = (_readIndex + 1) % (MAVLINK_MAX_PACKET_LEN);
	}
	HAL_UART_Receive_IT(_telemHuart, _tempbuffer , 1);
}

void Telemetry::processIncomingData(mavlink_param_set_t &paramSet) {
	mavlink_message_t msg;
	mavlink_status_t status;

	while (_readIndex != _writeIndex){
		uint8_t byte = _recievebuffer[_readIndex];
		_readIndex = (_readIndex + 1) % (MAVLINK_MAX_PACKET_LEN);

		if(mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)){
			switch(msg.msgid){
				case MAVLINK_MSG_ID_PARAM_SET:
					// Example handling for PARAM_SET
					mavlink_msg_param_set_decode(&msg, &paramSet);
					mavlink_msg_param_value_pack(paramSet.target_system,
							paramSet.target_component,
							&msg,
							paramSet.param_id,
							paramSet.param_value,
							paramSet.param_type,
							1, 1);
					SendMavLinkMessage(&msg);
					return;
					break;
				default:
					break;
			}

		}
	}
	paramSet.target_system = 0;
}
