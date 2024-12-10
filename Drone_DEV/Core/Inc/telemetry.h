/*
 * telemetry.h
 *
 *  Created on: Jan 4, 2024
 *      Author: tobii
 */

#ifndef SRC_TELEMETRY_H_
#define SRC_TELEMETRY_H_
#include "stdint.h"
#include <stdio.h>
#include "usart.h"
#include "imu.h"
#include "mavlink.h"

class Telemetry{
public:
    Telemetry(UART_HandleTypeDef *telem, uint32_t frequency);
    Telemetry(const Telemetry& t) = delete;
    Telemetry& operator=(const Telemetry &obj) = delete;
    void sendData(IMU *imu);
    void sendMessage(uint8_t message_code, uint8_t value);
    void SendMavLinkMessage(mavlink_message_t *msg);
    void setState(MAV_STATE state);
    void sendHeartbeat();
    void sendStatusText(const char* text, uint8_t severity);
    UART_HandleTypeDef* getUartHandle();
    void onReceiveComplete();
    void processIncomingData(mavlink_param_set_t &paramSet);
private:
    UART_HandleTypeDef *_telemHuart;
    MAV_STATE _state = MAV_STATE_UNINIT;
    uint32_t _lastSendTime;
    uint32_t _sendInterval; // Time in milliseconds
    uint32_t _startTime;
    uint8_t _mavlink_buffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t _recievebuffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t _tempbuffer[2];
    uint16_t _writeIndex = 0;
    uint16_t _readIndex = 0;
    char _gyrobuf[100];

};

#endif /* SRC_TELEMETRY_H_ */
