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
#include <Mavlink/common/mavlink.h>
class Telemetry{
public:
    Telemetry(UART_HandleTypeDef *telem, uint32_t frequency);
    Telemetry(const Telemetry& t) = delete;
    Telemetry& operator=(const Telemetry &obj) = delete;
    void gatherData(IMU *imu);
    void update(); // Call this in the loop
    void sendMessage(uint8_t message_code, uint8_t value);
private:
    UART_HandleTypeDef *_telemHuart;
    uint32_t _lastSendTime;
    uint32_t _sendInterval; // Time in milliseconds
    uint32_t _startTime = HAL_GetTick();
    char gyrobuf[40];
    void sendData();
};

#endif /* SRC_TELEMETRY_H_ */
