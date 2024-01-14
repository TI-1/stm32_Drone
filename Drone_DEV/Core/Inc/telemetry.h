/*
 * telemetry.h
 *
 *  Created on: Jan 4, 2024
 *      Author: tobii
 */

#ifndef SRC_TELEMETRY_H_
#define SRC_TELEMETRY_H_
#include "stdint.h"
#include "usart.h"
class Telemetry{
	public:
		static Telemetry *getInstance();
		Telemetry(const Telemetry&) = delete;
		Telemetry& operator=(const Telemetry&) = delete;
		void sendData(uint8_t databuf[]);
		static void setDefaultUART(UART_HandleTypeDef& uartHandle);

	private:
		static Telemetry* _telemetry;
		Telemetry();
		~Telemetry() {}
		 UART_HandleTypeDef& huart;
		 static UART_HandleTypeDef* defaultUART;


};

#endif /* SRC_TELEMETRY_H_ */
