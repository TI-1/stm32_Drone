#ifndef _REMOTE_H_
#define _REMOTE_H_

#include "usart.h"
#include "stdint.h"

#define IBUS_USER_CHANNELS 6

/* Defines */
#define IBUS_LENGTH				0x20	// 32 bytes
#define IBUS_COMMAND40			0x40	// Command to set servo or motor speed is always 0x40
#define IBUS_MAX_CHANNLES		14
#define REMOTE_MAX 2000
#define REMOTE_MIN 1000

enum remote {
	Roll, Pitch, Throttle, Yaw, Arm = 5
};

class Remote {
public:
	Remote(UART_HandleTypeDef *huart);
	Remote();
	Remote(const Remote &obj) = delete;
	Remote& operator=(const Remote &obj) = delete;
	virtual int16_t getRemoteData(remote rc);
	void debugRemote();
	virtual ~Remote(){_huart = nullptr;};
	virtual void initialise();
public:
	bool signalLost = false;

private:

	uint8_t _uartRxBuffer[IBUS_LENGTH] = { 0 };
	uint16_t _ibusData[IBUS_USER_CHANNELS];
	UART_HandleTypeDef *_huart;
	int8_t _nullcount = 0;

private:
	bool ibusIsValid();
	bool ibusChecksum();
	void ibusUpdate();
	bool ibusRead();
};
#endif
