#ifndef _IBUS_H_
#define _IBUS_H_

 #include "usart.h"
#include "stdint.h"



#define IBUS_USER_CHANNELS 6

/* Defines */
#define IBUS_LENGTH				0x20	// 32 bytes
#define IBUS_COMMAND40			0x40	// Command to set servo or motor speed is always 0x40
#define IBUS_MAX_CHANNLES		14

enum remote{
	Roll,
	Pitch,
	Throttle,
	Yaw
};


class Ibus
{
    public:
        Ibus(UART_HandleTypeDef* huart);
        bool ibusRead();
        uint16_t getRemoteData(remote rc);



    private:
        
        uint8_t m_uart_rx_buffer[IBUS_LENGTH] = {0};
        uint16_t _ibus_data[IBUS_USER_CHANNELS];
        UART_HandleTypeDef* m_huart;
        bool ibusIsValid();
        bool ibusChecksum();
        void ibusUpdate();
};
#endif
