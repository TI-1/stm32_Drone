#include "remote.h"
#include "stdio.h"


Remote::Remote(UART_HandleTypeDef* huart):m_huart(huart)
{
	HAL_UART_Receive_DMA(m_huart, m_uart_rx_buffer, 32);
}

bool Remote::ibusRead(){
    if(!ibusIsValid()) return false;

    if(!ibusChecksum()) return false;

    ibusUpdate();
    return true;
}

int16_t Remote::getRemoteData(remote rc) {
	if (ibusRead()){
		if (_ibus_data[Arm]!= 2000 || _ibus_data[Arm]!= 1000){
			signalLost = true;
		}
		_nullcount = 0;
		return _ibus_data[rc];
	}
	else {
		_nullcount += 1; //TODO may need to change error number
		if (_nullcount >= 10){
			signalLost = true;
		}
		return _ibus_data[rc];
	}
}



bool Remote::ibusIsValid(){
    return (m_uart_rx_buffer[0] == IBUS_LENGTH && m_uart_rx_buffer[1] == IBUS_COMMAND40);

}

bool Remote::ibusChecksum(){
    uint16_t checksum_cal = 0xffff;
    uint16_t checksum_ibus;

    for(int i=0; i<30; i++){

        checksum_cal -= m_uart_rx_buffer[i];
    }
    checksum_ibus = m_uart_rx_buffer[31] << 8 | m_uart_rx_buffer[30];
    return (checksum_ibus == checksum_cal);
}


void Remote::ibusUpdate(){
	for(int ch_idx=0, buf_idx = 2;ch_idx < IBUS_USER_CHANNELS; ch_idx++, buf_idx +=2){
		 _ibus_data[ch_idx] = m_uart_rx_buffer[buf_idx + 1] << 8 | m_uart_rx_buffer[buf_idx];
	}

}

void Remote::debugRemote() {
	printf("Throttle:%d\tYaw:%d\tPitch:%d\tRoll:%d\tArm:%d\r\n",getRemoteData(Throttle), getRemoteData(Yaw),getRemoteData(Pitch),getRemoteData(Roll),getRemoteData(Arm));
}
