#include "remote.h"
#include "stdio.h"

/**
 * Remote Class Constructor
 * @param huart pointer to UART
 */
Remote::Remote(UART_HandleTypeDef *huart) :
		_huart(huart) {

}

Remote::Remote() :
		_huart(nullptr) {

}

void Remote::initialise(){
	HAL_UART_Receive_DMA(_huart, _uartRxBuffer, 32);
}

/**
 * Read ibus data
 * @return true if data read was successful
 */
bool Remote::ibusRead() {
	if (!ibusIsValid())
		return false;

	if (!ibusChecksum())
		return false;

	ibusUpdate();
	return true;
}

/**
 * Get remote values for specific channel
 * @param rc channel to get value from
 * @return data value from channel
 */
int16_t Remote::getRemoteData(remote rc) {
	if (ibusRead()) {
		if (_ibusData[Arm] != 2000 || _ibusData[Arm] != 1000) {
			signalLost = true;
		}
		_nullcount = 0;
		return _ibusData[rc];
	} else {
		_nullcount += 1; //TODO may need to change error number
		if (_nullcount >= 10) {
			signalLost = true;
		}
		return _ibusData[rc];
	}
}

/**
 * Check if ibus data is valid
 * @return true if data is valid
 */
bool Remote::ibusIsValid() {
	return (_uartRxBuffer[0] == IBUS_LENGTH
			&& _uartRxBuffer[1] == IBUS_COMMAND40);

}

/**
 * checksum for ibus data
 * @return true if checksum is valid
 */
bool Remote::ibusChecksum() {
	uint16_t checksum_cal = 0xffff;
	uint16_t checksum_ibus;

	for (int i = 0; i < 30; i++) {

		checksum_cal -= _uartRxBuffer[i];
	}
	checksum_ibus = _uartRxBuffer[31] << 8 | _uartRxBuffer[30];
	return (checksum_ibus == checksum_cal);
}

/**
 * Update internal data buffer with new data
 */
void Remote::ibusUpdate() {
	for (int ch_idx = 0, buf_idx = 2; ch_idx < IBUS_USER_CHANNELS;
			ch_idx++, buf_idx += 2) {
		_ibusData[ch_idx] = _uartRxBuffer[buf_idx + 1] << 8
				| _uartRxBuffer[buf_idx];
	}

}

/**
 * debug remote
 */
void Remote::debugRemote() {
	printf("Throttle:%d\tYaw:%d\tPitch:%d\tRoll:%d\tArm:%d\r\n",
			getRemoteData(Throttle), getRemoteData(Yaw), getRemoteData(Pitch),
			getRemoteData(Roll), getRemoteData(Arm));
}
