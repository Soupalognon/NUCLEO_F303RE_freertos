/*
 * SerialStream.cpp
 *
 *  Created on: Apr 26, 2025
 *      Author: gdurand
 */
#include "SerialStream.h"

HAL_StatusTypeDef SerialStream::send(std::string payload) {
	if(_serial->gState == HAL_UART_STATE_READY) {
		_outputBuffer = payload;
		HAL_StatusTypeDef status = HAL_UART_Transmit_IT((UART_HandleTypeDef*) _serial,
			reinterpret_cast<uint8_t*>(&_outputBuffer[0]), _outputBuffer.size());

		return status;
	}
	return HAL_BUSY;
}

void SerialStream::processInputMsg(std::string payload) {
	addInfo(payload);
}
