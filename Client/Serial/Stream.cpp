/*
 * Stream.cpp
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */
#include "Stream.h"

void Stream::handler() {
	if (queueIn.size() > 0) {
		processInputMsg(queueIn.front());
		queueIn.pop_front();
	} else if (queueOutError.size() > 0) {
		if (send(queueOutError.front()) == HAL_OK) {
			queueOutError.pop_front();
		}
	} else if (queueOutWarning.size() > 0) {
		if (send(queueOutWarning.front()) == HAL_OK) {
			queueOutWarning.pop_front();
		}
	} else if (queueOutInfo.size() > 0) {
		if (send(queueOutInfo.front()) == HAL_OK) {
			queueOutInfo.pop_front();
		}
	} else {
		osEventFlagsClear(*_eventFlag, 0x01);
	}
}

bool Stream::saveInputMsg(uint8_t *data, uint8_t len) {
	if (HAL_GetTick() - _timer > 100) {
		clearBuffer();
	}
	_timer = HAL_GetTick();

	if (len < (BUFFER_SIZE - _bufferItr)) {
		memcpy(_inputBuffer + _bufferItr, data, len);
		_bufferItr += len;
	} else {
		addError("Message received too long. Trash it");
		clearBuffer();
	}

	if (_inputBuffer[_bufferItr - 1] == '\n') {
		queueIn.push_back((char*) _inputBuffer);
		osEventFlagsSet(*_eventFlag, 0x01);
		clearBuffer();

//		if (status == osOK) {
//			clearBuffer();
//		} else {
//			//ERROR MSG
//		}

		return true;
	}

	return false;
}
