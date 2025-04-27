/*
 * Stream.h
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */

#ifndef SERIAL_SERIAL_H_
#define SERIAL_SERIAL_H_

#include "ConfigurationFile.h"

class Serial {
public:
	Serial(osEventFlagsId_t &eventFlag) :
		_eventFlag(&eventFlag) {
		_mutex = osMutexNew(nullptr);
	}

	void handler();

	void addError(std::string payload) {
		osMutexAcquire(_mutex, osWaitForever);

		payload = "ERROR: " + payload;
		queueOutError.push_back(payload);
		osEventFlagsSet(*_eventFlag, 0x01);

		osMutexRelease(_mutex);
	}
	void addWarning(std::string payload) {
		osMutexAcquire(_mutex, osWaitForever);

		payload = "WARNING: " + payload;
		queueOutWarning.push_back(payload);
		osEventFlagsSet(*_eventFlag, 0x01);

		osMutexRelease(_mutex);
	}
	void addInfo(std::string payload) {
		osMutexAcquire(_mutex, osWaitForever);

		payload = "INFO: " + payload;
		queueOutInfo.push_back(payload);
		osEventFlagsSet(*_eventFlag, 0x01);

		osMutexRelease(_mutex);
	}

	void addErrorLn(std::string payload) {
		payload = payload + "\n";
		addError(payload);
	}
	void addWarningLn(std::string payload) {
		payload = payload + "\n";
		addWarning(payload);
	}
	void addInfoLn(std::string payload) {
		payload = payload + "\n";
		addInfo(payload);
	}

	bool saveInputMsg(uint8_t *data, uint8_t len);

protected:
	virtual HAL_StatusTypeDef send(std::string payload) = 0;
	virtual void processInputMsg(std::string) = 0;

	std::string _outputBuffer;

private:
	osMutexId_t _mutex;
	uint32_t _timer = 0;
	uint16_t _bufferItr = 0;
	static const uint8_t BUFFER_SIZE = 50;
	uint8_t _inputBuffer[BUFFER_SIZE] = { };

	std::deque<std::string> queueIn;

	std::deque<std::string> queueOutError;
	std::deque<std::string> queueOutWarning;
	std::deque<std::string> queueOutInfo;

	const osSemaphoreId_t *_eventFlag;

	void clearBuffer() {
		_bufferItr = 0;
		memset(_inputBuffer, 0, sizeof(_inputBuffer));
	}
};

#endif /* SERIAL_SERIAL_H_ */
