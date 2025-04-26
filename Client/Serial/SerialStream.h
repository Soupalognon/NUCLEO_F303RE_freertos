/*
 * SerialStream.h
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */

#ifndef SERIAL_SERIALSTREAM_H_
#define SERIAL_SERIALSTREAM_H_
#include "ConfigurationFile.h"
#include "Stream.h"

class SerialStream: public Stream {
public:
	SerialStream(UART_HandleTypeDef &huart2, osEventFlagsId_t &eventFlag) :
		Stream(eventFlag), _serial(&huart2) {
	}

protected:
	HAL_StatusTypeDef send(std::string payload) override;
	void processInputMsg(std::string) override;

private:
	const UART_HandleTypeDef *_serial;
};

#endif /* SERIAL_SERIALSTREAM_H_ */
