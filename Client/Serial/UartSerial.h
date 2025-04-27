/*
 * SerialStream.h
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */

#ifndef SERIAL_UARTSERIAL_H_
#define SERIAL_UARTSERIAL_H_
#include "Serial.h"

class UartSerial: public Serial {
public:
	UartSerial(UART_HandleTypeDef &huart2, osEventFlagsId_t &eventFlag) :
		Serial(eventFlag), _uart(&huart2) {
	}

protected:
	HAL_StatusTypeDef send(std::string payload) override;
	void processInputMsg(std::string) override;

private:
	const UART_HandleTypeDef *_uart;
};

#endif /* SERIAL_UARTSERIAL_H_ */
