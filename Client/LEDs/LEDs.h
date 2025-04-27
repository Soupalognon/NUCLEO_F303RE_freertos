/*
 * LEDs.h
 *
 *  Created on: Apr 27, 2025
 *      Author: gdurand
 */

#ifndef LEDS_LEDS_H_
#define LEDS_LEDS_H_
#include "ConfigurationFile.h"

class LEDs {
public:
	LEDs(const GPIO encoderTask, const GPIO motorTask, const GPIO initError, const GPIO freertosError,
		const GPIO stm32Error, const GPIO usbError) :
		_encoderTask(encoderTask), _motorTask(motorTask), _initError(initError), _freertosError(freertosError), _stm32Error(
			stm32Error), _usbError(usbError) {
		HAL_GPIO_WritePin(_encoderTask.port, _encoderTask.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(_motorTask.port, _motorTask.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(_initError.port, _initError.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(_freertosError.port, _freertosError.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(_stm32Error.port, _stm32Error.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(_usbError.port, _usbError.pin, GPIO_PIN_RESET);
	}

	void encoderTask();
	void motorTask();
	void initError();
	void freeRtosError();
	void stm32Error();
	void usbError();

protected:

private:
	const GPIO _encoderTask;
	const GPIO _motorTask;

	const GPIO _initError;
	const GPIO _freertosError;
	const GPIO _stm32Error;
	const GPIO _usbError;

};

#endif /* LEDS_LEDS_H_ */
