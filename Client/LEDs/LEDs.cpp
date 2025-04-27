/*
 * LEDs.cpp
 *
 *  Created on: Apr 27, 2025
 *      Author: gdurand
 */
#include "LEDs.h"

void LEDs::encoderTask() {
	HAL_GPIO_TogglePin(_encoderTask.port, _encoderTask.pin);
}

void LEDs::motorTask() {
	HAL_GPIO_WritePin(_motorTask.port, _motorTask.pin, GPIO_PIN_SET);
}

void LEDs::initError() {
	HAL_GPIO_WritePin(_initError.port, _initError.pin, GPIO_PIN_SET);
}

void LEDs::freeRtosError() {
	HAL_GPIO_WritePin(_freertosError.port, _freertosError.pin, GPIO_PIN_SET);
}

void LEDs::stm32Error() {
	HAL_GPIO_WritePin(_stm32Error.port, _stm32Error.pin, GPIO_PIN_SET);
}

void LEDs::usbError() {
	HAL_GPIO_WritePin(_usbError.port, _usbError.pin, GPIO_PIN_SET);
}
