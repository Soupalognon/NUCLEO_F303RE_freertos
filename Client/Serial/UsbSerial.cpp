/*
 * UsbStream.cpp
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */
#include <Serial/UsbSerial.h>
#include "usbd_cdc_if.h"

HAL_StatusTypeDef UsbSerial::send(std::string payload) {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) _usb->pClassData;
	if (hcdc->TxState == 0) {
		_outputBuffer = payload;
		uint8_t status = CDC_Transmit_FS(reinterpret_cast<uint8_t*>(&_outputBuffer[0]), _outputBuffer.size());

		if (status == USBD_OK)
			return HAL_OK;
		return HAL_ERROR;
	}
	return HAL_BUSY;
}

void UsbSerial::processInputMsg(std::string payload) {
	addInfo(payload);
}
