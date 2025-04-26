/*
 * UsbStream.cpp
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */
#include "UsbStream.h"
#include "usbd_cdc_if.h"

HAL_StatusTypeDef UsbStream::send(std::string payload) {
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

void UsbStream::processInputMsg(std::string payload) {
	addInfo(payload);
}
