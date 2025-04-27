/*
 * UsbStream.h
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */

#ifndef SERIAL_USBSERIAL_H_
#define SERIAL_USBSERIAL_H_
#include <Serial/Serial.h>
#include "ConfigurationFile.h"
#include "usbd_cdc_if.h"

class UsbSerial: public Serial {
public:
	UsbSerial(USBD_HandleTypeDef& hUsbDeviceFS, osEventFlagsId_t &eventFlag) :
		Serial(eventFlag), _usb(&hUsbDeviceFS) {
	}

protected:
	HAL_StatusTypeDef send(std::string payload) override;
	void processInputMsg(std::string) override;
private:
	USBD_HandleTypeDef* _usb;
};

#endif /* SERIAL_USBSERIAL_H_ */
