/*
 * UsbStream.h
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */

#ifndef SERIAL_USBSTREAM_H_
#define SERIAL_USBSTREAM_H_
#include "ConfigurationFile.h"
#include "Stream.h"
#include "usbd_cdc_if.h"

class UsbStream: public Stream {
public:
	UsbStream(USBD_HandleTypeDef& hUsbDeviceFS, osEventFlagsId_t &eventFlag) :
		Stream(eventFlag), _usb(&hUsbDeviceFS) {
	}

protected:
	HAL_StatusTypeDef send(std::string payload) override;
	void processInputMsg(std::string) override;
private:
	USBD_HandleTypeDef* _usb;
};

#endif /* SERIAL_USBSTREAM_H_ */
