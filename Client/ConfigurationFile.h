/*
 * ConfigurationFile.h
 *
 *  Created on: Apr 27, 2025
 *      Author: gdurand
 */

#ifndef CONFIGURATIONFILE_CONFIGURATIONFILE_H
#define CONFIGURATIONFILE_CONFIGURATIONFILE_H
#include "main.h"
#include "cmsis_os.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <deque>

#define ENCODER_CALLBACK_PERIOD	20	//in ms		//20	1000
#define MOTOR_CONTROL_CALLBACK_PERIOD 50	//in ms
#define DEFAULT_TTL_DURATION 100	//In ms

#define TWO_PI 	(double)(2.0 * M_PI)

typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
} GPIO;

const GPIO LED_ENCODER_TASK(GPIOA, GPIO_PIN_5);
const GPIO LED_MOTOR_TASK(GPIOA, GPIO_PIN_10);
const GPIO LED_INIT_ERROR(GPIOA, GPIO_PIN_9);
const GPIO LED_FREERTOS_ERROR(GPIOA, GPIO_PIN_8);
const GPIO LED_STM32_ERROR(GPIOC, GPIO_PIN_9);
const GPIO LED_USB_ERROR(GPIOC, GPIO_PIN_8);




//#define INCOMING_MSG_SIZE (uint32_t)50
//typedef uint8_t incomingMsg[INCOMING_MSG_SIZE];	//Create c array object

#endif /* CONFIGURATIONFILE_CONFIGURATIONFILE_H */
