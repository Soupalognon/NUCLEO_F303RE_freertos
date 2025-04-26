#include "main.h"
#include "cmsis_os.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <deque>

#define ENCODER_CALLBACK_PERIOD	20	//in ms		//20	//1000
#define MOTOR_CONTROL_CALLBACK_PERIOD 50	//in ms
#define DEFAULT_TTL_DURATION 100	//In ms

#define TWO_PI 	(double)(2.0 * M_PI)

//#define INCOMING_MSG_SIZE (uint32_t)50
//typedef uint8_t incomingMsg[INCOMING_MSG_SIZE];	//Create c array object
