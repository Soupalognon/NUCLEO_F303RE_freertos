#include <Serial/UartSerial.h>
#include <Serial/UsbSerial.h>
#include "ConfigurationFile.h"
#include "usbd_cdc_if.h"

#include "ClosedControlLoop/ClosedControlLoop.h"
#include "MotorControl/MotorControl.h"
#include "LEDs/LEDs.h"
#include "FreertosRealTimeStats/FreertosRealTimeStats.h"

/*********************************/
//External variables
extern TIM_HandleTypeDef htim1;	//Motor PWM
extern TIM_HandleTypeDef htim3;	//Encoder QEI
extern TIM_HandleTypeDef htim4;	//Encoder QEI

extern UART_HandleTypeDef huart2;	//UART Debug
extern USBD_HandleTypeDef hUsbDeviceFS;	//USB Brain computer

extern osEventFlagsId_t debugEventHandle;
extern osEventFlagsId_t usbEventHandle;
extern osTimerId_t EncoderTaskHandle;
extern osTimerId_t MotorTaskHandle;
/*********************************/

/*********************************/
//General objects
UsbSerial usbStream(hUsbDeviceFS, usbEventHandle);
UartSerial uartStream(huart2, debugEventHandle);

ClosedControlLoop closedLoopControl;
MotorControl motorControl(htim1);

LEDs leds(LED_ENCODER_TASK, LED_MOTOR_TASK, LED_INIT_ERROR, LED_FREERTOS_ERROR, LED_STM32_ERROR,
	LED_USB_ERROR);
/*********************************/

uint8_t rxData[1];

poseStruct currentPos = { 0.0, 0.0, 0.0, 0 };
velocityStruct currentVelocity = { 0.0, 0.0, 0 };

void setup() {
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);	//Left Encoder
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);	//Right Encoder

	HAL_UART_Receive_IT(&huart2, rxData, 1);

//	osTimerStart(EncoderTaskHandle, ENCODER_CALLBACK_PERIOD);
//	osTimerStart(MotorTaskHandle, MOTOR_CONTROL_CALLBACK_PERIOD);
}

void StartBrainComTaskCpp(void *argument) {

	for (;;) {
		osDelay(1000);

		//		if (print_real_time_stats(pdMS_TO_TICKS(1000)) == STM32_OK) {
		//			uartStream.addInfo("Real time stats obtained\n");
		//		} else {
		//			uartStream.addInfo("Error getting real time stats\n");
		//		}
	}
}

void StartDebugComTaskCpp(void *argument) {

	for (;;) {
		osEventFlagsWait(debugEventHandle, 0x01, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
		uartStream.handler();
	}
}

void StartUsbComTaskCpp(void *argument) {

	for (;;) {
		osEventFlagsWait(usbEventHandle, 0x01, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
		usbStream.handler();
	}
}

void EncoderTaskCallbackCpp(void *argument) {
	static uint32_t lastTime = 0;
	uint32_t time = HAL_GetTick();
	uint32_t elapsed = time - lastTime;
	lastTime = time;

	static int16_t previousQeiValueLeft = 0, previousQeiValueRight = 0;
	const static double dt = ENCODER_CALLBACK_PERIOD / 1000.0;

	int16_t currentQeiValueLeft = (int16_t) TIM3->CNT;
	int16_t currentQeiValueRight = (int16_t) TIM4->CNT;

	printf("%d, %d\n", (currentQeiValueLeft - previousQeiValueLeft),
		(currentQeiValueRight - previousQeiValueRight));

	closedLoopControl.updatePositionAndVelocityDiffDrive(&currentPos, &currentVelocity,
		(currentQeiValueLeft - previousQeiValueLeft),
		(currentQeiValueRight - previousQeiValueRight), dt);

	previousQeiValueLeft = currentQeiValueLeft;
	previousQeiValueRight = currentQeiValueRight;

	uint32_t duration = HAL_GetTick() - time;
	uartStream.addInfoLn(
		"Encoder - duration=" + std::to_string(duration) + "ms / period=" + std::to_string(elapsed)
			+ "ms");

	leds.encoderTask();

	HAL_Delay(5);
}

void MotorTaskCallbackCpp(void *argument) {
	//	serialStream.addInfo("Test2\n");
	leds.motorTask();
}

/**********************************************************************/
//UART and USB RX callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		uartStream.saveInputMsg(rxData, 1);
		HAL_UART_Receive_IT(&huart2, rxData, 1);
	}
}

void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len) {
	usbStream.saveInputMsg(Buf, Len);
}

/**********************************************************************/
