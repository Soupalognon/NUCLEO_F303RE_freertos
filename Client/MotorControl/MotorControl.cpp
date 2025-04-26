/*
 * MotorControl.cpp
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */
#include "MotorControl.h"

void MotorControl::SetPwm(motorSelectionEnum selection, motorDirectionEnum direction, uint8_t percent,
	bool pwmNeedToStart) {
	const static uint16_t MAX_PWM_VALUE = _pwm->Init.Period;
	uint32_t TIM_CHANNEL, TIM_CHANNEL_N;

	uint32_t value = percent * MAX_PWM_VALUE / 100;

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (selection == LEFT) {
		TIM_CHANNEL = TIM_CHANNEL_1;
		TIM_CHANNEL_N = TIM_CHANNEL_2;
	} else {
		TIM_CHANNEL = TIM_CHANNEL_3;
		TIM_CHANNEL_N = TIM_CHANNEL_4;
	}

	if (direction == FORWARD) {
		sConfigOC.Pulse = (uint16_t) value;
		HAL_TIM_PWM_ConfigChannel(_pwm, &sConfigOC, TIM_CHANNEL);

		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(_pwm, &sConfigOC, TIM_CHANNEL_N);
	} else {
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(_pwm, &sConfigOC, TIM_CHANNEL);

		sConfigOC.Pulse = (uint16_t) value;
		HAL_TIM_PWM_ConfigChannel(_pwm, &sConfigOC, TIM_CHANNEL_N);
	}

	if (pwmNeedToStart) {
		HAL_TIM_PWM_Start_IT(_pwm, TIM_CHANNEL);
		HAL_TIM_PWM_Start_IT(_pwm, TIM_CHANNEL_N);
	}
}
