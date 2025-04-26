/*
 * MotorControl.h
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */
#include "ConfigurationFile.h"

#ifndef MOTORCONTROL_MOTORCONTROL_H_
#define MOTORCONTROL_MOTORCONTROL_H_

class MotorControl {
public:
	typedef enum {
		FORWARD, BACKWARD
	} motorDirectionEnum;

	typedef enum {
		LEFT, RIGHT
	} motorSelectionEnum;

	MotorControl(TIM_HandleTypeDef &pwm) {
		_pwm = &pwm;
	}

	void SetPwm(motorSelectionEnum selection, motorDirectionEnum direction, uint8_t percent, bool pwmNeedToStart);

protected:
private:
	TIM_HandleTypeDef *_pwm;

};

#endif /* MOTORCONTROL_MOTORCONTROL_H_ */
