/*
 * ClosedControlLoop.cpp
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */
#include "ClosedControlLoop.h"
#include "RobotDefinition.h"

void ClosedControlLoop::updatePositionAndVelocityDiffDrive(poseStruct *pose, velocityStruct *velocity,
	const int16_t leftWheelEncoderCount, const int16_t rightWheelEncoderCount, const double dt) {

	double dist_left = TWO_PI * RobotDefinition::WHEEL_RADIUS
		* ((double) leftWheelEncoderCount / RobotDefinition::ENCODER_RESOLUTION);
	double dist_right = TWO_PI * RobotDefinition::WHEEL_RADIUS
		* ((double) rightWheelEncoderCount / RobotDefinition::ENCODER_RESOLUTION);

	double meanDistance = (dist_right + dist_left) / 2.0;
	double omega = (dist_right - dist_left) / RobotDefinition::WHEEL_BASE;

	pose->x += meanDistance * cos(omega);
	pose->y += meanDistance * sin(omega);
	pose->theta += omega;
	pose->ttl = HAL_GetTick();

	velocity->vLinear = meanDistance / dt;
	velocity->vAngular = omega / dt;
	velocity->ttl = HAL_GetTick();

	while (pose->theta > M_PI)
		pose->theta -= TWO_PI;
	while (pose->theta < -M_PI)
		pose->theta += TWO_PI;
}
