/*
 * ClosedControlLoop.h
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */

#ifndef CLOSEDCONTROLLOOP_CLOSEDCONTROLLOOP_H_
#define CLOSEDCONTROLLOOP_CLOSEDCONTROLLOOP_H_

#include "ConfigurationFile.h"

class ClosedControlLoop {
public:
	ClosedControlLoop() {
	}

	void updatePositionAndVelocityDiffDrive(poseStruct *pose, velocityStruct *velocity,
		const int16_t leftWheelEncoderCount, const int16_t rightWheelEncoderCount, const double dt);

protected:
private:

};

#endif /* CLOSEDCONTROLLOOP_CLOSEDCONTROLLOOP_H_ */
