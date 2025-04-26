/*
 * RobotSpec.h
 *
 *  Created on: Apr 25, 2025
 *      Author: gdurand
 */

#ifndef ROBOTDEFINITION_H_
#define ROBOTDEFINITION_H_
#include "ConfigurationFile.h"

class RobotDefinition {
public:
	static constexpr double WHEEL_BASE = 1.0; //in meter (distance between left and right wheel
	static constexpr double WHEEL_RADIUS = 1.0; //in meter
	static constexpr double ENCODER_RESOLUTION = 500; //Number of tick encoder will do for a revolution

protected:
private:

};

#endif /* ROBOTDEFINITION_H_ */
