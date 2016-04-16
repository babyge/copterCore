#include "controller.h"

struct Control control;

/*
 * resets all controllers and initializes the pointers in the controller structs
 */
void controller_Init(void) {
	/*
	 * angular velocity controller
	 */
	control.angularVelocity.X.P =
			&config.controller[CONTROLLER_ANGULARVELOCITY_X].P;
	control.angularVelocity.X.I =
			&config.controller[CONTROLLER_ANGULARVELOCITY_X].I;
	control.angularVelocity.X.D =
			&config.controller[CONTROLLER_ANGULARVELOCITY_X].D;
	control.angularVelocity.X.lowerBound =
			&config.controller[CONTROLLER_ANGULARVELOCITY_X].lowerLimit;
	control.angularVelocity.X.upperBound =
			&config.controller[CONTROLLER_ANGULARVELOCITY_X].upperLimit;
	control.angularVelocity.X.interval =
			&config.controller[CONTROLLER_ANGULARVELOCITY_X].interval;
	control.angularVelocity.X.actual = &kalman.x[KALMAN_X_GYRO_Y];

	control.angularVelocity.Y.P =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Y].P;
	control.angularVelocity.Y.I =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Y].I;
	control.angularVelocity.Y.D =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Y].D;
	control.angularVelocity.Y.lowerBound =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Y].lowerLimit;
	control.angularVelocity.Y.upperBound =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Y].upperLimit;
	control.angularVelocity.Y.interval =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Y].interval;
	control.angularVelocity.Y.actual = &kalman.x[KALMAN_X_GYRO_X];

	control.angularVelocity.Z.P =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Z].P;
	control.angularVelocity.Z.I =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Z].I;
	control.angularVelocity.Z.D =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Z].D;
	control.angularVelocity.Z.lowerBound =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Z].lowerLimit;
	control.angularVelocity.Z.upperBound =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Z].upperLimit;
	control.angularVelocity.Z.interval =
			&config.controller[CONTROLLER_ANGULARVELOCITY_Z].interval;
	control.angularVelocity.Z.actual = &kalman.x[KALMAN_X_GYRO_Z];

	/*
	 * angle controller
	 */
	control.angle.X.P = &config.controller[CONTROLLER_ANGLE_X].P;
	control.angle.X.I = &config.controller[CONTROLLER_ANGLE_X].I;
	control.angle.X.D = &config.controller[CONTROLLER_ANGLE_X].D;
	control.angle.X.lowerBound =
			&config.controller[CONTROLLER_ANGLE_X].lowerLimit;
	control.angle.X.upperBound =
			&config.controller[CONTROLLER_ANGLE_X].upperLimit;
	control.angle.X.interval = &config.controller[CONTROLLER_ANGLE_X].interval;
	control.angle.X.actual = &attitude.pitch;

	control.angle.Y.P = &config.controller[CONTROLLER_ANGLE_Y].P;
	control.angle.Y.I = &config.controller[CONTROLLER_ANGLE_Y].I;
	control.angle.Y.D = &config.controller[CONTROLLER_ANGLE_Y].D;
	control.angle.Y.lowerBound =
			&config.controller[CONTROLLER_ANGLE_Y].lowerLimit;
	control.angle.Y.upperBound =
			&config.controller[CONTROLLER_ANGLE_Y].upperLimit;
	control.angle.Y.interval = &config.controller[CONTROLLER_ANGLE_Y].interval;
	control.angle.Y.actual = &attitude.roll;

	control.angle.Z.P = &config.controller[CONTROLLER_ANGLE_Z].P;
	control.angle.Z.I = &config.controller[CONTROLLER_ANGLE_Z].I;
	control.angle.Z.D = &config.controller[CONTROLLER_ANGLE_Z].D;
	control.angle.Z.lowerBound =
			&config.controller[CONTROLLER_ANGLE_Z].lowerLimit;
	control.angle.Z.upperBound =
			&config.controller[CONTROLLER_ANGLE_Z].upperLimit;
	control.angle.Z.interval = &config.controller[CONTROLLER_ANGLE_Z].interval;
	control.angle.Z.actual = &attitude.yaw;

	/*
	 * height controller
	 */
	control.velocity.Z.P = &config.controller[CONTROLLER_ASCEND].P;
	control.velocity.Z.I = &config.controller[CONTROLLER_ASCEND].I;
	control.velocity.Z.D = &config.controller[CONTROLLER_ASCEND].D;
	control.velocity.Z.lowerBound =
			&config.controller[CONTROLLER_ASCEND].lowerLimit;
	control.velocity.Z.upperBound =
			&config.controller[CONTROLLER_ASCEND].upperLimit;
	control.velocity.Z.interval =
			&config.controller[CONTROLLER_ASCEND].interval;
	control.velocity.Z.actual = &position.velocity.Z;

	control.position.Z.P = &config.controller[CONTROLLER_HEIGHT].P;
	control.position.Z.I = &config.controller[CONTROLLER_HEIGHT].I;
	control.position.Z.D = &config.controller[CONTROLLER_HEIGHT].D;
	control.position.Z.lowerBound =
			&config.controller[CONTROLLER_HEIGHT].lowerLimit;
	control.position.Z.upperBound =
			&config.controller[CONTROLLER_HEIGHT].upperLimit;
	control.position.Z.interval =
			&config.controller[CONTROLLER_HEIGHT].interval;
	control.position.Z.actual = &position.Z;

	// controller reset
	controller_Reset(&control.angularVelocity.X);
	controller_Reset(&control.angularVelocity.Y);
	controller_Reset(&control.angularVelocity.Z);
	controller_Reset(&control.angle.X);
	controller_Reset(&control.angle.Y);
	controller_Reset(&control.angle.Z);
	controller_Reset(&control.velocity.Z);
	controller_Reset(&control.position.Z);

	controller_SetHorizontalCtrlStruct(HORI_NO_ACTION);
	controller_SetVerticalCtrlStruct(VERT_NO_ACTION);
	controller_SetYawCtrlStruct(YAW_NO_ACTION);
	// TODO gps controller
}
/*
 * connects and enables/disables the various controller to achieve the desired
 * behavior
 */
void controller_SetVerticalCtrlStruct(VerticalCtrlStruct newStruct) {
	if (newStruct == control.structureVertical)
		// no changes -> ignore request
		return;
	control.velocity.Z.enable = RESET;
	control.position.Z.enable = RESET;
	switch (newStruct) {
	case VERT_NO_ACTION:
		break;
	case VERT_MANUAL:
		break;
	case VERT_VARIO:
		// rate of ascend controller
		control.velocity.Z.input = &flightState.ascend;
		// reset used controller
		controller_Reset(&control.velocity.Z);
		// enable used controller
		control.velocity.Z.enable = SET;
		break;
	case VERT_HOLD:
		// height controller
		control.position.Z.input = &flightState.holdingHeight;
		// rate of ascend controller
		control.velocity.Z.input = &control.position.Z.output;
		// reset used controllers
		controller_Reset(&control.velocity.Z);
		controller_Reset(&control.position.Z);
		// enable used controllers
		control.velocity.Z.enable = SET;
		control.position.Z.enable = SET;
	}
	control.structureVertical = newStruct;
}
void controller_SetHorizontalCtrlStruct(HorizontalCtrlStruct newStruct) {
	if (newStruct == control.structureHorizontal)
		// no changes -> ignore request
		return;
	control.angle.X.enable = RESET;
	control.angle.Y.enable = RESET;
	control.angularVelocity.X.enable = RESET;
	control.angularVelocity.Y.enable = RESET;
	switch (newStruct) {
	case HORI_NO_ACTION:
		break;
	case HORI_MANUAL:
		// pitch controller
		control.angle.X.input = &flightState.pitch;
		control.angularVelocity.X.input = &control.angle.X.output;
		// roll controller
		control.angle.Y.input = &flightState.roll;
		control.angularVelocity.Y.input = &control.angle.Y.output;
		// reset used controllers
		controller_Reset(&control.angularVelocity.X);
		controller_Reset(&control.angularVelocity.Y);
		controller_Reset(&control.angle.X);
		controller_Reset(&control.angle.Y);
		// enable used controllers
		control.angle.X.enable = SET;
		control.angle.Y.enable = SET;
		control.angularVelocity.X.enable = SET;
		control.angularVelocity.Y.enable = SET;
		break;
	case HORI_PITCH_VELOCITY:
		// pitch controller
		control.angularVelocity.X.input = &stepresponse.inputSignal;
		// reset used controllers
		controller_Reset(&control.angularVelocity.X);
		// enable used controllersx
		control.angularVelocity.X.enable = SET;
		break;
	case HORI_ROLL_VELOCITY:
		// pitch controller
		control.angularVelocity.Y.input = &stepresponse.inputSignal;
		// reset used controllers
		controller_Reset(&control.angularVelocity.Y);
		// enable used controllers
		control.angularVelocity.Y.enable = SET;
		break;
	}
	control.structureHorizontal = newStruct;
}
void controller_SetYawCtrlStruct(YawCtrlStruct newStruct) {
	if (newStruct == control.structureYaw)
		// no changes -> ignore request
		return;
	control.angle.Z.enable = RESET;
	control.angularVelocity.Z.enable = RESET;
	switch (newStruct) {
	case YAW_NO_ACTION:
		break;
	case YAW_MANUAL:
		control.angularVelocity.Z.input = &flightState.yaw;
		controller_Reset(&control.angularVelocity.Z);
		control.angularVelocity.Z.enable = SET;
		break;
	case YAW_VELOCITY:
		// yaw controller
		control.angularVelocity.Z.input = &stepresponse.inputSignal;
		// reset used controllers
		controller_Reset(&control.angularVelocity.Z);
		// enable used controllers
		control.angularVelocity.Z.enable = SET;
		break;
	}
	control.structureYaw = newStruct;
}
/*
 * checks for (and if necessary performs) the next controller iteration
 */
void controller_Update(void) {
	controller_PID(&control.angularVelocity.X);
	controller_PID(&control.angularVelocity.Y);
	controller_PID(&control.angularVelocity.Z);
	controller_PID(&control.angle.X);
	controller_PID(&control.angle.Y);
	controller_PID(&control.angle.Z);
	controller_PID(&control.velocity.Z);
	controller_PID(&control.position.Z);
}

/*
 * calculates the next iteration of the PID controller
 */
void controller_PID(struct pidControl *c) {
	uint32_t currentTime = time_Get100us();
	if (c->enable == SET) {
		if (currentTime - c->timestamp >= *c->interval) {
			// controller error
			float e = *(c->input) - *(c->actual);
			// calculate interval since last update
			float timeDiff = (currentTime - c->timestamp) * 0.0001f;
			c->timestamp = currentTime;
			// only add error when the output is not saturated (anti-windup)
			if (!(e > 0 && c->output == *c->upperBound)
					&& !(e < 0 && c->output == *c->lowerBound))
				c->integral += e * timeDiff;
			// calculate derivative
			//float Tv = *c->D/(*c->P);
			//float TwoTv1 = 2 * CONTROLLER_DT * Tv;
			//float D_input = e * 2 * *c->D / (timeDiff + TwoTv1)
			//-c->D_last * (timeDiff - TwoTv1) / (timeDiff + TwoTv1);
			//float derivative = D_input - c->D_last;
			//c->D_last = D_input;
			float derivative = (e - c->e_last) / timeDiff;
			derivative += c->D_last * 0.7;
			c->e_last = e;
			c->D_last = derivative;
			if (c->useResidualOutput == SET)
				// start with residual output from previous iteration
				c->output = c->residualOutput;
			else
				c->output = 0.0f;
			c->residualOutput = 0.0f;
			// P-term
			c->output += e * *c->P;
			// I-term
			c->output += c->integral * *c->I;
			// D-term
			c->output += derivative * *c->D;
			// constrain output
			if (c->output > *c->upperBound) {
				c->residualOutput = c->output - *c->upperBound;
				c->output = *c->upperBound;
			} else if (c->output < *c->lowerBound) {
				c->residualOutput = c->output - *c->lowerBound;
				c->output = *c->lowerBound;
			}
		}
	}
}
/*
 * resets a controller
 */
void controller_Reset(struct pidControl *c) {
	c->D_last = 0;
	c->integral = 0;
	c->residualOutput = 0;
	c->useResidualOutput = RESET;
	c->timestamp = time_Get100us();
}
