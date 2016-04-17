#include "flightstate.h"

struct FlightState flightState;

/*
 * initializes flightState (e.g. !flying, motorOff, signalLost
 */
void flightState_Init(void) {
	flightState.flying = 0;
	flightState.motorOff = 1;
	flightState.signalLost = 1;
	flightState.howerPower = config.howerPower;
}
/*
 * calculates the new flightState based on the receiver signal
 */
void flightState_Update(void) {
	if (receiver.valid) {
		if (flightState.signalLost && flightState.flying)
			log_LogFileEntry("entered normal flying mode");
		flightState.signalLost = 0;
		// use receiver data to calculate flightState
		if (!flightState.flying) {
			if (receiver.switches[SWITCH_FLYING]
					&& flightState.RCpower < MAXIMAL_START_POWER) {
				if (flightState_FunctionAvailable(FLYING)) {
					/*
					 * enter flying mode.
					 */
					flightState.flying = 1;
					flightState.flightTimeMinutes = 0;
					flightState.flightTimeSeconds = 0;
					flightState.timestampFlightStart = time_GetMillis();
					stdComm_puts("flight started\n");
					log_LogFileEntry("FLIGHT STARTED");
					log_FlightLogStartRequest();
					if (config.configMask1 & CONFIG_MASK_KALMAN_LOG) {
						// kalmanLog enabled
						log_KalmanLogStartRequest();
					}
				} else {
					hott.speak = SPEAK_ERR_SENSOR;
				}
			} else if (receiver.switches[SWITCH_MOTOR_OFF]) {
				if (!flightState.motorOff)
					log_LogFileEntry("motors stopped");
				flightState.motorOff = 1;
			} else {
				if (flightState.motorOff) {
					log_LogFileEntry("motors running");
					gps_SetHomePosition();
					pressure_SetHeight(0.0f);
					imuPosition_Reset();
				}
				flightState.motorOff = 0;
			}
		} else {
			// copter is flying
			// keep track of flight time
			uint32_t timediff = time_GetMillis()
					- flightState.timestampFlightStart;
			flightState.flightTimeMinutes = timediff / 60000;
			flightState.flightTimeSeconds = (timediff / 1000) % 60;
			/*********************************************
			 * stop flight when necessary
			 ********************************************/
			if (!receiver.switches[SWITCH_FLYING]
					&& flightState.RCpower < MAXIMAL_START_POWER) {
				// flight stopped
				// deactivate all flighstate functions
				flightState.flying = 0;
				flightState.carefree = 0;
				flightState.heightControl = 0;
				flightState.heightVario = 0;
				stdComm_puts("flight stopped\n");
				log_LogFileEntry("FLIGHT STOPPED");
				log_FlightLogStopRequest();
				log_KalmanLogStopRequest();
			}
			/*********************************************
			 * handle carefree
			 ********************************************/
			if (receiver.switches[SWITCH_CAREFREE] && !flightState.carefree) {
				if (flightState_FunctionAvailable(CAREFREE)) {
					// activate carefree mode
					flightState.carefree = 1;
					flightState.carefreeHeading = attitude.yaw;
					log_LogFileEntry("activated carefree");
					hott.speak = SPEAK_CF_ON;
				} else {
					hott.speak = SPEAK_ERR_SENSOR;
				}
			} else if ((!receiver.switches[SWITCH_CAREFREE]
					|| !flightState_FunctionAvailable(CAREFREE))
					&& flightState.carefree) {
				//carefree deactivated
				flightState.carefree = 0;
				log_LogFileEntry("deactivated carefree");
				hott.speak = SPEAK_CF_OFF;
			}

			/*********************************************
			 * handle heading lock
			 ********************************************/
			if (receiver.switches[SWITCH_HEADING_LOCK] && !flightState.headingLock) {
				if (flightState_FunctionAvailable(HEADINGLOCK)) {
					// activate heading lock
					flightState.headingLock = 1;
					flightState.lockHeading = attitude.yaw;
					log_LogFileEntry("activated heading lock");
				} else {
					hott.speak = SPEAK_ERR_SENSOR;
				}
			} else if ((!receiver.switches[SWITCH_HEADING_LOCK]
					|| !flightState_FunctionAvailable(HEADINGLOCK))
					&& flightState.headingLock) {
				// heading lock deactivated
				flightState.headingLock = 0;
				log_LogFileEntry("deactivated heading lock");
			}

			/*********************************************
			 * handle height control
			 ********************************************/
			if (!receiver.switches[SWITCH_HEIGHT_OFF]
					&& !flightState.heightControl) {
				if (flightState_FunctionAvailable(HEIGHT)) {
					// activate height control
					flightState.heightControl = 1;
					log_LogFileEntry("activated height control");
					hott.speak = SPEAK_ALTITUDE_ON;
				} else {
					// no valid pressure data -> no height control possible
					hott.speak = SPEAK_ERR_SENSOR;
				}
			} else if ((receiver.switches[SWITCH_HEIGHT_OFF]
					|| !flightState_FunctionAvailable(HEIGHT))
					&& flightState.heightControl) {
				// deactivate height control
				flightState.heightControl = 0;
				flightState.heightVario = 0;
				log_LogFileEntry("deactivated height control");
				hott.speak = SPEAK_ALTITUDE_OFF;
			}
			if (flightState.heightControl) {
				// differentiate between vario and height hold
				if (receiver.switches[SWITCH_HEIGHT_HOLD]
						&& flightState.heightVario) {
					// switch to height hold
					flightState.heightVario = 0;
					flightState.holdingHeight = position.Z;
					log_LogFileEntry("height control mode: hold");
				} else if (!receiver.switches[SWITCH_HEIGHT_HOLD]
						&& !flightState.heightVario) {
					flightState.heightVario = 1;
					log_LogFileEntry("height control mode: vario");
				}
				if (pressure.valid != SET) {
					// no valid pressure data -> no height control possible
					// deactivate height control
					flightState.heightControl = 0;
					flightState.heightVario = 0;
					log_LogFileEntry(
							"deactivated height control due to faulty pressure sensor");
					hott.speak = SPEAK_ALTITUDE_OFF;
				}
			}
			/*********************************************
			 * handle autostart/-landing
			 ********************************************/
			if (receiver.switches[SWITCH_START_LAND] && !flightState.landing
					&& !flightState.starting) {
				// initiate landing/starting
				// check height above ground
				if (distance.bottom < 10) {
					// copter is on the ground -> start
					flightState.starting = 1;
					hott.speak = SPEAK_STARTING;
				} else {
					// copter is in the air -> land
					flightState.landing = 1;
					hott.speak = SPEAK_LANDING;
				}
			} else if (!receiver.switches[SWITCH_START_LAND]) {
				// abort landing/starting
				flightState.landing = flightState.starting = 0;
			}
		}
		// update control structures
		if (flightState.flying) {
			controller_SetHorizontalCtrlStruct(HORI_MANUAL);
			if (flightState.headingLock) {
				controller_SetYawCtrlStruct(YAW_ANGLE);
			} else {
				controller_SetYawCtrlStruct(YAW_MANUAL);
			}
			if (flightState.heightControl) {
				if (flightState.heightVario) {
					controller_SetVerticalCtrlStruct(VERT_VARIO);
				} else {
					controller_SetVerticalCtrlStruct(VERT_HOLD);
				}
			} else {
				controller_SetVerticalCtrlStruct(VERT_MANUAL);
			}
		} else {
			controller_SetHorizontalCtrlStruct(HORI_NO_ACTION);
			controller_SetVerticalCtrlStruct(VERT_NO_ACTION);
			controller_SetYawCtrlStruct(YAW_NO_ACTION);
		}

		// calculate main RC signals
		flightState.yaw = (float) receiver.channel[CHANNEL_YAW] * 0.0003125
				* config.maxRotation;
		flightState.pitch = (float) receiver.channel[CHANNEL_PITCH] * 0.0003125
				* config.maxAngle;
		flightState.roll = (float) receiver.channel[CHANNEL_ROLL] * 0.0003125
				* config.maxAngle;
		flightState.ascend = (float) receiver.channel[CHANNEL_POWER] * 0.0003125
				* config.maxAscend;
		if (flightState.carefree) {
			// rotate roll and pitch
			float pitchBuffer = flightState.pitch;
			float rollBuffer = flightState.roll;
			float headingDiff = attitude.yaw - flightState.carefreeHeading;
			flightState.pitch = pitchBuffer * cosf(headingDiff)
					- rollBuffer * sinf(headingDiff);
			flightState.roll = pitchBuffer * sinf(headingDiff)
					+ rollBuffer * cosf(headingDiff);
		}
		flightState.RCpower = receiver.channel[CHANNEL_POWER] / 32 + 100;
		if (flightState.RCpower < 0)
			flightState.RCpower = 0;
	} else {
		// receiver fault
		buzzer_Signal(3);
		if (!flightState.flying) {
			if (!flightState.motorOff) {
				log_LogFileEntry("motors stopped");
				flightState.motorOff = 1;
			}
		} else {
			// TODO change to higher emergency functions once they are available
			flightState.yaw = 0;
			flightState.roll = 0;
			flightState.pitch = 0;
			// 95% of howerPower
			flightState.RCpower = flightState.howerPower
					- flightState.howerPower / 20;
			if (pressure.valid == SET) {
				// pressure sensor available
				// use controller to descend at certain rate
				flightState.ascend = -0.5f;
				controller_SetVerticalCtrlStruct(VERT_VARIO);
			} else {
				// 95% of howerPower
				flightState.RCpower = flightState.howerPower
						- flightState.howerPower / 20;
				controller_SetVerticalCtrlStruct(VERT_MANUAL);
			}
		}
		if (!flightState.signalLost && flightState.flying)
			log_LogFileEntry("entered emergency flying mode");
		flightState.signalLost = 1;
	}
}
/*
 * sets the motor values according to the flightState
 */
void flightState_SetMotors(void) {
	if (flightState.motorOff) {
		// manually switch motors completely off
		uint8_t i;
		for (i = 0; i < config.motor.num; i++) {
			motor.velocity[i] = 0;
		}
	} else if (flightState.flying) {
		// set motor values according to current control structure
		// TODO change this section as more flightStates get implemented
		if (!flightState.heightControl) {
			flightState.motorPower = flightState.RCpower;
		} else {
			// compensate for pitch and roll angle
			flightState.motorPower = (flightState.howerPower
					+ control.velocity.Z.output)
					/ (attitude.cosPitch * attitude.cosRoll);
		}
		motor_Mixer(flightState.motorPower, control.angularVelocity.Y.output,
				control.angularVelocity.X.output,
				control.angularVelocity.Z.output);
	} else {
		// motors should be running slowly (->startup phase)
		motor_Mixer(MOTOR_MINVELOCITY, 0, 0, 0);
	}
}

/*
 * adjusts howerpower value if necessary
 */
void flightState_UpdateHowerpower(void) {
	uint32_t time = time_GetMillis();
	float timediff = (time - flightState.timestampHowerpowerUpdate) * 0.001f;
	flightState.timestampHowerpowerUpdate = time;
	if (flightState.flying) {
		if (position.acceleration.Z > 0
				&& flightState.motorPower < flightState.howerPower) {
			// copter is accelerating with less than howerpower
			// -> howerpower value is too high
			flightState.howerPower -= timediff;
		} else if (position.acceleration.Z < 0
				&& flightState.motorPower > flightState.howerPower) {
			// copter is decelerating with more than howerpower
			// -> howerpower value is too low
			flightState.howerPower += timediff;
		}
	}
}

/*
 * checks whether enough sensor information is available to enable a specific function
 * @retval 1 if function is available, 0 otherwise
 */
uint8_t flightState_FunctionAvailable(FlightFunction_t func) {
	uint8_t res = 0;
	switch (func) {
	case FLYING:
		if (accelerometer.valid == SET && gyro.valid == SET
				&& motor.genericError == 0)
			res = 1;
		break;
	case CAREFREE:
	case HEADINGLOCK:
		if (flightState_FunctionAvailable(FLYING) && magnetometer.valid == SET)
			res = 1;
		break;
	case HEIGHT:
		if (flightState_FunctionAvailable(FLYING) && pressure.valid == SET)
			res = 1;
		break;
	}
	return res;
}
