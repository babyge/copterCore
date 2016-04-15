#include "stepresponse.h"

/*
 * initializes a new step response
 * @param type			indicates the system which is tested. May be any of the following values: ROLL, PITCH, YAW
 * @param stepsize		input value for t>0 (after the step)
 * @param power			motor-power during step-response
 * @param advanceTime	time in ms before the step
 * @param responseTime	time in ms after the step
 * @param intervalTime	time in ms between data points
 */
ErrorStatus StepResponse_Start(StepResponseType type, float stepsize,
		float power, uint16_t advanceTime, uint16_t responseTime,
		uint16_t intervalTime) {
	if (stepresponse.active || !flightState.motorOff)
		// step response already active or motors already running
		return ERROR;
	if (responseTime / intervalTime >= MAX_STEP_RESPONSE_DATA_SIZE)
		// data interval is too small -> too many values
		return ERROR;
	// initialize step response
	stepresponse.type = type;
	stepresponse.stepSize = stepsize;
	stepresponse.stepPower = power;
	stepresponse.responseTime = responseTime;
	stepresponse.dataInterval = intervalTime;
	time_SetTimer(&stepresponse.advanceTimer, advanceTime);
	stepresponse.active = SET;
	stepresponse.stepDone = RESET;
	stepresponse.dataCount = 0;
	stepresponse.inputSignal = 0;

	switch (stepresponse.type) {
	case ROLLVELOCITY:
		controller_SetHorizontalCtrlStruct(HORI_ROLL_VELOCITY);
		break;
	case PITCHVELOCITY:
		controller_SetHorizontalCtrlStruct(HORI_PITCH_VELOCITY);
		break;
	case YAWVELOCITY:
		controller_SetYawCtrlStruct(YAW_VELOCITY);
		break;
	default:
		controller_SetHorizontalCtrlStruct(HORI_NO_ACTION);
		controller_SetVerticalCtrlStruct(VERT_NO_ACTION);
		controller_SetYawCtrlStruct(YAW_NO_ACTION);
		break;
	}

	log_LogFileEntry("started step response");

	return SUCCESS;
}

/*
 * Updates the step response data
 */
void StepResponse_Update() {
	if (stepresponse.active) {
		if (stepresponse.stepDone) {
			// after step -> in segment 'response'
			if (time_TimerElapsed(&stepresponse.responseTimer)) {
				// step response finished
				stepresponse.active = RESET;
				controller_SetHorizontalCtrlStruct(HORI_NO_ACTION);
				controller_SetVerticalCtrlStruct(VERT_NO_ACTION);
				controller_SetYawCtrlStruct(YAW_NO_ACTION);
				log_LogFileEntry("finished step response");
				// send step response data
				stdComm_SendMessage(MESSAGE_SEND_STEPRESPONSE,
						(uint8_t*) &stepresponse.responseData,
						stepresponse.dataCount * 4);
			} else if (time_TimerElapsed(&stepresponse.dataIntervalTimer)) {
				// new data must be saved
				StepResponse_AddDataPoint();
				// set timer for next data interval
				stepresponse.dataIntervalTimer += stepresponse.dataInterval
						* 10;
			}
		} else {
			// before step -> in segment 'advance'
			if (time_TimerElapsed(&stepresponse.advanceTimer)) {
				// 'advance' segment finsihed
				stepresponse.stepDone = SET;
				stepresponse.inputSignal = stepresponse.stepSize;
				// save first data point
				StepResponse_AddDataPoint();
				// set timer for 'response' segment
				time_SetTimer(&stepresponse.responseTimer,
						stepresponse.responseTime);
				// set timer for next data point
				time_SetTimer(&stepresponse.dataIntervalTimer,
						stepresponse.dataInterval);
			}
		}
	}
}

/*
 * saves the current response value into the responseData-Array
 */
void StepResponse_AddDataPoint(void) {
	if (stepresponse.active
			&& stepresponse.dataCount < MAX_STEP_RESPONSE_DATA_SIZE) {
		float data = 0.0f;

		//select variable to save
		switch (stepresponse.type) {
		case ROLL:
		case ROLLVELOCITY:
			data = kalman.x[KALMAN_X_GYRO_X];
			break;
		case PITCH:
		case PITCHVELOCITY:
			data = kalman.x[KALMAN_X_GYRO_Y];
			break;
		case YAW:
		case YAWVELOCITY:
			data = kalman.x[KALMAN_X_GYRO_Z];
			break;
		}
		// save new data into array
		stepresponse.responseData[stepresponse.dataCount++] = data;
	}
}

/*
 * Sets the motor values according to the step response
 */
void StepResponse_SetMotors(void) {
	if (stepresponse.active) {
		switch (stepresponse.type) {
		case ROLL:
			motor_Mixer(stepresponse.stepPower, stepresponse.inputSignal, 0, 0);
			break;
		case PITCH:
			motor_Mixer(stepresponse.stepPower, 0, stepresponse.inputSignal, 0);
			break;
		case YAW:
			motor_Mixer(stepresponse.stepPower, 0, 0, stepresponse.inputSignal);
			break;
		case ROLLVELOCITY:
			motor_Mixer(stepresponse.stepPower,
					control.angularVelocity.Y.output, 0, 0);
			break;
		case PITCHVELOCITY:
			motor_Mixer(stepresponse.stepPower, 0,
					control.angularVelocity.X.output, 0);
			break;
		case YAWVELOCITY:
			motor_Mixer(stepresponse.stepPower, 0, 0,
					control.angularVelocity.Z.output);
			break;
		}
	}
}

