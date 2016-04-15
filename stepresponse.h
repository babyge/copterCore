#ifndef STEPRESPONSE_H_
#define STEPRESPONSE_H_

#include "hal.h"
#include "flightstate.h"

#define MAX_STEP_RESPONSE_DATA_SIZE		512

typedef enum {
	ROLL = 0, PITCH = 1, YAW = 2, ROLLVELOCITY = 3, PITCHVELOCITY = 4, YAWVELOCITY = 5
} StepResponseType;

struct {
	// step response active indicator
	FlagStatus active;
	// SET: currently in 'response'-period, RESET: currently in 'advance'-period
	FlagStatus stepDone;
	// timer for the different periods
	uint32_t advanceTimer;
	uint32_t responseTimer;
	uint32_t dataIntervalTimer;
	// buffer for period times
	uint16_t responseTime;
	uint16_t dataInterval;
	// stepsize for the roll, pitch or yaw signal
	float stepSize;
	// overall motorpower during the stepresponse
	float stepPower;
	// input signal for stepresponse (0 before step, stepSize after step)
	float inputSignal;
	StepResponseType type;
	// memory space for step response
	float responseData[MAX_STEP_RESPONSE_DATA_SIZE];
	uint16_t dataCount;
} stepresponse;

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
		uint16_t intervalTime);

/*
 * Updates the step response data
 */
void StepResponse_Update();

/*
 * saves the current response value into the responseData-Array
 */
void StepResponse_AddDataPoint(void);

/*
 * Sets the motor values according to the step response
 */
void StepResponse_SetMotors(void);

#endif /* STEPRESPONSE_H_ */
