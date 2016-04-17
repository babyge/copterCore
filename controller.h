/*
 * general implementation of a PID controller
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "hal.h"
#include "flightstate.h"
#include "kalman.h"
#include "imu.h"
#include "imuPosition.h"
#include "stepresponse.h"

// various control structures
typedef enum {
	VERT_NO_ACTION = 0, VERT_MANUAL = 1, VERT_VARIO = 2, VERT_HOLD = 3
} VerticalCtrlStruct;

typedef enum {
	HORI_NO_ACTION = 0,
	HORI_MANUAL = 1,
	HORI_ROLL_VELOCITY = 2,
	HORI_PITCH_VELOCITY = 3
} HorizontalCtrlStruct;

typedef enum {
	YAW_NO_ACTION = 0, YAW_MANUAL = 1, YAW_VELOCITY = 2, YAW_ANGLE = 3
} YawCtrlStruct;

struct pidControl {
	// enable/disable the complete controller
	FlagStatus enable;
	// pointer to the desired value
	float *input;
	// pointer to the measured value
	float *actual;
	// output value of the controller
	float output, residualOutput;
	// enable/disable the residual output
	FlagStatus useResidualOutput;
	// enable/disable input wrap-around
	// (for angle controllers)
	FlagStatus useInputWrapAround;
	// maximum 'distance' between input and actual value
	// (if it is bigger, the other way around is actually faster)
	float wrapAroundLimit;
	// controller parameters (should be >= 0)
	float *P, *I, *D;
	// maximal output interval
	float *upperBound, *lowerBound;
	// sum over the controller error
	float integral;
	// memory for D-term
	float D_last;
	float e_last;
	// last execution time in multiples of 100us
	uint32_t timestamp;
	// time between executions in multiples of 100us
	uint16_t *interval;
};

struct Control {
	struct {
		struct pidControl X, Y, Z;
	} angularVelocity;
	struct {
		struct pidControl X, Y, Z;
	} angle;
	struct {
		struct pidControl X, Y, Z;
	} velocity;
	struct {
		struct pidControl X, Y, Z;
	} position;
	VerticalCtrlStruct structureVertical;
	HorizontalCtrlStruct structureHorizontal;
	YawCtrlStruct structureYaw;
};

extern struct Control control;

/*
 * resets all controllers and initializes the pointers in the controller structs
 */
void controller_Init(void);
/*
 * connects and enables/disables the various controller to achieve the desired
 * behavior
 */
void controller_SetVerticalCtrlStruct(VerticalCtrlStruct newStruct);
void controller_SetHorizontalCtrlStruct(HorizontalCtrlStruct newStruct);
void controller_SetYawCtrlStruct(YawCtrlStruct newStruct);
/*
 * checks for (and if necessary performs) the next controller iteration
 */
void controller_Update(void);
/*
 * calculates the next iteration of the PID controller
 */
void controller_PID(struct pidControl *c);
/*
 * resets a controller
 */
void controller_Reset(struct pidControl *c);

#endif /* CONTROLLER_H_ */
