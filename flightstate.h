#ifndef FLIGHTSTATE_H_
#define FLIGHTSTATE_H_

#include "receiver.h"
#include "hott.h"
#include "imu.h"
#include "imuPosition.h"
#include "pressure.h"
#include "controller.h"


#define MAXIMAL_START_POWER		5

typedef enum {
	FLYING, CAREFREE, HEIGHT
} FlightFunction_t;

struct FlightState{
	// RC main signals (things like carefree already included).
	// At 100% RC signal an inclination of config.maxAngle will be reached
	float roll, pitch;
	// At 100% RC signal a rotation of config.maxRotation will be reached
	float yaw;
	// rate of desired ascend in m/s
	float ascend;
	float motorPower;
	// 0-200
	int16_t RCpower;
	float howerPower;
	uint32_t timestampHowerpowerUpdate;
	float carefreeHeading;
	float holdingHeight;
	// state bit field
	uint8_t motorOff :1;
	uint8_t flying :1;
	uint8_t signalLost :1;
	uint8_t carefree :1;
	uint8_t heightControl :1;
	uint8_t heightVario :1;
	uint8_t landing :1;
	uint8_t starting :1;

	uint8_t flightTimeSeconds;
	uint8_t flightTimeMinutes;
	uint32_t timestampFlightStart;
};

extern struct FlightState flightState;

/*
 * initializes flightState (e.g. !flying, motorOff, signalLost
 */
void flightState_Init(void);
/*
 * calculates the new flightState based on the receiver signal
 */
void flightState_Update(void);

/*
 * sets the motor values according to the flightState
 */
void flightState_SetMotors(void);

/*
 * adjusts howerpower value if necessary
 */
void flightState_UpdateHowerpower(void);

/*
 * checks whether enough sensor information is available to enable a specific function
 * @retval 1 if function is available, 0 otherwise
 */
uint8_t flightState_FunctionAvailable(FlightFunction_t func);

#endif /* FLIGHTSTATE_H_ */
