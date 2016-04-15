/*
 * accelerometer.h
 *
 *  Created on: Oct 30, 2013
 *      Author: jan
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include "hal.h"
#include "hott.h"
#include <math.h>


#define ACC_SCALE			(1.0f/4096.0f)

extern struct Accelerometer{
	float X, Y, Z, magnitude;
	int16_t rawX, rawY, rawZ;
	uint8_t rawValues[6];
	uint32_t timestampUpdate, timestampChange;
	FlagStatus valid;
} accelerometer;



/*
 * writes the values from accDataRX into the accValues structure
 */
void acc_Update(void);
/*
 * writes the accelerometer to the standard output (USART2)
 */
void acc_Print(void);
/*
 * calibrates the accelerometer. Do to this the IMU must be level.
 * This functions takes 100 values and averages them. Execution time
 * depends on the sensor update rate (see I2C_CYCLE_FREQUENCY in i2c.h)
 */
void acc_Calibrate(void);

#endif /* ACCELEROMETER_H_ */
