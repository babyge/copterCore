/*
 * gyro.h
 *
 *  Created on: Oct 31, 2013
 *      Author: jan
 */

#ifndef GYRO_H_
#define GYRO_H_

#include "hal.h"
#include "hott.h"


//#define GYRO_SCALE_NOM			7
//#define GYRO_SCALE_DEN			40
/* sensitivity: 65.5 LSB/°/s
 * gyro value should be in rad/s
 */
//#define GYRO_SCALE				(1.0f/65.5f)*DEG_TO_RAD // 500°/s
#define GYRO_SCALE				(1.0f/16.4f)*DEG_TO_RAD // 2000°/s

struct {
	float X, Y, Z;
	int16_t rawX, rawY, rawZ;
	uint8_t rawValues[6];
	uint32_t timestampUpdate, timestampChange;
	FlagStatus valid;
} gyro;


/*
 * writes the values from gyroDataRX into the gyro structur
 */
void gyro_Update(void);
/*
 * writes the gyro to the standard output (USART2)
 */
void gyro_Print(void);
/*
 * calibrates the gyro. Do to this the imu must not be moving.
 * This functions takes 100 values and averages them. Execution time
 * depends on the sensor update rate (see I2C_CYCLE_FREQUENCY in i2c.h)
 */
void gyro_Calibrate(void);
#endif /* GYRO_H_ */
