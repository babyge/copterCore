/*
 * magnetometer.h
 *
 *  Created on: Nov 3, 2013
 *      Author: jan
 */

#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

#include "hal.h"
#include "hott.h"
#include <math.h>

/*************************************************************
 * calibration data
 ************************************************************/
/* minimum range of the measured during calibration to accept
 * the calibration data. Increase this value to force a more
 * thoroughly calibration, decrease it when to many calibrations
 * get rejected
 */
#define MAG_MINIMUM_RANGE	600

typedef enum {START = 1, STOP = !START} MAG_ActivityTypeDef;

/*************************************************************
 * magnetometer values
 ************************************************************/
struct Magnetometer{
	float X, Y, Z, magnitude;
	int16_t rawX, rawY, rawZ;
	uint8_t rawValues[6];
	// contains the minimum and maximum values measured during calibration
	struct {
		int16_t maxX, maxY, maxZ;
		int16_t minX, minY, minZ;
		// indicates whether a calibration is currently running
		MAG_ActivityTypeDef state;
	} calibration;
	uint32_t timestampUpdate, timestampChange;
	FlagStatus valid;
};

extern struct Magnetometer magnetometer;

/*
 * writes the values from magDataRX into the magnetometer structur
 */
void mag_Update(void);
/*
 * writes the magnetometer data to the standard output (USART2)
 */
void mag_Print(void);
/*
 * starts/stop the calibration process (data acquisition for the calibration
 * is done in mag_Update, thus mag_Update must be called in regularly during
 * the calibration)
 */
void mag_Calibrate(MAG_ActivityTypeDef action);

#endif /* MAGNETOMETER_H_ */
