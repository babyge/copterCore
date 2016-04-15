/*
 * magnetometer.c
 *
 *  Created on: Nov 3, 2013
 *      Author: jan
 */

#include "magnetometer.h"

/*
 * writes the values from magDataRX into the magnetometer structur
 */
void mag_Update(void) {
	magnetometer.timestampUpdate = time_GetMillis();
	// buffer values to detect 'frozen' data
	int16_t rawbufX = magnetometer.rawX;
	int16_t rawbufY = magnetometer.rawY;
	int16_t rawbufZ = magnetometer.rawZ;
	magnetometer.rawY = magnetometer.rawValues[1]
			+ (magnetometer.rawValues[0] << 8);
	magnetometer.rawZ = magnetometer.rawValues[3]
			+ (magnetometer.rawValues[2] << 8);
	magnetometer.rawX = magnetometer.rawValues[5]
			+ (magnetometer.rawValues[4] << 8);
	if (rawbufX != magnetometer.rawX || rawbufY != magnetometer.rawY
			|| rawbufZ != magnetometer.rawZ) {
		// data has changed
		magnetometer.timestampChange = magnetometer.timestampUpdate;
		if (magnetometer.valid != SET)
			log_LogFileEntry("magnetometer data valid");
		magnetometer.valid = SET;
	}
	if (magnetometer.calibration.state == START) {
		// save minimum and maximum values during calibration
		if (magnetometer.rawX < magnetometer.calibration.minX)
			magnetometer.calibration.minX = magnetometer.rawX;
		if (magnetometer.rawX > magnetometer.calibration.maxX)
			magnetometer.calibration.maxX = magnetometer.rawX;

		if (magnetometer.rawY < magnetometer.calibration.minY)
			magnetometer.calibration.minY = magnetometer.rawY;
		if (magnetometer.rawY > magnetometer.calibration.maxY)
			magnetometer.calibration.maxY = magnetometer.rawY;

		if (magnetometer.rawZ < magnetometer.calibration.minZ)
			magnetometer.calibration.minZ = magnetometer.rawZ;
		if (magnetometer.rawZ > magnetometer.calibration.maxZ)
			magnetometer.calibration.maxZ = magnetometer.rawZ;
	}
	float magX = (magnetometer.rawX - config.magXOffset);
	float magY = (magnetometer.rawY - config.magYOffset);
	float magZ = -(magnetometer.rawZ - config.magZOffset);

	magX *= config.magXScale;
	magY *= config.magYScale;
	magZ *= config.magZScale;

	/*
	 * matrix rotation
	 */
	float bufX = config.MagMatrix[0][0] * magX + config.MagMatrix[0][1] * magY
			+ config.MagMatrix[0][2] * magZ;
	float bufY = config.MagMatrix[1][0] * magX + config.MagMatrix[1][1] * magY
			+ config.MagMatrix[1][2] * magZ;
	float bufZ = config.MagMatrix[2][0] * magX + config.MagMatrix[2][1] * magY
			+ config.MagMatrix[2][2] * magZ;

	magnetometer.X = magnetometer.X * (1 - config.alphaMag)
			+ bufX * config.alphaMag;
	magnetometer.Y = magnetometer.Y * (1 - config.alphaMag)
			+ bufY * config.alphaMag;
	magnetometer.Z = magnetometer.Z * (1 - config.alphaMag)
			+ bufZ * config.alphaMag;

	magnetometer.magnitude = sqrtf(
			magnetometer.X * magnetometer.X + magnetometer.Y * magnetometer.Y
					+ magnetometer.Z * magnetometer.Z);
	if (magnetometer.magnitude > 1.5f || magnetometer.magnitude < 0.5f) {
		hott.speak = SPEAK_ERR_COMPASS;
	}
}
/*
 * writes the magnetometer to the standard output (USART2)
 */
void mag_Print(void) {
	// convert to mg
	stdComm_puts("magnetometer X:");
	stdComm_PrintValue(magnetometer.X * 1000);
	stdComm_puts(" Y:");
	stdComm_PrintValue(magnetometer.Y * 1000);
	stdComm_puts(" Z:");
	stdComm_PrintValue(magnetometer.Z * 1000);
	stdComm_puts(" %:");
	stdComm_PrintValue(magnetometer.magnitude * 100);
	usart_putcStdComm(0x0A);
}
/*
 * starts/stop the calibration process (data acquisition for the calibration
 * is done in mag_Update, thus mag_Update must be called in regularly during
 * the calibration)
 */
void mag_Calibrate(MAG_ActivityTypeDef action) {
	if (action == START) {
		log_LogFileEntry("starting mag. calibration.");
		buzzer_Signal(1);
		// reset minimum and maximum values
		magnetometer.calibration.maxX = INT16_MIN;
		magnetometer.calibration.maxY = INT16_MIN;
		magnetometer.calibration.maxZ = INT16_MIN;
		magnetometer.calibration.minX = INT16_MAX;
		magnetometer.calibration.minY = INT16_MAX;
		magnetometer.calibration.minZ = INT16_MAX;
		magnetometer.calibration.state = START;
		hott.speak = SPEAK_CALIBRATE;
	} else if (action == STOP) {
		magnetometer.calibration.state = STOP;
		/*
		 * check for minimum amplitude
		 * (small differences between max and min values indicate
		 * insufficient calibration data)
		 */
		if (magnetometer.calibration.maxX
				- magnetometer.calibration.minX>= MAG_MINIMUM_RANGE
				&& magnetometer.calibration.maxY - magnetometer.calibration.minY >= MAG_MINIMUM_RANGE
				&& magnetometer.calibration.maxZ - magnetometer.calibration.minZ >= MAG_MINIMUM_RANGE) {
			config.magXOffset = (magnetometer.calibration.maxX
					+ magnetometer.calibration.minX) / 2;
			config.magYOffset = (magnetometer.calibration.maxY
					+ magnetometer.calibration.minY) / 2;
			config.magZOffset = (magnetometer.calibration.maxZ
					+ magnetometer.calibration.minZ) / 2;
			config.magXScale = 2.0f
					/ (magnetometer.calibration.maxX
							- magnetometer.calibration.minX);
			config.magYScale = 2.0f
					/ (magnetometer.calibration.maxY
							- magnetometer.calibration.minY);
			config.magZScale = 2.0f
					/ (magnetometer.calibration.maxZ
							- magnetometer.calibration.minZ);
			log_LogFileEntry("finished mag. calibration.");
			buzzer_Signal(1);
		} else {
			log_LogFileEntry("ERROR: mag. calibration");
			buzzer_Signal(2);
			hott.speak = SPEAK_ERR_CALIBRATION;
		}
	}
}
