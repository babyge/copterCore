/*
 * accelerometer.c
 *
 *  Created on: Oct 30, 2013
 *      Author: jan
 */

#include "accelerometer.h"

struct Accelerometer accelerometer;

/*
 * writes the values from accDataRX into the accValues structure
 */
void acc_Update(void) {
	accelerometer.timestampUpdate = time_GetMillis();
	// buffer values to detect 'frozen' data
	int16_t bufX = accelerometer.rawX;
	int16_t bufY = accelerometer.rawY;
	int16_t bufZ = accelerometer.rawZ;
	accelerometer.rawX = accelerometer.rawValues[1]
			+ (accelerometer.rawValues[0] << 8);
	accelerometer.rawY = accelerometer.rawValues[3]
			+ (accelerometer.rawValues[2] << 8);
	accelerometer.rawZ = accelerometer.rawValues[5]
			+ (accelerometer.rawValues[4] << 8);
	if (bufX != accelerometer.rawX || bufY != accelerometer.rawY
			|| bufZ != accelerometer.rawZ) {
		// data has changed
		accelerometer.timestampChange = accelerometer.timestampUpdate;
		if (accelerometer.valid != SET)
			log_LogFileEntry("accelerometer data valid");
		accelerometer.valid = SET;
	}
	float accX = -(accelerometer.rawX - config.accXOffset);
	float accY = (accelerometer.rawY - config.accYOffset);
	float accZ = -(accelerometer.rawZ - config.accZOffset);

	accX *= ACC_SCALE;
	accY *= ACC_SCALE;
	accZ *= ACC_SCALE;

	/*
	 * matrix rotation
	 */
	float rX = config.AccGyroMatrix[0][0] * accX
			+ config.AccGyroMatrix[0][1] * accY
			+ config.AccGyroMatrix[0][2] * accZ;
	float rY = config.AccGyroMatrix[1][0] * accX
			+ config.AccGyroMatrix[1][1] * accY
			+ config.AccGyroMatrix[1][2] * accZ;
	float rZ = config.AccGyroMatrix[2][0] * accX
			+ config.AccGyroMatrix[2][1] * accY
			+ config.AccGyroMatrix[2][2] * accZ;

	accelerometer.X = accelerometer.X * (1 - config.alphaAcc)
			+ rX * config.alphaAcc;
	accelerometer.Y = accelerometer.Y * (1 - config.alphaAcc)
			+ rY * config.alphaAcc;
	accelerometer.Z = accelerometer.Z * (1 - config.alphaAcc)
			+ rZ * config.alphaAcc;

	accelerometer.magnitude = sqrtf(
			accelerometer.X * accelerometer.X
					+ accelerometer.Y * accelerometer.Y
					+ accelerometer.Z * accelerometer.Z);
}
/*
 * writes the accelerometer to the standard output (USART2)
 */
void acc_Print(void) {
	// convert to mg
	stdComm_puts("accelerometer X:");
	stdComm_PrintValue(accelerometer.X * 1000);
	stdComm_puts(" Y:");
	stdComm_PrintValue(accelerometer.Y * 1000);
	stdComm_puts(" Z:");
	stdComm_PrintValue(accelerometer.Z * 1000);
	stdComm_puts(" %:");
	stdComm_PrintValue(accelerometer.magnitude * 100);
	usart_putcStdComm(0x0A);
}
/*
 * calibrates the accelerometer. Do to this the imu must be level.
 * This functions takes 100 values and averages them. Execution time
 * depends on the sensor update rate (see I2C_CYCLE_FREQUENCY in i2c.h)
 */
void acc_Calibrate(void) {
	log_LogFileEntry("calibrating accelerometer...");
	buzzer_Signal(1);
	int32_t sumX = 0, sumY = 0, sumZ = 0;
	uint8_t counter;
	hott.speak = SPEAK_CALIBRATE;
	for (counter = 0; counter < 100; counter++) {
//		// wait till reading cycle is finished
//		while (internali2c.autoReadingRunning)
//			;
		time_Waitms(10);
		// add raw values
		acc_Update();
		sumX += accelerometer.rawX;
		sumY += accelerometer.rawY;
		sumZ += accelerometer.rawZ;
//		// wait till the next reading cycle started
//		while (!internali2c.autoReadingRunning)
//			;
	}
	sumX /= 100;
	sumY /= 100;
	sumZ /= 100;
	config.accXOffset = sumX - 1.0f / ACC_SCALE * config.AccGyroMatrix[2][0];
	config.accYOffset = sumY - 1.0f / ACC_SCALE * config.AccGyroMatrix[2][1];
	config.accZOffset = sumZ - 1.0f / ACC_SCALE * config.AccGyroMatrix[2][2];
	log_LogFileEntry("calibrated accelerometer.");
	buzzer_Signal(1);
}
