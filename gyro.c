/*
 * gyro.c
 *
 *  Created on: Oct 31, 2013
 *      Author: jan
 */

#include "gyro.h"

struct Gyro gyro;

/*
 * writes the values from gyroDataRX into the gyro structur
 */
void gyro_Update(void) {
	gyro.timestampUpdate = time_GetMillis();
	// buffer values to detect 'frozen' data
	int16_t bufX = gyro.rawX;
	int16_t bufY = gyro.rawY;
	int16_t bufZ = gyro.rawZ;
	gyro.rawX = gyro.rawValues[1] + (gyro.rawValues[0] << 8);
	gyro.rawY = gyro.rawValues[3] + (gyro.rawValues[2] << 8);
	gyro.rawZ = gyro.rawValues[5] + (gyro.rawValues[4] << 8);
	if (bufX != gyro.rawX || bufY != gyro.rawY || bufZ != gyro.rawZ) {
		// data has changed
		gyro.timestampChange = gyro.timestampUpdate;
		if (gyro.valid != SET)
			log_LogFileEntry("gyro data valid");
		gyro.valid = SET;
	}
	float gyroX = -(gyro.rawX - config.gyroXOffset);
	float gyroY = -(gyro.rawY - config.gyroYOffset);
	float gyroZ = -(gyro.rawZ - config.gyroZOffset);

	gyroX *= GYRO_SCALE;
	gyroY *= GYRO_SCALE;
	gyroZ *= GYRO_SCALE;

	/*
	 * matrix rotation
	 */
	float rX = config.AccGyroMatrix[0][0] * gyroX
			+ config.AccGyroMatrix[0][1] * gyroY
			+ config.AccGyroMatrix[0][2] * gyroZ;
	float rY = config.AccGyroMatrix[1][0] * gyroX
			+ config.AccGyroMatrix[1][1] * gyroY
			+ config.AccGyroMatrix[1][2] * gyroZ;
	float rZ = config.AccGyroMatrix[2][0] * gyroX
			+ config.AccGyroMatrix[2][1] * gyroY
			+ config.AccGyroMatrix[2][2] * gyroZ;

	gyro.X = gyro.X * (1 - config.alphaGyro) + rX * config.alphaGyro;
	gyro.Y = gyro.Y * (1 - config.alphaGyro) + rY * config.alphaGyro;
	gyro.Z = gyro.Z * (1 - config.alphaGyro) + rZ * config.alphaGyro;
}
/*
 * writes the gyro to the standard output (USART2)
 */
void gyro_Print(void) {
	// convert from rad/s to deg/s
	stdComm_puts("gyro X:");
	stdComm_PrintValue(gyro.X * RAD_TO_DEG);
	stdComm_puts(" Y:");
	stdComm_PrintValue(gyro.Y * RAD_TO_DEG);
	stdComm_puts(" Z:");
	stdComm_PrintValue(gyro.Z * RAD_TO_DEG);
	usart_putcStdComm(0x0A);
}
/*
 * calibrates the gyro. Do to this the imu must not be moving.
 * This functions takes 100 values and averages them. Execution time
 * depends on the sensor update rate (see I2C_CYCLE_FREQUENCY in i2c.h)
 */
void gyro_Calibrate(void) {
	log_LogFileEntry("calibrating gyro...");
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
		gyro_Update();
		sumX += gyro.rawX;
		sumY += gyro.rawY;
		sumZ += gyro.rawZ;
//		// wait till the next reading cycle started
//		while (!internali2c.autoReadingRunning)
//			;
	}
	sumX /= 100;
	sumY /= 100;
	sumZ /= 100;
	config.gyroXOffset = sumX;
	config.gyroYOffset = sumY;
	config.gyroZOffset = sumZ;
	log_LogFileEntry("calibrated gyro.");
	buzzer_Signal(1);
}
