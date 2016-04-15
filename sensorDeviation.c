#include "sensorDeviation.h"

struct Deviation deviation;

void deviation_StartMeasurement(uint8_t sensor, uint8_t samples) {
	deviation.nsamples = samples;
	deviation.sampleCounter = 0;
	deviation.sensor = sensor;
	deviation.measurementRunning = 1;
}

void deviation_NewData(void) {
	if (deviation.measurementRunning) {
		int32_t newdat = 0;
		switch (deviation.sensor) {
		case DEVIATION_ACC_X:
			newdat = accelerometer.rawX;
			break;
		case DEVIATION_ACC_Y:
			newdat = accelerometer.rawY;
			break;
		case DEVIATION_ACC_Z:
			newdat = accelerometer.rawZ;
			break;
		case DEVIATION_MAG_X:
			newdat = magnetometer.rawX;
			break;
		case DEVIATION_MAG_Y:
			newdat = magnetometer.rawY;
			break;
		case DEVIATION_MAG_Z:
			newdat = magnetometer.rawZ;
			break;
		case DEVIATION_GYRO_X:
			newdat = gyro.rawX;
			break;
		case DEVIATION_GYRO_Y:
			newdat = gyro.rawY;
			break;
		case DEVIATION_GYRO_Z:
			newdat = gyro.rawZ;
			break;
		case DEVIATION_BARO:
			newdat = pressure.ADCTempCompensated;
			break;
		}
		deviation.data[deviation.sampleCounter++] = newdat;
		if (deviation.sampleCounter == deviation.nsamples) {
			// finished taking data
			deviation.measurementRunning = 0;
			// calculate mean
			int32_t mean = 0;
			uint8_t i;
			for (i = 0; i < deviation.nsamples; i++) {
				mean += deviation.data[i];
			}
			mean /= deviation.nsamples;
			// calculate standard deviation
			uint32_t dev = 0;
			for (i = 0; i < deviation.nsamples; i++) {
				dev += (deviation.data[i] - mean) * (deviation.data[i] - mean);
			}
			dev /= deviation.nsamples;
			unsigned char message[9];
			message[0] = deviation.sensor;
			message[1] = (mean >> 24);
			message[2] = (mean >> 16) & 0xff;
			message[3] = (mean >> 8) & 0xff;
			message[4] = (mean) & 0xff;
			message[5] = (dev >> 24);
			message[6] = (dev >> 16) & 0xff;
			message[7] = (dev >> 8) & 0xff;
			message[8] = (dev) & 0xff;
			stdComm_SendMessage(MESSAGE_SENSOR_DEVIATION, message, 9);
		}
	}
}
