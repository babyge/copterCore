#ifndef SENSORDEVIATION_H_
#define SENSORDEVIATION_H_

#include <stdint.h>
#include "stdcomm.h"

extern struct Deviation {
	int32_t data[256];
	uint8_t nsamples;
	uint8_t sampleCounter;
	uint8_t sensor;
#define DEVIATION_ACC_X		0
#define DEVIATION_ACC_Y		1
#define DEVIATION_ACC_Z		2
#define DEVIATION_MAG_X		3
#define DEVIATION_MAG_Y		4
#define DEVIATION_MAG_Z		5
#define DEVIATION_GYRO_X	6
#define DEVIATION_GYRO_Y	7
#define DEVIATION_GYRO_Z	8
#define DEVIATION_BARO		9

	uint8_t measurementRunning;
} deviation;

void deviation_StartMeasurement(uint8_t sensor, uint8_t samples);

void deviation_NewData(void);

#endif
