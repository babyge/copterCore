/*
 * handles data from external sensors (e.g. a potentiometer to determine the exact angle)
 */

#ifndef EXTERNALSENSORS_H_
#define EXTERNALSENSORS_H_

#include "hal.h"

#define EXTERNAL_DATA_BAUD 38400

#define DEG_TO_RAD		0.017453292f
#define RAD_TO_DEG		57.29578f

struct {
	// measured angle in radiant
	float angle;
	FlagStatus valid;
	uint32_t timestamp;
	// internal flags for the UART protocol
	FlagStatus transmissionStarted;
	uint8_t bytecount;
	uint8_t degreeBuffer;
} externalSensor;

void externalData_IncomingData(uint8_t data);


#endif /* EXTERNALSENSORS_H_ */
