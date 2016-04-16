/*
 * handles data from external sensors (e.g. a potentiometer to determine the exact angle)
 */

#ifndef EXTERNALSENSORS_H_
#define EXTERNALSENSORS_H_

#include "hal.h"

#define EXTERNAL_DATA_BAUD 38400

struct ExternalSensor{
	// measured angle in radiant
	float angle;
	FlagStatus valid;
	uint32_t timestamp;
	// internal flags for the UART protocol
	FlagStatus transmissionStarted;
	uint8_t bytecount;
	uint8_t degreeBuffer;
};

extern struct ExternalSensor externalSensor;

void externalData_IncomingData(uint8_t data);


#endif /* EXTERNALSENSORS_H_ */
