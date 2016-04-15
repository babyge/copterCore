#include "externalSensors.h"

void externalData_IncomingData(uint8_t data) {
	if (data == 0x01 && externalSensor.transmissionStarted == RESET) {
		// start of transmission detected
		externalSensor.transmissionStarted = SET;
		externalSensor.bytecount = 0;
	} else if (externalSensor.transmissionStarted == SET) {
		if (externalSensor.bytecount == 0) {
			// first transmission byte: tilt angle in degree (int8_t)
			externalSensor.degreeBuffer = data;
		} else if (externalSensor.bytecount == 1) {
			// second transmission byte
			int16_t degree = (externalSensor.degreeBuffer << 8) + (data & 0xC0);
			// 0.00390625 = 1/256
			externalSensor.angle = degree * 0.00390625f * DEG_TO_RAD;
			if(data&0x01){
				if(externalSensor.valid==RESET)
					log_LogFileEntry("external tilt angle available");
				externalSensor.valid = SET;
			} else {
				if(externalSensor.valid==SET)
					log_LogFileEntry("WARNING: external tilt angle invalid");
				externalSensor.valid = RESET;
			}
			externalSensor.timestamp = time_GetMillis();
		}
		externalSensor.bytecount++;
		if(externalSensor.bytecount>=2)
			externalSensor.transmissionStarted = RESET;
	}
}
