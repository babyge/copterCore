#include "distance.h"

// calculates distance(s) based on current ADC readings
void distance_Update(void) {
	// convert ADC value into voltage
	// 3V3/4095 -> 0.0008058608
	float voltage = ((float) adc.raw[1]) * 0.0008058608f;
	distance.bottom = distance_SharpGP2D12ToMeter(
			voltage) - DISTANCE_SENSOR_ABOVE_GROUND;
	distance.timestamp = time_GetMillis();
	if (distance.bottom < SHARP_MAXIMUM_DISTANCE) {
		distance.valid = SET;
	} else {
		// voltage outside of sensor specifications
		// -> no sensor connected or sensor error
		distance.valid = RESET;
	}
}

// converts a voltage from a GP2D12 IR-sensor into a distance
float distance_SharpGP2D12ToMeter(float voltage) {
	float dist = 0;
	if (voltage > SHARPGP2D12_CONSTANT_B) {
		dist = SHARPGP2D12_CONSTANT_A / (voltage - SHARPGP2D12_CONSTANT_B);
		if (dist > SHARP_MAXIMUM_DISTANCE)
			dist = SHARP_MAXIMUM_DISTANCE;
	} else {
		dist = SHARP_MAXIMUM_DISTANCE;
	}
	return dist;
}
