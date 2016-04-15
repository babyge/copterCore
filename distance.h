#ifndef DISTANCE_H_
#define DISTANCE_H_

#include "hal.h"

// constants do convert voltage into distance
// see: http://rn-wissen.de/wiki/index.php?title=Sensorarten#Sharp_GP2D12
// measurement1: 20cm -> 1,43V
// measurement2: 60cm -> 0,54V
#define SHARPGP2D12_CONSTANT_A			0.267
#define SHARPGP2D12_CONSTANT_B			0.095

#define DISTANCE_SENSOR_ABOVE_GROUND	0.13f
#define SHARP_MAXIMUM_DISTANCE			1.0f

struct {
	float bottom;
	uint32_t timestamp;
	FlagStatus valid;
} distance;

// calculates distance(s) based on current ADC readings
void distance_Update(void);

// converts a voltage from a GP2D12 IR-sensor into a distance
float distance_SharpGP2D12ToMeter(float voltage);

#endif
