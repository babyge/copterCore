/*
 * This file contains routines to monitor the battery state
 *
 * Pin coniguration:
 * PA0: battery
 * PA1: balancer pin 2 (cell 1)
 * PA2: balancer pin 3 (cell 1+2)
 * PA3: balancer pin 4 (cell 1+2+3)
 * PA4: balancer pin 5 (cell 1+2+3+4)
 * PA5: balancer pin 6 (cell 1+2+3+4+5)
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include "hal.h"
#include "stdcomm.h"
#include "motor.h"

// minimal and maximal cell voltage in mV
#define BATTERY_MINCELL			3300
#define BATTERY_MAXCELL			4300

// cells below this voltage are assumed to be non-existent (in mV)
#define BATTERY_CELLTHRESHOLD	2000
#define BATTERY_ADCTHRESHOLD 	((BATTERY_CELLTHRESHOLD*273UL)/2420)

// low pass filter
// 0: no filter
// >0: filter activated, higher value means lower cut-off frequency
#define BATTERY_LOWPASS			10

// current in mA consumed by all electronics (without motor load)
#define BATTERY_NOLOAD_CURRENT	500

struct Battery {
	// overall battery voltage (mV)
	uint16_t voltage;
//	// voltages of the single battery cells (mV)
//	uint16_t cellVoltage[5];
	// number of detected battery cells
	uint8_t cells;
	// raw ADC values
//	uint16_t rawADC[6];
//	FlagStatus balancerAvailable;
	uint32_t errorMessageTimer;
	uint16_t usedCapacity;
	uint16_t current;
	uint32_t timestamp;
};

extern struct Battery battery;

/*
 * calculates the number of battery cells
 */
void battery_CalcCellNumber(void);
/*
 * calculates the battery values based on the ADC readings and current
 * data from the motordrivers (if available)
 */
void battery_Update(void);
/*
 * checks the battery
 * @return 	ERROR, when the battery voltage is too low
 * 			SUCCESS, otherwise
 */
ErrorStatus battery_GetStatus(void);
/*
 * writes the battery state to the standard output (USART2)
 */
void battery_Print(void);


#endif /* BATTERY_H_ */
