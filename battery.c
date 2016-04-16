#include "battery.h"

struct Battery battery;

/*
 * calculates the number of battery cells
 */
void battery_CalcCellNumber(void) {
//	if (battery.balancerAvailable == SET) {
//		// difference between sum over single cells nearly identical to voltage
//		// -> use balancer info to determine number of cells
//		battery.cells = 1;
//		while (battery.cellVoltage[battery.cells] > 0) {
//			battery.cells++;
//		}
//		battery.cells--;
//	} else {
//		// no balancer info available, use cell voltage to calculate number of cells
//		battery.cells = 0;
	while (battery.voltage > battery.cells * BATTERY_MAXCELL) {
		battery.cells++;
	}
//	}
}
/*
 * calculates the battery values based on the ADC readings and current
 * data from the motordrivers (if available)
 */
void battery_Update(void) {
	/*
	 * ADC receives the battery voltage/11
	 * -> (1/11)/3.3*4095 = ~112.81 LSB/V
	 *
	 * ADC value is multiplied by (2420/273) to get to 1mV/LSB
	 */
	uint32_t voltage = ((int32_t) adc.raw[0] * 2420) / 273;
	// averaging result
	battery.voltage = ((int32_t) battery.voltage * BATTERY_LOWPASS + voltage)
			/ (BATTERY_LOWPASS + 1);
//	uint32_t cellVoltage[5];
//	if (battery.rawADC[1] >= BATTERY_ADCTHRESHOLD) {
//		cellVoltage[0] = ((int32_t) battery.rawADC[1] * 2420) / 273;
//		// averaging voltage measurement
//		battery.cellVoltage[0] = ((int32_t) battery.cellVoltage[0]
//				* BATTERY_LOWPASS + cellVoltage[0]) / (BATTERY_LOWPASS + 1);
//	} else {
//		cellVoltage[0] = 0;
//		battery.cellVoltage[0] = 0;
//	}
//	// calculate cell voltages as the difference between the current and
//	// the previous cell
//	uint8_t i;
//	for (i = 1; i < 5; i++) {
//		// skip not existent cells
//		if (battery.rawADC[i + 1] >= BATTERY_ADCTHRESHOLD) {
//			cellVoltage[i] = ((int32_t) (battery.rawADC[i + 1]
//					- battery.rawADC[i]) * 2420) / 273;
//			// averaging voltage measurement
//			battery.cellVoltage[i] = ((int32_t) battery.cellVoltage[i]
//					* BATTERY_LOWPASS + cellVoltage[i]) / (BATTERY_LOWPASS + 1);
//		} else {
//			cellVoltage[i] = 0;
//			battery.cellVoltage[i] = 0;
//		}
//	}
//
//	// check whether the balancer is available
//	uint32_t balancerSum = 0;
//	for (i = 0; i < 5; i++) {
//		balancerSum += battery.cellVoltage[i];
//	}
//	// check whether balancer is connected
//	if (battery.voltage - balancerSum < 500
//			&& balancerSum - battery.voltage < 500)
//		battery.balancerAvailable = SET;
//	else
//		battery.balancerAvailable = RESET;
	/****************************************************
	 * calculate current and capacity
	 ***************************************************/
	// calculate current based on motor data
	uint8_t i;
	uint16_t current = BATTERY_NOLOAD_CURRENT;
	for (i = 0; i < config.motor.num; i++) {
		current += motor.current[i];
	}
	battery.current = current;
	// update used capacity
	int32_t currentTime = time_GetMillis();
	float timediff = (currentTime - battery.timestamp) * 0.001f;
	battery.timestamp = currentTime;
	static float mABuffer;
	mABuffer += (float) current * timediff * (1.0f / 3600);
	if (mABuffer >= 1) {
		battery.usedCapacity++;
		mABuffer -= 1.0f;
	}
}
/*
 * checks the battery
 */
ErrorStatus battery_GetStatus(void) {
	ErrorStatus ret;
//	if (battery.balancerAvailable == SET) {
//		// check every single cell
//		uint8_t i;
//		for (i = 0; i < 5; i++) {
//			if ((battery.cellVoltage[i] > 0)
//					&& (battery.cellVoltage[i] < BATTERY_MINCELL))
//				ret = ERROR;
//		}
//		ret = SUCCESS;
//	} else {
		// check overall battery voltage
		if (battery.voltage < battery.cells * BATTERY_MINCELL)
			ret = ERROR;
		else
			ret = SUCCESS;
//	}
	if (ret == ERROR && time_TimerElapsed(&battery.errorMessageTimer)) {
		time_SetTimer(&battery.errorMessageTimer, 1000);
		log_LogFileEntry("WARNING: low battery");
	}
	return ret;
}
/*
 * writes the battery state to the standard output (USART2)
 */
void battery_Print(void) {
	stdComm_puts("battery:");
	stdComm_PrintValue(battery.voltage);
	stdComm_puts("mV\n");
//	stdComm_puts("mV\nCell1:");
//	stdComm_PrintValue(battery.cellVoltage[0]);
//	stdComm_puts("mV\nCell2:");
//	stdComm_PrintValue(battery.cellVoltage[1]);
//	stdComm_puts("mV\nCell3:");
//	stdComm_PrintValue(battery.cellVoltage[2]);
//	stdComm_puts("mV\nCell4:");
//	stdComm_PrintValue(battery.cellVoltage[3]);
//	stdComm_puts("mV\nCell5:");
//	stdComm_PrintValue(battery.cellVoltage[4]);
//	stdComm_puts("mV\n");
}

