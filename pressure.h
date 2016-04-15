/*
 * handles the communication with the external ADC connected to the pressure sensor.
 * The ADC is connected via 3 lines: CS, SCK and SDO.
 * Additionally the reference voltage and the negative input are tied to channel 1
 * and 2 of timer 9 (PWM + low pass -> adjustable gain and offset)
 */

#ifndef PRESSURE_H_
#define PRESSURE_H_

#include "hal.h"
#include "imu.h"
/*
 * sensor sensitivity: 46mV/kPa
 * pressure to height: ~0.0125kPa/m
 * ADC sensitivity: 2^21LSB/3.3V
 * -> 0.046*0.0125*2097152/3.3 LSB/m = 365.413 LSB/m
 * -> PRESSURE_SCALE = 1/365.413
 */
#define PRESSURE_SCALE		0.00273663064f

// defines are just for clarification, they shouldn't be changed
#define ADC_CS				GPIO_Pin_5
#define ADC_SCK				GPIO_Pin_3
#define ADC_SDO				GPIO_Pin_4

struct {
	// height in m
	float height;
	// temperature of the pressure sensor in °C
	float temp;
	// raw bit representation of the ADC data
	uint32_t rawADC;
	// ADC data stripped of over-/underflow indicator
	int32_t ADCValue;
	int32_t ADCTempCompensated;
	// lookup table for temperature offset
	// index = temp[°C]+50
	float tempOffset[150];
#define PRESSURE_TEMPPROFILE_VERSION 	1
	FlagStatus TempCalibrationRunning;
	// ADC value at a height of 0cm
	int32_t zeroReference;
	// Flag indicates valid data
	FlagStatus valid;
	// system time in milliseconds at the last data update
	uint32_t timestamp;
} pressure;

/*
 * retrieves the temperature profile from the EEPROM
 */
void pressure_LoadTempProfile(void);

/*
 * writes the temperature profile into the EEPROM
 */
void pressure_SaveTempProfile(void);

/*
 * measures the barometer sensor offset at the current temperature.
 * !! Assumes constant height, don't use in flight !!
 * Should only be used during calibration on ground with a large
 * temperature range
 */
void pressure_UpdateTempProfile(void);

/*
 * interpolates missing data in the temperature profile
 */
void pressure_InterpolateTempProfile(void);

/*
 * updates the height value
 */
void pressure_Update(void);

/*
 * adjusts the pressure sensor offset. The parameter 'height'
 * must be the current height in meters
 */
void pressure_SetHeight(float height);

#endif /* PRESSURE_H_ */
