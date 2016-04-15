#include "pressure.h"

/*
 * contains the temperature for a given ADC value
 * (index = ADC/128), ADC must be in range [128;3968)
 */
const float ADCtoTemp[32] = { -100, -32.3346, -22.1292, -15.5212, -10.4407,
		-6.2082, -2.5107, 0.8239, 3.9019, 6.7942, 9.5518, 12.2130, 14.8085,
		17.3640, 19.9023, 22.4441, 25.0101, 27.6210, 30.2995, 33.0703, 35.9624,
		39.0112, 42.2604, 45.7670, 49.6078, 53.8913, 58.7776, 64.5205, 71.5558,
		80.7309, 94.0479, 118.4554 };

/*
 * retrieves the temperature profile from the EEPROM
 */
void pressure_LoadTempProfile(void) {
	uint8_t version;
	eeprom_ReadByte(0x4FFF, &version);
	// set all pressure offset data to zero in case off missing barometer profile
	uint8_t i;
	for (i = 0;
			i < sizeof(pressure.tempOffset) / sizeof(pressure.tempOffset[0]);
			i++) {
		pressure.tempOffset[i] = 0;
	}
	if (version == 0xff) {
		log_LogFileEntry("no barometer profile data");
	} else if (version != PRESSURE_TEMPPROFILE_VERSION) {
		log_LogFileEntry("incompatible barometer profile data");
	} else {
		eeprom_ReadBlock(0x4000, (uint8_t*) &pressure.tempOffset,
				sizeof(pressure.tempOffset));
		log_LogFileEntry("barometer profile data loaded");
	}
}

/*
 * writes the temperature profile into the EEPROM
 */
void pressure_SaveTempProfile(void) {
	eeprom_WriteBlock(0x4000, (uint8_t*) &pressure.tempOffset,
			sizeof(pressure.tempOffset));
	eeprom_WriteByte(0x4FFF, PRESSURE_TEMPPROFILE_VERSION);
	log_LogFileEntry("barometer profile data saved");
}

/*
 * measures the barometer sensor offset at the current temperature.
 * !! Assumes constant height, don't use in flight !!
 * Should only be used during calibration on ground with a large
 * temperature range
 */
void pressure_UpdateTempProfile(void) {
	// remove sensor offset (values in tempOffset are floats
	// and therefore should be small to achieve good accuracy)
	int32_t biasFreeValue = pressure.ADCValue - pressure.zeroReference;
	// round current temperature
	int8_t floorTemp = (int8_t) (pressure.temp);
	int8_t CeilTemp = floorTemp + 1;
	float floorFactor = (float) CeilTemp - pressure.temp;
	float CeilFactor = pressure.temp - (float) floorTemp;
	// set floor index
	uint8_t floorIndex = floorTemp + 50;
	if (pressure.tempOffset[floorIndex] == 0) {
		// no data set yet -> use current value
		pressure.tempOffset[floorIndex] = biasFreeValue;
	} else {
		// already data set -> average with new data
		float alpha = 0.1 * floorFactor;
		pressure.tempOffset[floorIndex] = (float) biasFreeValue * alpha
				+ pressure.tempOffset[floorIndex] * (1 - alpha);
	}
	// set ceiling index
	uint8_t CeilIndex = CeilTemp + 50;
	if (pressure.tempOffset[CeilIndex] == 0) {
		// no data set yet -> use current value
		pressure.tempOffset[CeilIndex] = biasFreeValue;
	} else {
		// already data set -> average with new data
		float alpha = 0.1 * CeilFactor;
		pressure.tempOffset[CeilIndex] = (float) biasFreeValue * alpha
				+ pressure.tempOffset[CeilIndex] * (1 - alpha);
	}
}

/*
 * interpolates missing data in the temperature profile
 * TESTED -> works fine
 */
void pressure_InterpolateTempProfile(void) {
	uint8_t i;
	for (i = 0;
			i < sizeof(pressure.tempOffset) / sizeof(pressure.tempOffset[0]);
			i++) {
		if (pressure.tempOffset[i] == 0) {
			// no data available for this temperature
			// -> look for nearest available temperature and interpolate
			// seek lower value
			uint8_t distLow = 0;
			float valueLow = 0;
			uint8_t j = i;
			while (j > 0) {
				j--;
				distLow++;
				if (pressure.tempOffset[j] != 0) {
					valueLow = pressure.tempOffset[j];
					break;
				}
			}
			// seek higher value
			uint8_t distHigh = 0;
			float valueHigh = 0;
			j = i;
			while (j < 149) {
				j++;
				distHigh++;
				if (pressure.tempOffset[j] != 0) {
					valueHigh = pressure.tempOffset[j];
					break;
				}
			}
			// interpolate
			if (valueLow != 0 && valueHigh != 0) {
				// valid data found in both directions
				float weightLow = (float) distHigh / (distHigh + distLow);
				pressure.tempOffset[i] = valueLow * weightLow
						+ valueHigh * (1 - weightLow);
			} else if (valueLow != 0) {
				// only lower value found
				pressure.tempOffset[i] = valueLow;
			} else if (valueHigh != 0) {
				// only higher value found
				pressure.tempOffset[i] = valueHigh;
			}
		}
	}
}

/*
 * updates the height value
 */
void pressure_Update(void) {
	// calculate temperature
	int16_t rawADC = adc.raw[5];
	if (rawADC <= 128 || rawADC >= 3968) {
		// -> sensor probably not connected, set default value
		pressure.temp = 25;
	} else {
		uint8_t indexLow = rawADC / 128;
		uint8_t ADCToIndex = rawADC - (uint16_t) indexLow * 128;
		float temp = ADCtoTemp[indexLow];
		// interpolate linear between two datapoints
		temp += ((ADCtoTemp[indexLow + 1] - ADCtoTemp[indexLow]) * ADCToIndex)
				/ 128;
		// constrain temperature
		if (temp < -50)
			temp = -50;
		if (temp > 148)
			temp = 148;
		pressure.temp = temp;
	}
	// calculate temperature offset
	// interpolate offset from nearest temperature data
	uint8_t indexLow = (int) pressure.temp + 50;
	float weightLow = ((int) pressure.temp + 1) - pressure.temp;
	float tempOffset = pressure.tempOffset[indexLow] * weightLow
			+ pressure.tempOffset[indexLow + 1] * (1 - weightLow);
	// check for overflow
	if (!(pressure.rawADC & 0x00C00000)) {
		pressure.ADCValue =
				(pressure.rawADC & 0x00200000) ?
						pressure.rawADC - 0x00200000 : pressure.rawADC;
		pressure.ADCTempCompensated = pressure.ADCValue - tempOffset;
		pressure.height = (pressure.zeroReference - pressure.ADCTempCompensated)
				* PRESSURE_SCALE;
		pressure.timestamp = time_GetMillis();
		if (!pressure.valid)
			log_LogFileEntry("barometer data valid");
		pressure.valid = SET;
		if (logfile.flags.baroTempProfileActive
				&& pressure.TempCalibrationRunning == RESET) {
			// start temperature profile calibration
			pressure.TempCalibrationRunning = SET;
			// store current offset
			pressure.zeroReference = pressure.ADCValue;
			log_LogFileEntry("starting barometer calibration");
			buzzer_Signal(5);
		}
		if (pressure.TempCalibrationRunning == SET) {
			pressure_UpdateTempProfile();
		}
		if (pressure.TempCalibrationRunning == SET
				&& (attitude.roll > 45 * DEG_TO_RAD
						|| attitude.roll < -45 * DEG_TO_RAD
						|| attitude.pitch > 45 * DEG_TO_RAD
						|| attitude.pitch < -45 * DEG_TO_RAD)) {
			// calibration still running and board tilted
			// -> stop calibration
			// interpolate missing temperature data
			pressure_InterpolateTempProfile();
			// save temperature profile
			pressure_SaveTempProfile();
			// clear flag
			pressure.TempCalibrationRunning = RESET;
			logfile.flags.baroTempProfileActive = 0;
			log_LogFileEntry("board tilted: finished barometer calibration");
			buzzer_Signal(6);
		}
	} else {
		if (pressure.valid)
			log_LogFileEntry("ERROR: barometer data invalid");
		pressure.valid = RESET;
	}
}
/*
 * adjusts the pressure sensor offset. The parameter 'height'
 * must be the current height in meters
 */
void pressure_SetHeight(float height) {
	pressure.zeroReference = height / PRESSURE_SCALE
			+ pressure.ADCTempCompensated;
	pressure.height = height;
}

