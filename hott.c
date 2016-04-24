#include "hott.h"

struct Hott hott;

/*
 * updates HoTT data when requested
 */
void hott_Update(void) {
	if (hott.updateRequest & HOTT_UPDATE_GPS) {
		hott.gps.StartByte = 0x7C;
		hott.gps.Packet_ID = HOTTV4_GPS_PACKET_ID;
		hott.gps.WarnBeep = hott.speak;
		hott.speak = 0;
		hott.gps.SensorID = HOTTV4_GPS_SENSOR_ID;
		// direction and velocity
		hott.gps.Heading = gps.heading / 20;
		hott.gps.SpeedH = 0;
		hott.gps.SpeedL = gps.speed;
		// Latitude
		hott.gps.Lat_North = gps.latitude > 0 ? 0 : 1;
		uint8_t deg = gps.latitude / 10000000;
		uint32_t sec = ((gps.latitude % 10000000) * 6) / 100;
		uint8_t min = sec / 10000;
		sec = sec % 10000;
		uint16_t degMin = (deg * 100) + min;
		hott.gps.Lat_MinH = degMin >> 8;
		hott.gps.Lat_MinL = degMin & 0xff;
		hott.gps.Lat_SekH = sec >> 8;
		hott.gps.Lat_SekL = sec & 0xff;
		// Longitude
		hott.gps.Lon_East = gps.longitude > 0 ? 0 : 1;
		deg = gps.longitude / 10000000;
		sec = ((gps.longitude % 10000000) * 6) / 100;
		min = sec / 10000;
		sec = sec % 10000;
		degMin = (deg * 100) + min;
		hott.gps.Lon_MinH = degMin >> 8;
		hott.gps.Lon_MinL = degMin & 0xff;
		hott.gps.Lon_SekH = sec >> 8;
		hott.gps.Lon_SekL = sec & 0xff;
		// altitude data
		// altitude: height above ground in meters, m_sec: rate of ascend in cm
		uint16_t altitude = position.Z / 100.0f, m_sec = position.velocity.Z;
		hott.gps.AltitudeH = (altitude + 500) >> 8;
		hott.gps.AltitudeL = (altitude + 500) & 0xff;
		hott.gps.m_secH = (m_sec + 30000) >> 8;
		hott.gps.m_secL = (m_sec + 30000) & 0xff;
		hott.gps.m_3sec = 120;
		// satellite data
		hott.gps.NumOfSats = gps.satellites;

		hott.gps.EndByte = 0x7D;

		hott.updateRequest &= ~HOTT_UPDATE_GPS;
	}
	if (hott.updateRequest & HOTT_UPDATE_EAM) {
		hott.eam.StartByte = 0x7C;
		hott.eam.Packet_ID = HOTTV4_EAM_PACKET_ID;
		hott.eam.WarnBeep = hott.speak;
		hott.speak = 0;
		hott.eam.SensorID = HOTTV4_EAM_SENSOR_ID;
		// battery cells
		hott.eam.VoltageCell1 = 0;
		hott.eam.VoltageCell2 = 0;
		hott.eam.VoltageCell3 = 0;
		hott.eam.VoltageCell4 = 0;
		hott.eam.VoltageCell5 = 0;
//		hott.eam.VoltageCell1 = battery.cellVoltage[0] / 20;
//		hott.eam.VoltageCell2 = battery.cellVoltage[1] / 20;
//		hott.eam.VoltageCell3 = battery.cellVoltage[2] / 20;
//		hott.eam.VoltageCell4 = battery.cellVoltage[3] / 20;
//		hott.eam.VoltageCell5 = battery.cellVoltage[4] / 20;
		// dummy temperatures -> 0°C
		hott.eam.Temperature1 = 20;
		hott.eam.Temperature2 = 20;
		// altitude data
		// altitude: height above ground in meters, m_sec: rate of ascend in cm
		uint16_t altitude = position.Z / 100.0f, m_sec = position.velocity.Z;
		hott.eam.AltitudeH = (altitude + 500) >> 8;
		hott.eam.AltitudeL = (altitude + 500) & 0xff;
		hott.eam.m_secH = (m_sec + 30000) >> 8;
		hott.eam.m_secL = (m_sec + 30000) & 0xff;
		hott.eam.m_3sec = 120;

		// battery information
		hott.eam.CurrentH = (battery.current / 100) >> 8;
		hott.eam.CurrentL = (battery.current / 100) & 0xff;
		hott.eam.InputVoltageH = (battery.voltage / 100) >> 8;
		hott.eam.InputVoltageL = (battery.voltage / 100) & 0xff;
		hott.eam.CapacityH = (battery.usedCapacity / 10) >> 8;
		hott.eam.CapacityL = (battery.usedCapacity / 10) & 0xff;

		// flight time
		hott.eam.FlightTimeMinutes = flightState.flightTimeMinutes;
		hott.eam.FlightTimeSeconds = flightState.flightTimeSeconds;

		hott.eam.EndByte = 0x7D;

		hott.updateRequest &= ~HOTT_UPDATE_EAM;
	}
	if (hott.updateRequest & HOTT_UPDATE_GEM) {
		// HoTT general data
		hott.gem.StartByte = 0x7C;
		hott.gem.Packet_ID = HOTTV4_GEM_PACKET_ID;
		hott.gem.WarnBeep = hott.speak;
		hott.speak = 0;
		hott.gem.SensorID = HOTTV4_GEM_SENSOR_ID;
		// battery cells
		hott.eam.VoltageCell1 = 0;
		hott.eam.VoltageCell2 = 0;
		hott.eam.VoltageCell3 = 0;
		hott.eam.VoltageCell4 = 0;
		hott.eam.VoltageCell5 = 0;
//		hott.eam.VoltageCell1 = battery.cellVoltage[0] / 20;
//		hott.eam.VoltageCell2 = battery.cellVoltage[1] / 20;
//		hott.eam.VoltageCell3 = battery.cellVoltage[2] / 20;
//		hott.eam.VoltageCell4 = battery.cellVoltage[3] / 20;
//		hott.eam.VoltageCell5 = battery.cellVoltage[4] / 20;
		// altitude data
		// altitude: height above ground in meters, m_sec: rate of ascend in cm
		uint16_t altitude = position.Z / 100.0f, m_sec = position.velocity.Z;
		hott.gem.AltitudeH = (altitude + 500) >> 8;
		hott.gem.AltitudeL = (altitude + 500) & 0xff;
		hott.gem.m_secH = (m_sec + 30000) >> 8;
		hott.gem.m_secL = (m_sec + 30000) & 0xff;
		hott.gem.m_3sec = 120;
		// battery information
		hott.gem.CurrentH = (battery.current / 100) >> 8;
		hott.gem.CurrentL = (battery.current / 100) & 0xff;
		hott.gem.InputVoltageH = (battery.voltage / 100) >> 8;
		hott.gem.InputVoltageL = (battery.voltage / 100) & 0xff;
		hott.gem.CapacityH = (battery.usedCapacity / 10) >> 8;
		hott.gem.CapacityL = (battery.usedCapacity / 10) & 0xff;
		// TODO speed data
//		if (battery.balancerAvailable) {
//			// lowest cell
//			uint16_t minVoltage = 0xFFFF;
//			uint8_t cellnumber, i;
//			for (i = 0; i < 5; i++) {
//				if (battery.cellVoltage[i] < minVoltage
//						&& battery.cellVoltage[i] > 0) {
//					minVoltage = battery.cellVoltage[i];
//					cellnumber = i + 1;
//				}
//			}
//			hott.gem.LowestCellNumber = cellnumber;
//			hott.gem.LowestCellVoltage = minVoltage / 20;
//		} else {
		hott.gem.LowestCellNumber = 0;
		hott.gem.LowestCellVoltage = 0;
//		}
		// TODO pressure
		hott.gem.EndByte = 0x7D;

		hott.updateRequest &= ~HOTT_UPDATE_GEM;
	}
	if (hott.updateRequest & HOTT_UPDATE_VARIO) {
		hott.vario.StartByte = 0x7C;
		hott.vario.Packet_ID = HOTTV4_VARIO_PACKET_ID;
		hott.vario.WarnBeep = hott.speak;
		hott.speak = 0;
		hott.vario.SensorID = HOTTV4_VARIO_SENSOR_ID;
		// altitude data
		// altitude: height above ground in meters, m_sec: rate of ascend in cm
		uint16_t altitude = position.Z / 100.0f, m_sec = position.velocity.Z;
		hott.vario.AltitudeH = (altitude + 500) >> 8;
		hott.vario.AltitudeL = (altitude + 500) & 0xff;
		hott.vario.m_secH = (m_sec + 30000) >> 8;
		hott.vario.m_secL = (m_sec + 30000) & 0xff;

		hott.vario.EndByte = 0x7D;

		hott.updateRequest &= ~HOTT_UPDATE_VARIO;
	}
	if (hott.updateRequest & HOTT_UPDATE_TEXT) {
		hott.text.StartByte = 0x7B;
		hott.text.WarnBeep = hott.speak;
		hott.speak = 0;

		hott_KeyPressed();
		hott_SetTextBuffer();

		hott.text.EndByte = 0x7D;

		hott.updateRequest &= ~HOTT_UPDATE_TEXT;
	}
}
/*
 * processes received bytes
 */
void hott_IncomingData(uint8_t data) {
	switch (hott.mode) {
	case HOTT_IDLE:
		// switch to non-idle state
		if (data == 0x80)
			hott.mode = HOTT_BINARY;
		else if (data == 0x7F)
			hott.mode = HOTT_TEXT;
		break;
	case HOTT_BINARY:
		switch (data) {
		case HOTTV4_GPS_PACKET_ID:
			hott.updateRequest |= HOTT_UPDATE_GPS;
			hott.outputPointer = (uint8_t*) &hott.gps;
			hott.bytes = 44;
			hott.delay = 5;
			hott.dataToBeSend = SET;
			hott.mode = HOTT_IDLE;
			break;

		case HOTTV4_GEM_PACKET_ID:
			hott.updateRequest |= HOTT_UPDATE_GEM;
			hott.outputPointer = (uint8_t*) &hott.gem;
			hott.bytes = 44;
			hott.delay = 5;
			hott.dataToBeSend = SET;
			hott.mode = HOTT_IDLE;
			break;

		case HOTTV4_EAM_PACKET_ID:
			hott.updateRequest |= HOTT_UPDATE_EAM;
			hott.outputPointer = (uint8_t*) &hott.eam;
			hott.bytes = 44;
			hott.delay = 5;
			hott.dataToBeSend = SET;
			hott.mode = HOTT_IDLE;
			break;

		case HOTTV4_VARIO_PACKET_ID:
			hott.updateRequest |= HOTT_UPDATE_VARIO;
			hott.outputPointer = (uint8_t*) &hott.vario;
			hott.bytes = 44;
			hott.delay = 5;
			hott.dataToBeSend = SET;
			hott.mode = HOTT_IDLE;
			break;
		default:
			hott.mode = HOTT_IDLE;
		}
		break;
	case HOTT_TEXT:
		if ((data & 0xf0) != 0) {
			hott.keyPressed = data & 0x0F;
			hott.updateRequest |= HOTT_UPDATE_TEXT;
			hott.text.Packed_ID = data & 0xF0;
			hott.outputPointer = (uint8_t*) &hott.text;
			hott.bytes = 172;
			hott.delay = 5;
			hott.dataToBeSend = SET;
			hott.mode = HOTT_IDLE;
		} else {
			hott.mode = HOTT_IDLE;
		}
		break;
	}
}

/*
 * handles user inputs in the text menu
 */
void hott_KeyPressed(void) {
	if (hott.keyPressed == HOTT_RIGHT) {
		// one menu to the right
		if (hott.text.selectedMenu < HOTT_N_MENUS - 1) {
			hott.text.selectedMenu++;
		}
	}
	if (hott.keyPressed == HOTT_LEFT) {
		// one menu to the left
		if (hott.text.selectedMenu > 0) {
			hott.text.selectedMenu--;
		}
	}
}

/*
 * fills the text buffer with valid data
 */
void hott_SetTextBuffer(void) {
	hott_ClearText();
	switch (hott.text.selectedMenu) {
	case HOTT_MENU_OVERVIEW:
		hott_Print(0, 0, "   COPTER OVERVIEW   ", 1);
		// Accelerometer
		hott_Print(1, 1, "Accelerometer:", 0);
		if (accelerometer.valid)
			hott_Print(17, 1, "OK", 0);
		else
			hott_Print(16, 1, "ERR", 1);
		// Gyro
		hott_Print(1, 2, "Gyro:", 0);
		if (gyro.valid)
			hott_Print(17, 2, "OK", 0);
		else
			hott_Print(16, 2, "ERR", 1);
		// Magnetometer
		hott_Print(1, 3, "Magnetometer:", 0);
		if (magnetometer.valid)
			hott_Print(17, 3, "OK", 0);
		else
			hott_Print(16, 3, "ERR", 1);
		// Barometer
		hott_Print(1, 4, "Barometer:", 0);
		if (pressure.valid)
			hott_Print(17, 4, "OK", 0);
		else
			hott_Print(16, 4, "ERR", 1);
		// GPS
		hott_Print(1, 5, "GPS:", 0);
		if (gps.FixValid)
			hott_Print(17, 5, "OK", 0);
		else if (gps.available)
			hott_Print(13, 5, "NO FIX", 1);
		else
			hott_Print(13, 5, "NO GPS", 1);
		// Motor
		hott_Print(1, 6, "Motors:", 0);
		if (!motor.genericError)
			hott_Print(17, 6, "OK", 0);
		else
			hott_Print(16, 6, "ERR", 1);
		// Battery
		hott_Print(1, 7, "Battery:", 0);
		if (battery_GetStatus() == SUCCESS)
			hott_Print(17, 7, "OK", 0);
		else
			hott_Print(16, 7, "ERR", 1);
		break;
	case HOTT_MENU_ACC:
		hott_Print(0, 0, "    ACCELEROMETER    ", 1);
		hott_Print(0, 1, "State:", 0);
		if (accelerometer.valid)
			hott_Print(17, 1, "OK", 0);
		else
			hott_Print(16, 1, "ERR", 1);
		hott_Print(0, 3, "X:", 0);
		hott_PrintFloat(2, 3, accelerometer.X, 0);
		hott_PrintInt(11, 3, accelerometer.rawX, 0);
		hott_Print(0, 4, "Y:", 0);
		hott_PrintFloat(2, 4, accelerometer.Y, 0);
		hott_PrintInt(11, 4, accelerometer.rawY, 0);
		hott_Print(0, 5, "Z:", 0);
		hott_PrintFloat(2, 5, accelerometer.Z, 0);
		hott_PrintInt(11, 5, accelerometer.rawZ, 0);
		hott_Print(0, 6, "M:", 0);
		hott_PrintFloat(2, 6, accelerometer.magnitude, 0);
		break;
	case HOTT_MENU_GYRO:
		hott_Print(0, 0, "      GYROSCOPE      ", 1);
		hott_Print(0, 1, "State:", 0);
		if (gyro.valid)
			hott_Print(17, 1, "OK", 0);
		else
			hott_Print(16, 1, "ERR", 1);
		hott_Print(0, 3, "X:", 0);
		hott_PrintFloat(2, 3, gyro.X, 0);
		hott_PrintInt(11, 3, gyro.rawX, 0);
		hott_Print(0, 4, "Y:", 0);
		hott_PrintFloat(2, 4, gyro.Y, 0);
		hott_PrintInt(11, 4, gyro.rawY, 0);
		hott_Print(0, 5, "Z:", 0);
		hott_PrintFloat(2, 5, gyro.Z, 0);
		hott_PrintInt(11, 5, gyro.rawZ, 0);
		break;
	case HOTT_MENU_MAG:
		hott_Print(0, 0, "    MAGNETOMETER     ", 1);
		hott_Print(0, 1, "State:", 0);
		if (magnetometer.valid)
			hott_Print(17, 1, "OK", 0);
		else
			hott_Print(16, 1, "ERR", 1);
		hott_Print(0, 3, "X:", 0);
		hott_PrintFloat(2, 3, magnetometer.X, 0);
		hott_PrintInt(11, 3, magnetometer.rawX, 0);
		hott_Print(0, 4, "Y:", 0);
		hott_PrintFloat(2, 4, magnetometer.Y, 0);
		hott_PrintInt(11, 4, magnetometer.rawY, 0);
		hott_Print(0, 5, "Z:", 0);
		hott_PrintFloat(2, 5, magnetometer.Z, 0);
		hott_PrintInt(11, 5, magnetometer.rawZ, 0);
		hott_Print(0, 6, "M:", 0);
		hott_PrintFloat(2, 6, magnetometer.magnitude, 0);
		break;
	case HOTT_MENU_BARO:
		hott_Print(0, 0, "      BAROMETER      ", 1);
		hott_Print(0, 1, "State:", 0);
		if (pressure.valid)
			hott_Print(17, 1, "OK", 0);
		else
			hott_Print(16, 1, "ERR", 1);
		hott_Print(0, 2, "Height:", 0);
		hott_PrintFloat(12, 2, pressure.height, 0);
		hott_Print(0, 3, "Raw:", 0);
		hott_PrintInt(12, 3, pressure.ADCValue, 0);
		hott_Print(0, 5, "Temp:", 0);
		hott_PrintFloat(12, 5, pressure.temp, 0);
		hott_Print(0, 6, "TempOffset:", 0);
		hott_PrintInt(12, 6, pressure.ADCValue - pressure.ADCTempCompensated,
				0);
		break;
	case HOTT_MENU_MOTORS:
		hott_Print(0, 0, "        MOTORS       ", 1);
		uint8_t i;
		for (i = 0; i < MOTOR_MAXNUM; i++) {
			uint8_t x, y;
			if (i < 6) {
				y = i + 1;
				x = 0;
			} else {
				y = i - 5;
				x = 12;
			}
			hott_Print(x, y, "Mot", 0);
			hott.text.text[x + y * 21 + 3] = i / 10 + '0';
			hott.text.text[x + y * 21 + 4] = i % 10 + '0';
			hott.text.text[x + y * 21 + 5] = ':';

			if (i >= config.motor.num) {
				hott_Print(x + 6, y, "---", 0);
			} else if (motor.error[i]) {
				hott_Print(x + 6, y, "ERR", 1);
			} else {
				hott_Print(x + 7, y, "OK", 0);
			}
		}
		hott_Print(0, 7, "PPM", config.motor.output & MOTOR_USE_PPM);
		hott_Print(9, 7, "I2C", config.motor.output & MOTOR_USE_I2C);
		hott_Print(18, 7, "CAN", config.motor.output & MOTOR_USE_CAN);
	}
}

/*
 * Clears the entire text buffer
 */
void hott_ClearText(void) {
	uint8_t i;
	for (i = 0; i < 168; i++) {
		hott.text.text[i] = ' ';
	}
}

/*
 * writes a string into the text buffer
 */
void hott_Print(uint8_t x, uint8_t y, char *text, uint8_t inverted) {
	while (*text != 0) {
		hott.text.text[x + y * 21] = *text;
		if (inverted)
			hott.text.text[x + y * 21] |= 0x80;
		x++;
		text++;
	}
}

void hott_PrintInt(uint8_t x, uint8_t y, int32_t n, uint8_t inverted) {
	char buf[10];
	char *b = buf;
	b = itoASCII(n, b);
	*b = 0;
	hott_Print(x, y, buf, inverted);
}

void hott_PrintFloat(uint8_t x, uint8_t y, float f, uint8_t inverted) {
	char buf[10];
	char *b = buf;
	b = ftoa(f, b);
	*b = 0;
	hott_Print(x, y, buf, inverted);
}
