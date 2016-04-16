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
		hott.text.startbyte = 0x7C;
		hott.text.alarm = hott.speak;
		hott.speak = 0;
		hott.text.text[0] = 'T';
		hott.text.text[1] = 'e';
		hott.text.text[2] = 's';
		hott.text.text[3] = 't';
		hott.text.endbyte = 0x7D;

		hott.updateRequest &= ~HOTT_UPDATE_TEXT;
	}
}
/*
 * processes received bytes
 */
void hott_IncomingData(uint8_t data) {
	if (hott.mode == HOTT_IDLE) {
		// switch to non-idle state
		if (data == 0x80)
			hott.mode = HOTT_BINARY;
		else if (data == 0x7F)
			hott.mode = HOTT_TEXT;
	} else if (hott.mode == HOTT_BINARY) {
		switch (data) {
		case HOTTV4_GPS_PACKET_ID:
			hott.updateRequest |= HOTT_UPDATE_GPS;
			hott.outputPointer = (uint8_t*) &hott.gps;
			hott.bytes = 44;
			hott.delay = 5;
			hott.dataToBeSend = SET;
			break;

		case HOTTV4_GEM_PACKET_ID:
			hott.updateRequest |= HOTT_UPDATE_GEM;
			hott.outputPointer = (uint8_t*) &hott.gem;
			hott.bytes = 44;
			hott.delay = 5;
			hott.dataToBeSend = SET;
			break;

		case HOTTV4_EAM_PACKET_ID:
			hott.updateRequest |= HOTT_UPDATE_EAM;
			hott.outputPointer = (uint8_t*) &hott.eam;
			hott.bytes = 44;
			hott.delay = 5;
			hott.dataToBeSend = SET;
			break;

		case HOTTV4_VARIO_PACKET_ID:
			hott.updateRequest |= HOTT_UPDATE_VARIO;
			hott.outputPointer = (uint8_t*) &hott.vario;
			hott.bytes = 44;
			hott.delay = 5;
			hott.dataToBeSend = SET;
			break;
		default:
			hott.mode = HOTT_IDLE;
		}
	} else if (hott.mode == HOTT_TEXT) {
		if ((data & 0xf0) != 0) {
			hott.keyPressed = data & 0x0F;
			hott.updateRequest |= HOTT_UPDATE_TEXT;
			hott.outputPointer = (uint8_t*) &hott.text;
			hott.bytes = 172;
			hott.delay = 5;
			hott.dataToBeSend = SET;
		} else {
			hott.mode = HOTT_IDLE;
		}
	}
}
