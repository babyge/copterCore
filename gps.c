#include "gps.h"

/*
 * initializes the GPS modul (should be called some time after power-up, e.g. after
 * 2 seconds)
 */
void gps_Init(void) {
	gps.homePosition.valid = RESET;
	gps.FixValid = RESET;
	gps.available = RESET;
	gps.sentenceComplete = RESET;
	gps.sentenceStarted = RESET;
	led_Cmd(1, LED_ON);
	led_Cmd(2, LED_ON);
	// try to access the module at its default baudrate
	// (if the baudrate has already been changed this command
	// has no effect)
	usart_GPSSetBaudrate(9600);
	// switch to 38400 baud
	gps_SendCommand(GPS_COMMAND_38400);
	time_Waitms(100);
	usart_GPSSetBaudrate(38400);
	// switch to custom binary mode
	gps_SendCommand(GPS_COMMAND_BINARY);
	time_Waitms(100);
	// set update rate to 10HZ
	gps_SendCommand(GPS_COMMAND_10HZ);
}

/*
 * must be called whenever a new sign is read from the gps modul
 */
void gps_IncomingData(uint8_t data) {
	if (gps.sentenceStarted) {
		gps.buffer[gps.bytecount++] = data;
		if (gps.bytecount >= 31) {
			gps.sentenceComplete = SET;
			gps.sentenceStarted = RESET;
		}
	} else if (data == 0xDD
			&& gps.lastChar
					== 0xD1 /*was supposed to be 0xD0 (error in manual?)*/) {
		gps.bytecount = 0;
		gps.sentenceStarted = SET;
	}
	gps.lastChar = data;
}
/*
 * checks whether a complete GPS sentence has been received
 */
FlagStatus gps_FrameComplete(void) {
	return gps.sentenceComplete;
}

/*
 * processes received GPS sentences
 */
void gps_EvaluateFrame(void) {
	if (gps.sentenceComplete != SET)
		return;
	gps.sentenceComplete = RESET;
	/*
	 * frame content
	 *
	 * every data is stored low byte to high byte
	 *
	 * content			| number of bytes	| byte offset
	 * -----------------+-------------------+------------
	 * payload			|	1				|	0
	 * latitude			|	4				|	1
	 * longitude		|	4				|	5
	 * MSL altitude		|	4				|	9
	 * ground speed		|	4				|	13
	 * heading			|	4				|	17
	 * satellites		|	1				|	21
	 * fix type			|	1				|	22
	 * date				|	4				|	23
	 * UTC time			|	4				|	27
	 * HDOP				|	2				|	31
	 * checksum			|	2				|	33
	 */
	gps.timestamp = time_GetMillis();
	if (!gps.available) {
		gps.available = SET;
		log_LogFileEntry("GPS receiver available");
		led_Cmd(1, LED_OFF);
	}
	gps.latitude = gps.buffer[1] + (gps.buffer[2] << 8) + (gps.buffer[3] << 16)
			+ (gps.buffer[4] << 24);
	gps.longitude = gps.buffer[5] + (gps.buffer[6] << 8) + (gps.buffer[7] << 16)
			+ (gps.buffer[8] << 24);
	gps.altitude = gps.buffer[9] + (gps.buffer[10] << 8)
			+ (gps.buffer[11] << 16) + (gps.buffer[12] << 24);
	gps.speed = gps.buffer[13] + (gps.buffer[14] << 8);
	gps.heading = gps.buffer[17] + (gps.buffer[18] << 8);
	gps.satellites = gps.buffer[21];
	if (gps.buffer[22] != 0x03) {
		if (gps.FixValid) {
			log_LogFileEntry("WARNING: lost GPS fix");
			hott.speak = SPEAK_ERR_GPS;
			gps.FixValid = RESET;
			led_Cmd(2, LED_ON);
		}
	} else {
		if (!gps.FixValid) {
			// first fix -> update cosLongitude
			float radLatitude = (float) (gps.latitude / 10000000L) * DEG_TO_RAD;
			gps.cosLatitude = cos(radLatitude);
			hott.speak = SPEAK_GPS_FIX;
			log_LogFileEntry("GPS fix");
			gps.FixValid = SET;
			led_Cmd(2, LED_OFF);
		}
	}
	uint32_t date = gps.buffer[23] + (gps.buffer[24] << 8)
			+ (gps.buffer[25] << 16) + (gps.buffer[26] << 24);
	gps.date.year = date % 100 + 2000;
	gps.date.month = (date / 100) % 100;
	gps.date.day = date / 10000;
	uint32_t time = gps.buffer[27] + (gps.buffer[28] << 8)
			+ (gps.buffer[29] << 16) + (gps.buffer[30] << 24);
	gps.date.second = (time / 100) % 100;
	gps.date.minute = (time / 10000) % 100;
	gps.date.hour = time / 1000000;
	gps.HDOP = gps.buffer[31] + (gps.buffer[33] << 8);

	// calculate components of the velocity vector
	float radHeading = ((float) gps.heading) * 0.01 * DEG_TO_RAD;
	gps.sinHeading = sin(radHeading);
	gps.cosHeading = cos(radHeading);
	gps.velocity.X = gps.cosHeading * (float) gps.speed * 0.01;
	gps.velocity.Y = gps.sinHeading * (float) gps.speed * 0.01;
	// TODO validate checksum necessary?

	if (gps.FixValid) {
		if (gps.homePosition.valid) {
			/*
			 * calculate position with reference to the homeposition
			 */
			int32_t diffNS, diffEW, diffHeight;
			diffNS = gps.latitude - gps.homePosition.Latitude;
			diffEW = gps.longitude - gps.homePosition.Longitude;
			diffHeight = gps.altitude - gps.homePosition.height;
			/*
			 * 1° in N/S-direction equals 111.32 km. At a resolution
			 * of 10000000 LSB/°, one LSB equals 0.011132m
			 */
			gps.position.X = diffNS * 0.011132f;
			/*
			 * in E/W direction, this value must be multiplied by
			 * the cosine of the latitude
			 */
			gps.position.Y = diffEW * 0.011132 * gps.cosLatitude;
			/*
			 * convert height difference from centimeters to meters
			 */
			gps.position.Z = diffHeight * 0.01f;
		} else {
			/*
			 * no current homeposition
			 * -> set homeposition = current position
			 */
			gps_SetHomePosition();
			imuPosition_Reset();
		}
	}
}
/*
 * sets the GPS homeposition to the current position
 */
void gps_SetHomePosition(void) {
	if (gps.FixValid) {
		gps.homePosition.Latitude = gps.latitude;
		gps.homePosition.Longitude = gps.longitude;
		gps.homePosition.height = gps.altitude;
		gps.homePosition.valid = SET;
		gps.position.X = gps.position.Y = gps.position.Z = 0.0f;
		log_LogFileEntry("GPS homeposition set");
	} else {
		log_LogFileEntry("WARNING: unable to set GPS homeposition");
	}
}
/*
 * sends a configuration command to the GPS. Implemented using Polling
 * -> execution is blocked until the transmission is completed ~4ms
 */
void gps_SendCommand(uint8_t command) {
	char buffer[24];
	uint8_t cmdLength;
	switch (command) {
	case GPS_COMMAND_BINARY:
		buffer[0] = '$';
		buffer[1] = 'P';
		buffer[2] = 'G';
		buffer[3] = 'C';
		buffer[4] = 'M';
		buffer[5] = 'D';
		buffer[6] = ',';
		buffer[7] = '1';
		buffer[8] = '6';
		buffer[9] = ',';
		buffer[10] = '0';
		buffer[11] = ',';
		buffer[12] = '0';
		buffer[13] = ',';
		buffer[14] = '0';
		buffer[15] = ',';
		buffer[16] = '0';
		buffer[17] = ',';
		buffer[18] = '0';
		buffer[19] = '*';
		buffer[20] = '6';
		buffer[21] = 'A';
		buffer[22] = '\r';
		buffer[23] = '\n';
		cmdLength = 24;
		break;
	case GPS_COMMAND_10HZ:
		buffer[0] = '$';
		buffer[1] = 'P';
		buffer[2] = 'M';
		buffer[3] = 'T';
		buffer[4] = 'K';
		buffer[5] = '2';
		buffer[6] = '2';
		buffer[7] = '0';
		buffer[8] = ',';
		buffer[9] = '1';
		buffer[10] = '0';
		buffer[11] = '0';
		buffer[12] = '*';
		buffer[13] = '2';
		buffer[14] = 'F';
		buffer[15] = '\r';
		buffer[16] = '\n';
		cmdLength = 17;
		break;
	case GPS_COMMAND_38400:
		buffer[0] = '$';
		buffer[1] = 'P';
		buffer[2] = 'M';
		buffer[3] = 'T';
		buffer[4] = 'K';
		buffer[5] = '2';
		buffer[6] = '5';
		buffer[7] = '1';
		buffer[8] = ',';
		buffer[9] = '3';
		buffer[10] = '8';
		buffer[11] = '4';
		buffer[12] = '0';
		buffer[13] = '0';
		buffer[14] = '*';
		buffer[15] = '2';
		buffer[16] = '7';
		buffer[17] = '\r';
		buffer[18] = '\n';
		cmdLength = 19;
		break;
	default:
		cmdLength = 0;
	}
	uint8_t i;
	for (i = 0; i < cmdLength; i++) {
		usart_putcGPS(buffer[i]);
	}
}
