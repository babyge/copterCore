/*
 * stdcomm.c
 *
 *  Created on: Oct 29, 2013
 *      Author: jan
 */

#include "stdcomm.h"

/*
 * interprets incoming messages and sends the appropriate response
 */
void stdComm_MessageHandler(uint8_t id) {
	switch (id) {
	case MESSAGE_ENQUIRY:
		stdComm_SendMessage(MESSAGE_ACK, 0, 0);
		break;
	case MESSAGE_REQUEST_CONFIG:
		stdComm_SendMessage(MESSAGE_SEND_CONFIG, (uint8_t*) &config,
				sizeof(config));
		break;
	case MESSAGE_SEND_CONFIG:
		if (com.byteCount == sizeof(config)) {
			stdComm_SendMessage(MESSAGE_ACK, 0, 0);
			stdcomm_GetMessage((uint8_t*) &config);
			log_LogFileEntry("received new configuration");
		} else {
			stdComm_SendMessage(MESSAGE_NACK, 0, 0);
		}
		break;
	case MESSAGE_SAVE_CONFIG:
		stdComm_SendMessage(MESSAGE_ACK, 0, 0);
		eeprom_SaveConfig();
		break;
	case MESSAGE_TRIGGER_STEPRES: {
		;
		union {
			float f;
			uint32_t i;
		} stepsize, power;
		// extract step response settings from buffer
		stepsize.i = (com.RecBuffer[1] << 24) + (com.RecBuffer[2] << 16)
				+ (com.RecBuffer[3] << 8) + com.RecBuffer[4];
		power.i = (com.RecBuffer[5] << 24) + (com.RecBuffer[6] << 16)
				+ (com.RecBuffer[7] << 8) + com.RecBuffer[8];
		uint16_t advance = (com.RecBuffer[9] << 8) + com.RecBuffer[10];
		uint16_t response = (com.RecBuffer[11] << 8) + com.RecBuffer[12];
		uint16_t interval = (com.RecBuffer[13] << 8) + com.RecBuffer[14];
		// attempt to start the step response
		if (StepResponse_Start((StepResponseType) com.RecBuffer[0], stepsize.f,
				power.f, advance, response, interval) == SUCCESS) {
			// step response started -> send ACK message
			stdComm_SendMessage(MESSAGE_ACK, 0, 0);
		} else {
			// error while starting step response -> send NACK message
			stdComm_SendMessage(MESSAGE_NACK, 0, 0);
		}
	}
		break;
	case MESSAGE_REQUEST_BARO_CALIB:
		stdComm_SendMessage(MESSAGE_SEND_BARO_CALIB,
				(uint8_t*) &pressure.tempOffset, sizeof(pressure.tempOffset));
		break;
	case MESSAGE_SEND_BARO_CALIB:
		if (com.byteCount == sizeof(pressure.tempOffset)) {
			stdComm_SendMessage(MESSAGE_ACK, 0, 0);
			stdcomm_GetMessage((uint8_t*) &pressure.tempOffset);
			log_LogFileEntry("received barometer calibration data");
			// store in EEPROM
			pressure_SaveTempProfile();
		} else {
			stdComm_SendMessage(MESSAGE_NACK, 0, 0);
		}
		break;
	case MESSAGE_SENSOR_DEVIATION:
		if (com.byteCount == 2) {
			stdComm_SendMessage(MESSAGE_ACK, 0, 0);
			uint8_t dat[2];
			stdcomm_GetMessage(dat);
			deviation_StartMeasurement(dat[0], dat[1]);
		} else {
			stdComm_SendMessage(MESSAGE_NACK, 0, 0);
		}
		break;
	case MESSAGE_MANUAL_MOTORSPEED:
		if (com.byteCount == config.motor.num) {
			stdComm_SendMessage(MESSAGE_ACK, 0, 0);
			uint8_t pwmlist[MOTOR_MAXNUM];
			stdcomm_GetMessage(pwmlist);
			motor_setManual(pwmlist);
		} else {
			stdComm_SendMessage(MESSAGE_NACK, 0, 0);
		}
		break;
	case MESSAGE_AUTO_MOTORSPEED:
		stdComm_SendMessage(MESSAGE_ACK, 0, 0);
		motor_setAuto();
		break;
	}
	com.messageComplete = 0;
}

void stdComm_Send(uint8_t data[], uint8_t length) {
	uint8_t i;
	// transmit data
	for (i = 0; i < length; i++) {
		usart_putcStdComm(data[i]);
	}
}
void stdComm_puts(const char *s) {
	while (*s) {
		usart_putcStdComm(*s);
		s++;
	}
}

void stdComm_PrintValue(int32_t value) {
	char buffer[12];
	itoASCII(value, buffer);
	stdComm_puts(buffer);
}
/*
 * sends message consisting of start-identifier, message and end-identifier
 */
void stdComm_SendMessage(uint8_t id, uint8_t *message, uint16_t length) {
	// send start
	usart_putcStdComm(0x10);
	usart_putcStdComm(0x01);
	// send ID
	usart_putcStdComm(id);
	// send message
	for (; length > 0; length--) {
		if (*message == 0x10)
			usart_putcStdComm(0x10);
		usart_putcStdComm(*message++);
	}
	// send end
	usart_putcStdComm(0x10);
	usart_putcStdComm(0x04);
}

int8_t stdcomm_MessageReady(void) {
	if (com.messageComplete) {
		return com.messageID;
	} else
		return 0;
}
void stdcomm_GetMessage(uint8_t data[]) {
	uint16_t i;
	// transmit data from buffer into data-array
	for (i = 0; i < com.byteCount; i++)
		data[i] = com.RecBuffer[i];
}
/*
 * this function must be called whenever a new data byte is available
 */
void stdComm_IncomingData(uint8_t data) {
//	// store incoming data
//	com.RecBuffer[com.RecPos] = data;
//	com.MessageLength = 0;
//	// search for end of input (0xAA followed by 0x0A)
//	if (com.RecPos > 0 && com.RecBuffer[com.RecPos] == 0x0A
//			&& com.RecBuffer[com.RecPos - 1] == 0xAA) {
//		com.MessageLength = com.RecPos - 1;
//		com.RecPos = 0;
//	} else
//		com.RecPos++;
	if (com.lastChar == 0x10) {
		// data link escape
		if (data == 0x10) {
			com.RecBuffer[com.byteCount++] = 0x10;
		} else if (data == 0x01) {
			// start of heading
			com.messageStarted = 1;
			com.byteCount = -1;
		} else if (data == 0x04) {
			// end of transmission
			com.messageStarted = 0;
			com.messageComplete = 1;
		}
	} else if (com.messageStarted && data != 0x10) {
		if (com.byteCount >= 0)
			com.RecBuffer[com.byteCount++] = data;
		else {
			com.messageID = data;
			com.byteCount = 0;
		}
	}
	if (com.lastChar == 0x10)
		com.lastChar = 0;
	else
		com.lastChar = data;
}

