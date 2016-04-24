/*
 * stdcomm.h
 *
 *  Created on: Oct 29, 2013
 *      Author: jan
 *
 */

#ifndef STDCOMM_H_
#define STDCOMM_H_

#include "hal.h"
#include "String/stringconversion.h"
#include "pressure.h"
#include "sensorDeviation.h"

// message IDs
#define MESSAGE_REQUEST_CONFIG			0x01
#define MESSAGE_SEND_CONFIG				0x02
#define MESSAGE_SAVE_CONFIG				0x03
#define MESSAGE_TRIGGER_STEPRES			0x04
#define MESSAGE_ENQUIRY					0x05
#define MESSAGE_ACK						0x06
#define MESSAGE_SEND_STEPRESPONSE		0x07
#define MESSAGE_REQUEST_BARO_CALIB		0x08
#define MESSAGE_SEND_BARO_CALIB			0x09
#define MESSAGE_SENSOR_DEVIATION 		0x0B
#define MESSAGE_MANUAL_MOTORSPEED		0x0C
#define MESSAGE_AUTO_MOTORSPEED			0x0D
#define MESSAGE_REQUEST_SENSOR_STATE	0x0E
#define MESSAGE_SEND_SENSOR_STATE		0x0F

#define MESSAGE_NACK				0x15


#define STDCOMM_BAUD				115200
#define STDCOMM_SEND_BUFFER_SIZE	1024
#define STDCOMM_REC_BUFFER_SIZE		1024

#define stdcommIncSendReadPos() com.SendBufferReadPos=(com.SendBufferReadPos+1)%STDCOMM_SEND_BUFFER_SIZE
#define stdcommIncSendWritePos() com.SendBufferWritePos=(com.SendBufferWritePos+1)%STDCOMM_SEND_BUFFER_SIZE


struct Com{
	uint8_t SendBuffer[STDCOMM_SEND_BUFFER_SIZE];
	uint32_t SendBufferWritePos;
	volatile uint32_t SendBufferReadPos;

	uint8_t RecBuffer[STDCOMM_REC_BUFFER_SIZE];
	int16_t byteCount;
	uint8_t lastChar;
	uint8_t messageID;
	//uint8_t MessageLength;
	uint8_t messageStarted, messageComplete;
};

extern struct Com com;

/*
 * interprets incoming messages and sends the appropriate response
 */
void stdComm_MessageHandler(uint8_t id);


void stdComm_Send(uint8_t data[], uint8_t length);
void stdComm_puts(const char *s);

void stdComm_PrintValue(int32_t value);

/*
 * sends message consisting of start-identifier, message and end-identifier
 */
void stdComm_SendMessage(uint8_t id, uint8_t *message, uint16_t length);

int8_t stdcomm_MessageReady(void);
void stdcomm_GetMessage(uint8_t data[]);
/*
 * this function must be called whenever a new data byte is available
 */
void stdComm_IncomingData(uint8_t data);



#endif /* STDCOMM_H_ */
