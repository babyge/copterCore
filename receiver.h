/*
 * implementation of the Graupner SUMD protocol.
 * Receiver sends UART frame each 10ms. UART settings are 115200, 8N1
 *
 * Each frame consists of three bytes header, including the number of
 * channels and a failsafe indicator, followed by the channel values
 * and is completed by a CCITT-16 CRC (in SUMD, in SUMH a simple
 * checksum is used).
 *
 * byte	|	content
 * -----------------------------------
 * 0	|	start indicator (always 0xA8)
 * 1	|	0x01: SUMD, 0x00: SUMH, 0x81: failsafe
 * 2	|	number of channels n
 * 3 	|   channel 1 high byte
 * 4	| 	channel 1 low byte
 * .	.	.
 * .	.	.
 * 2n+1 | 	channel n high byte
 * 2n+2 | 	channel n low byte
 * 2n+3	|	CRC/checksum high byte
 * 2n+4 |	CRC/checksum low byte
 *
 * In this code the CRC/checksum is not evaluated
 */

#ifndef RECEIVER_H_
#define RECEIVER_H_

#include "time.h"
#include "stdcomm.h"
#include "hal.h"

// maximum number of receiver channels this code should be able to handle
#define RECEIVERMAXCHANNELS		16

#define CHANNEL_POWER			0
#define CHANNEL_PITCH			2
#define CHANNEL_ROLL			1
#define CHANNEL_YAW				3
#define CHANNEL_CONF1			6
#define CHANNEL_CONF2			7

// receiver switch functions (MC16 switch label - 1)
#define SWITCH_HEIGHT_OFF		5
#define SWITCH_HEIGHT_HOLD		4
#define SWITCH_MOTOR_OFF		11
#define SWITCH_FLYING			10
#define SWITCH_CAREFREE			12
#define SWITCH_START_LAND		13

struct {
	/*
	 * values of the RC channels. The values are stored in full resolution
	 * (-4800 to 4800). To convert to % divide by 32. Currently the lowest
	 * 3 bits are always 0 (with MC-16/GR-16) -> smallest detectable channel
	 * variation is 1/4%
	 */
	int16_t channel[RECEIVERMAXCHANNELS];
	/*
	 * current switch positions of the 15 MC-16 switches
	 * (these positions are encoded in the 5th and 6th channel)
	 */
	uint8_t switches[15];
	// this Flag indicates valid data in the channel array
	FlagStatus valid, failsafe;
	// Flag indicates new data, must be reset by external software
	volatile FlagStatus newData;
	// number of received channels
	uint8_t NumChannels;
	// system time in milliseconds at the last data update
	uint32_t timestamp;
	/*
	 * internal used variable. Counts the bytes in the currently receiving frame.
	 */
	uint8_t byteCount;
	uint8_t transmissionStarted;
	// buffer for partial received channel values
	uint16_t channelBuffer;
} receiver;

/*
 * decodes channel 5 and 6 and calculates the 15 switch positions
 */
void receiver_DecodeSwitches(void);
/*
 * writes the receiver data to the standard output (USART2)
 */
void receiver_Print(void);
/*
 * handles incoming data
 */
void receiver_IncomingData(uint8_t data);
/*
 * returns the receiver channel value within the range of -100 to 100
 */
int8_t receiver_GetChannel(uint8_t channel);
/*
 * routes specified receiver channels to servo outputs
 * (defined via config 'servoSource')
 */
void receiver_UpdateServos(void);



#endif /* RECEIVER_H_ */
