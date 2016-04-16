/*
 * receiver.c
 *
 *  Created on: Nov 7, 2013
 *      Author: jan
 */

#include "receiver.h"

struct Receiver receiver;

/*
 * decodes channel 5 and 6 and calculates the 15 switch positions
 *
 * Channel 5 - Offset: -125
 * switch		effect
 * 1			+1/4
 * 2			+1
 * 3			+2
 * 4			+1/2
 * 5			+4
 * 6			+8
 * 7			+16
 * 8			+32
 * 9			+64
 * 10			+128
 *
 * Channel 6 - Offset: 0
 * switch		effect
 * 11			+1
 * 12			+2
 * 13			+4
 * 14			+8
 * 15			+/-100
 */
void receiver_DecodeSwitches(void) {
	int16_t ch5 = receiver.channel[4];
	// remove offset; add 125%
	ch5 += 4000;
	// extract the single switch positions (1% equals 32LSB).
	// At this point each switch is represented by one bit in ch5
	if (ch5 & 0x0008)
		receiver.switches[0] = 1;
	else
		receiver.switches[0] = 0;
	if (ch5 & 0x0020)
		receiver.switches[1] = 1;
	else
		receiver.switches[1] = 0;
	if (ch5 & 0x0040)
		receiver.switches[2] = 1;
	else
		receiver.switches[2] = 0;
	if (ch5 & 0x0010)
		receiver.switches[3] = 1;
	else
		receiver.switches[3] = 0;
	if (ch5 & 0x0080)
		receiver.switches[4] = 1;
	else
		receiver.switches[4] = 0;
	if (ch5 & 0x0100)
		receiver.switches[5] = 1;
	else
		receiver.switches[5] = 0;
	if (ch5 & 0x0200)
		receiver.switches[6] = 1;
	else
		receiver.switches[6] = 0;
	if (ch5 & 0x0400)
		receiver.switches[7] = 1;
	else
		receiver.switches[7] = 0;
	if (ch5 & 0x0800)
		receiver.switches[8] = 1;
	else
		receiver.switches[8] = 0;
	if (ch5 & 0x1000)
		receiver.switches[9] = 1;
	else
		receiver.switches[9] = 0;
	// channel 6
	int16_t ch6 = receiver.channel[5];
	// special case: switch 15
	if (ch6 > 0) {
		receiver.switches[14] = 1;
		// remove switch 15 offset
		ch6 -= 3200;
	} else {
		receiver.switches[14] = 0;
		// remove switch 15 offset
		ch6 += 3200;
	}
	// at this point the remaining switches are represented by one bit in ch6
	if (ch6 & 0x0020)
		receiver.switches[10] = 1;
	else
		receiver.switches[10] = 0;
	if (ch6 & 0x0040)
		receiver.switches[11] = 1;
	else
		receiver.switches[11] = 0;
	if (ch6 & 0x0080)
		receiver.switches[12] = 1;
	else
		receiver.switches[12] = 0;
	if (ch6 & 0x0100)
		receiver.switches[13] = 1;
	else
		receiver.switches[13] = 0;
}
/*
 * writes the receiver data to the standard output (USART2)
 */
void receiver_Print(void) {
	stdComm_puts("receiver:");
	if (receiver.valid)
		stdComm_puts("valid\n");
	else
		stdComm_puts("failsafe\n");
	stdComm_puts("channel1: ");
	stdComm_PrintValue(receiver.channel[0]);
	stdComm_puts("\nchannel2: ");
	stdComm_PrintValue(receiver.channel[1]);
	stdComm_puts("\nchannel3: ");
	stdComm_PrintValue(receiver.channel[2]);
	stdComm_puts("\nchannel4: ");
	stdComm_PrintValue(receiver.channel[3]);
	stdComm_puts("\nchannel5: ");
	stdComm_PrintValue(receiver.channel[4]);
	stdComm_puts("\nchannel6: ");
	stdComm_PrintValue(receiver.channel[5]);
	stdComm_puts("\nchannel7: ");
	stdComm_PrintValue(receiver.channel[6]);
	stdComm_puts("\nchannel8: ");
	stdComm_PrintValue(receiver.channel[7]);
	stdComm_puts("\nswitches: ");
	uint8_t i;
	for (i = 0; i < 15; i++) {
		stdComm_PrintValue(receiver.switches[i]);
	}
	usart_putcStdComm(0x0A);
}
/*
 * handles incoming data
 */
void receiver_IncomingData(uint8_t data) {
	// check for start indicator
	if (!receiver.transmissionStarted && data == 0xA8) {
		receiver.transmissionStarted = 1;
		receiver.byteCount = 0;
	}
	if (receiver.transmissionStarted) {
		if (receiver.byteCount == 1) {
			// check for failsafe
			if (data > 0x01) {
				if (!receiver.failsafe)
					log_LogFileEntry("ERROR: receiver failsafe");
				receiver.failsafe = SET;
				receiver.valid = RESET;
				led_Cmd(0, LED_ON);
			} else {
				receiver.failsafe = RESET;
			}
		} else if (receiver.byteCount == 2) {
			// number of channels
			receiver.NumChannels = data;
		} else if (receiver.byteCount > 2
				&& receiver.byteCount < (receiver.NumChannels << 1) + 3) {
			// received byte is part of a channel value
			if (receiver.byteCount % 2 == 1) {
				// high byte, transmitted first
				receiver.channelBuffer = (data << 8);
			} else {
				// low byte, transmitted last
				receiver.channelBuffer += data;
				// calculate channel number (starting at 0)
				uint8_t channelNum = (receiver.byteCount >> 1) - 2;
				// remove offset and store value
				receiver.channel[channelNum] = (int16_t) receiver.channelBuffer
						- 12000;
			}
		} else if (receiver.byteCount == (receiver.NumChannels << 1) + 4) {
			// reached end of frame
			receiver.transmissionStarted = 0;
			receiver.timestamp = time_GetMillis();
			if (!receiver.failsafe) {
				if (!receiver.valid)
					log_LogFileEntry("receiver signal valid");
				receiver.valid = SET;
				led_Cmd(0, LED_OFF);
			}
			receiver.newData = SET;
		}
		// increase bytecount while receiving bytes of a frame
		receiver.byteCount++;
	}
}
/*
 * returns the receiver channel value within the range of -100 to 100
 */
int8_t receiver_GetChannel(uint8_t channel) {
	return receiver.channel[channel] / 32;
}
/*
 * routes specified receiver channels to servo outputs
 * (defined via config 'servoSource')
 */
void receiver_UpdateServos(void) {
	uint8_t i;
	for (i = 0; i < 12; i++) {
		if (config.servoSource[i] <= receiver.NumChannels) {
			int16_t channel = receiver.channel[config.servoSource[i]];
			// convert channel to servo position (e.g. 4800->1500)
			channel *= 5;
			channel /= 16;
			servo_SetPosition(i, channel);
		}
	}
}
