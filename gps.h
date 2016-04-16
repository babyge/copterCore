/*
 * gps data parser for custom 3DR protocol
 * (see: Customize Function Specicfication 3D Robotics V1.6
 */

#ifndef GPS_H_
#define GPS_H_

#include "hal.h"
#include "hott.h"

#define GPS_COMMAND_10HZ		0
#define GPS_COMMAND_BINARY		1
#define GPS_COMMAND_38400		2

struct GPS{
	// coordinates, 10000000LSB/°
	int32_t latitude, longitude;
	// measured height above sea level in cm
	int32_t altitude;
	// direction of travel in 0.01°, velocity in cm/s
	uint16_t heading, speed;
	// additional buffered information
	float sinHeading, cosHeading, cosLatitude;
	struct {
		// velocity in m/s
		float X, Y;
	} velocity;
	struct {
		uint8_t hour, minute, second;
		uint8_t day, month;
		uint16_t year;
	} date;
	struct {
		// position in meter with reference to homeposition
		float X, Y, Z;
	} position;
	struct {
		// values are set at start
		int32_t Latitude, Longitude;
		int32_t height;
		FlagStatus valid;
	} homePosition;
	// additional fix parameters
	// number of satellites in view
	uint8_t satellites;
	FlagStatus FixValid;
	// dilution of precision, multiplied by 100 (e.g. 150 for a dilution of 1.50)
	uint16_t HDOP;
	// indicates the presence of a GPS receiver
	FlagStatus available;
	// internal data buffer
	uint8_t buffer[35];
	uint8_t lastChar;
	uint8_t bytecount;
	FlagStatus sentenceComplete, sentenceStarted;
	uint32_t timestamp;
};

extern struct GPS gps;

//typedef enum {GPS_RMC, GPS_GSA, GPS_GGA, GPS_GSV, GPS_UNKNOWN} GPSSentenceType;

/*
 * initializes the GPS modul (should be called some time after power-up, e.g. after
 * 2 seconds)
 */
void gps_Init(void);

/*
 * must be called whenever a new sign is read from the GPS modul
 */
void gps_IncomingData(uint8_t data);

/*
 * checks whether a complete GPS sentence has been received
 */
FlagStatus gps_FrameComplete(void);

/*
 * processes received GPS sentences
 */
void gps_EvaluateFrame(void);

/*
 * sets the GPS homeposition to the current position
 */
void gps_SetHomePosition(void);

/*
 * sends a configuration command to the GPS. Implemented using Polling
 * -> execution is blocked until the transmission is completed ~4ms
 */
void gps_SendCommand(uint8_t command);

#endif /* GPS_H_ */
