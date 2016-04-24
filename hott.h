/*
 * implementation of the Graupner HoTT protocol
 */

#ifndef HOTT_H_
#define HOTT_H_

#include "hal.h"
#include "gps.h"
#include "battery.h"
#include "imuPosition.h"
#include "flightstate.h"

// HoTT modes
#define HOTT_IDLE 			0
#define HOTT_BINARY			1
#define HOTT_TEXT 			2
// HoTT module masks
#define HOTT_UPDATE_GPS		1
#define HOTT_UPDATE_GEM		2
#define HOTT_UPDATE_EAM		4
#define HOTT_UPDATE_VARIO	8
#define HOTT_UPDATE_TEXT	16
// HoTT module IDs
#define HOTTV4_GPS_PACKET_ID 	0x8A
#define HOTTV4_GEM_PACKET_ID 	0x8D
#define HOTTV4_EAM_PACKET_ID 	0x8E
#define HOTTV4_VARIO_PACKET_ID 0x89
// HoTT sensor IDs
#define HOTTV4_GPS_SENSOR_ID	0xA0
#define HOTTV4_GEM_SENSOR_ID	0xD0
#define HOTTV4_EAM_SENSOR_ID	0xE0
#define HOTTV4_VARIO_SENSOR_ID	0x90
// HoTT speak data
#define SPEAK_ERR_CALIBRATION  1
#define SPEAK_ERR_RECEIVER 	 2
#define SPEAK_ERR_DATABUS  	 3
#define SPEAK_ERR_NAVI	  	 4
#define SPEAK_ERROR		 	 5
#define SPEAK_ERR_COMPASS 	 6
#define SPEAK_ERR_SENSOR 	 7
#define SPEAK_ERR_GPS	 	 8
#define SPEAK_ERR_MOTOR	 	 9
#define SPEAK_MAX_TEMPERAT  10
#define SPEAK_ALT_REACHED  11
#define SPEAK_WP_REACHED    12
#define SPEAK_NEXT_WP       13
#define SPEAK_LANDING       14
#define SPEAK_GPS_FIX       15
#define SPEAK_UNDERVOLTAGE  16
#define SPEAK_GPS_HOLD      17
#define SPEAK_GPS_HOME      18
#define SPEAK_GPS_OFF       19
#define SPEAK_BEEP          20
#define SPEAK_MIKROKOPTER   21
#define SPEAK_CAPACITY      22
#define SPEAK_CF_OFF        23
#define SPEAK_CALIBRATE     24
#define SPEAK_MAX_RANGE     25
#define SPEAK_MAX_ALTITUDE   26

#define SPEAK_MK_OFF	  	38
#define SPEAK_ALTITUDE_ON  	39
#define SPEAK_ALTITUDE_OFF 	40
#define SPEAK_CF_ON      	46
#define SPEAK_SINKING      	47
#define SPEAK_RISING      	48
#define SPEAK_HOLDING      	49
#define SPEAK_GPS_ON      	50
#define SPEAK_FOLLOWING    	51
#define SPEAK_STARTING      52
// HoTT key codes
#define HOTT_LEFT			0x07
#define HOTT_RIGHT			0x0E
#define HOTT_UP				0x0D
#define HOTT_DOWN			0x0B
#define HOTT_SET			0x09

#define HOTT_MENU_OVERVIEW	0
#define HOTT_MENU_ACC		1
#define HOTT_MENU_GYRO		2
#define HOTT_MENU_MAG		3
#define HOTT_MENU_BARO		4
#define HOTT_MENU_MOTORS	5
// must always be last!
#define HOTT_N_MENUS		6

struct Hott {
	/**********************************
	 * data containing output values
	 * (is accessed by usart.h for
	 * data transmission)
	 *********************************/
	// pointer to the HoTT data to be send next (is modified during sending)
	uint8_t *outputPointer;
	// delay in ms till sending begins (typically 5ms)
	uint8_t delay;
	// number of bytes to be transmitted (without checksum)
	uint8_t bytes;
	// flag indicates new data is to be send
	FlagStatus dataToBeSend;
	/**********************************
	 * HoTT state data
	 *********************************/
	// can be HOTT_IDLE, HOTT_BINARY or HOTT_TEXT
	uint8_t mode;
	// can be a combination of HOTT_GPS, HOTT_GEM, HOTT_EAM, HOTT_VARIO, HOTT_TEXT
	uint8_t updateRequest;
	uint8_t keyPressed;
	// alarm buffer
	uint8_t speak;
	/**********************************
	 * HoTT data structs
	 *********************************/
	struct {
		uint8_t StartByte;  //1 	// 0x7C
		uint8_t Packet_ID;  //2  	// 0x8A  - GPS ID
		uint8_t WarnBeep;   //3 	// Anzahl der Töne 0..36
		uint8_t SensorID;       // 4 0xA0
		uint8_t InverseStatus1; // 5
		uint8_t InverseStatus2; // 6
		uint8_t Heading;	//7  	// 1 = 2°
		uint8_t SpeedL;		//8   // in km/h
		uint8_t SpeedH;		//9
		uint8_t Lat_North;	//10
		uint8_t Lat_MinL;	//11
		uint8_t Lat_MinH;	//12
		uint8_t Lat_SekL;	//13+14
		uint8_t Lat_SekH;
		uint8_t Lon_East;	//15
		uint8_t Lon_MinL;	    //16+17
		uint8_t Lon_MinH;
		uint8_t Lon_SekL;	//18+19
		uint8_t Lon_SekH;
		uint8_t DistanceL;	//20+21	   // 9000 = 0m
		uint8_t DistanceH;
		uint8_t AltitudeL;	//22+23    // 500 = 0m
		uint8_t AltitudeH;
		uint8_t m_secL;	 	//24+25    // 3000 = 0
		uint8_t m_secH;
		uint8_t m_3sec;	 	//26 120 = 0
		uint8_t NumOfSats;	//27
		uint8_t SatFix;     //28
		uint8_t HomeDirection; // 29
		uint8_t AngleX;		  // 30
		uint8_t AngleY;       // 31
		uint8_t AngleZ;       // 32
		uint8_t GyroXL;         //33+34
		uint8_t GyroXH;
		uint8_t GyroYL;         //35+36
		uint8_t GyroYH;
		uint8_t GyroZL;         //37+38
		uint8_t GyroZH;
		uint8_t Vibration;    // 39
		uint8_t FreeCharacters[3]; // 40-42
		uint8_t Version;   	// 43
		uint8_t EndByte;   		// 0x7D
	} gps;
	struct {
		uint8_t StartByte;   	// 0x7C
		uint8_t Packet_ID;    	// HOTT_GENERAL_PACKET_ID
		uint8_t WarnBeep;    	// 3 Anzahl der Töne 0..36
		uint8_t SensorID;       // 4 0xD0
		uint8_t InverseStatus1; // 5
		uint8_t InverseStatus2; // 6
		uint8_t VoltageCell1;	// 7 208 = 4,16V  (Voltage * 50 = Wert)
		uint8_t VoltageCell2;	// 8 209 = 4,18V
		uint8_t VoltageCell3;	// 9
		uint8_t VoltageCell4;	// 10
		uint8_t VoltageCell5;	// 11
		uint8_t VoltageCell6;	// 12
		uint8_t Battery1L;		// 13+14 51  = 5,1V
		uint8_t Battery1H;
		uint8_t Battery2L;		// 15+16 51  = 5,1V
		uint8_t Battery2H;
		uint8_t Temperature1;	// 17 44 = 24°C, 0 = -20°C
		uint8_t Temperature2;	// 18 44 = 24°C, 0 = -20°C
		uint8_t FuelPercent;    // 19
		uint8_t FuelCapacityL;   // 20+21
		uint8_t FuelCapacityH;
		uint8_t RpmL;  			// 22+23
		uint8_t RpmH;
		uint8_t AltitudeL;		// 24+25
		uint8_t AltitudeH;
		uint8_t m_secL;	 	    // 26+27 3000 = 0
		uint8_t m_secH;
		uint8_t m_3sec;	 	    // 28 120 = 0
		uint8_t CurrentL;		// 29+30 1 = 0.1A
		uint8_t CurrentH;
		uint8_t InputVoltageL;	// 31+32 66  = 6,6V
		uint8_t InputVoltageH;
		uint8_t CapacityL;		// 33+34 1  = 10mAh
		uint8_t CapacityH;
		uint8_t SpeedL;			// 35+36
		uint8_t SpeedH;
		uint8_t LowestCellVoltage; 	// 37
		uint8_t LowestCellNumber;  	// 38
		uint8_t Rpm2L;  				// 39+40
		uint8_t Rpm2H;
		uint8_t ErrorNumber;		// 41
		uint8_t Pressure;           // 42  in 0,1bar 20=2,0bar
		uint8_t Version;            // 43
		uint8_t EndByte;   		// 0x7D
	} gem;
	struct {
		uint8_t StartByte;   	// 0x7C
		uint8_t Packet_ID;    	// HOTT_ELECTRIC_AIR_PACKET_ID
		uint8_t WarnBeep;    	// Anzahl der Töne 0..36
		uint8_t SensorID;       // 4 0xE0
		uint8_t InverseStatus1; // 5
		uint8_t InverseStatus2; // 6
		uint8_t VoltageCell1;	// 7 208 = 4,16V  (Voltage * 50 = Wert)
		uint8_t VoltageCell2;	// 209 = 4,18V
		uint8_t VoltageCell3;	//
		uint8_t VoltageCell4;	//
		uint8_t VoltageCell5;	//
		uint8_t VoltageCell6;	//
		uint8_t VoltageCell7;	//
		uint8_t VoltageCell8;	//
		uint8_t VoltageCell9;	//
		uint8_t VoltageCell10;	//
		uint8_t VoltageCell11;	//
		uint8_t VoltageCell12;	//
		uint8_t VoltageCell13;	//
		uint8_t VoltageCell14;	// 20
		uint8_t Battery1L;		// 21+22 51  = 5,1V
		uint8_t Battery1H;
		uint8_t Battery2L;		// 23+24 51  = 5,1V
		uint8_t Battery2H;
		uint8_t Temperature1;	// 25 44 = 24°C, 0 = -20°C
		uint8_t Temperature2;	// 26 44 = 24°C, 0 = -20°C
		uint8_t AltitudeL;		// 27+28
		uint8_t AltitudeH;
		uint8_t CurrentL;		// 29+30 1 = 0.1A
		uint8_t CurrentH;
		uint8_t InputVoltageL;	// 31+32 66  = 6,6V
		uint8_t InputVoltageH;
		uint8_t CapacityL;		// 33+34 1  = 10mAh
		uint8_t CapacityH;
		uint8_t m_secL;	 	    // 35+36 30000 = 0
		uint8_t m_secH;
		uint8_t m_3sec;	 	    // 37 120 = 0
		uint8_t RpmL;			// 38+39
		uint8_t RpmH;
		uint8_t FlightTimeMinutes; // 40
		uint8_t FlightTimeSeconds; // 41
		uint8_t Speed;			// 42  1=2km
		uint8_t Version;   		// 43 0x00
		uint8_t EndByte;   		// 0x7D
	} eam;
	struct {
		uint8_t StartByte;   	// 0x7C
		uint8_t Packet_ID;    	// 0x89  - Vario ID
		uint8_t WarnBeep;   //3 	// Anzahl der Töne 0..36
		uint8_t SensorID;        // 0x90
		uint8_t InverseStatus;
		uint8_t AltitudeL;	//6+7    // 500 = 0m
		uint8_t AltitudeH;
		uint8_t MaxAltitudeL;	//8+9    // 500 = 0m
		uint8_t MaxAltitudeH;
		uint8_t MinAltitudeL;	//10+11   // 500 = 0m
		uint8_t MinAltitudeH;
		uint8_t m_secL;	 	//12+13    // 3000 = 0
		uint8_t m_secH;
		uint8_t m_3secL;	 	//14+15
		uint8_t m_3secH;
		uint8_t m_10secL;	//26+17
		uint8_t m_10secH;
		uint8_t Text[21];   //18-38
		uint8_t FreeCharacters[3]; // 39-41
		uint8_t NullByte;   // 42 0x00
		uint8_t Version;   	// 43
		uint8_t EndByte;   	// 0x7D
	} vario;
	struct {
		uint8_t StartByte;
		uint8_t Packed_ID;
		uint8_t WarnBeep;
		char text[8 * 21];
		uint8_t EndByte;

		uint8_t selectedMenu;
	} text;
};

extern struct Hott hott;

/*
 * updates HoTT data when requested
 */
void hott_Update(void);
/*
 * processes received bytes
 */
void hott_IncomingData(uint8_t data);

/*
 * handles user inputs in the text menu
 */
void hott_KeyPressed(void);

/*
 * fills the text buffer with valid data
 */
void hott_SetTextBuffer(void);

/*
 * Clears the entire text buffer
 */
void hott_ClearText(void);

/*
 * writes a string into the text buffer
 */
void hott_Print(uint8_t x, uint8_t y, char *text, uint8_t inverted);

void hott_PrintInt(uint8_t x, uint8_t y, int32_t n, uint8_t inverted);

void hott_PrintFloat(uint8_t x, uint8_t y, float f, uint8_t inverted);

#endif /* HOTT_H_ */
