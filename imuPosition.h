/*
 * The position is calculated using three independent kalman filters.
 * The filters for the horizontal position are identical for the
 * north-south and east-west axis:
 *
 * (values are positive for north and east location, negative for south
 * and west)
 *
 * state vector:
 * 		X/Y
 * 		velocity.X/Y
 * 		acceleration.X/Y
 *
 * measurement vector:
 * 		gps.X/Y
 *		gps.velocity.X/Y
 *		acceleration.X/Y (acceleration from accelerometer transformed to
 *							local navigation frame)
 * transition matrix F:
 * 		1	dt	0
 * 		0	1	dt
 * 		0	0	1,
 * 		where dt is the time since the last filter update
 *
 * measurement matrix H:
 *		1	0	0
 * 		0	1	0
 * 		0	0	1
 *
 * The vertical filter consists of four states and three different sensors:
 * state vector:
 * 		Z
 * 		velocity
 * 		acceleration
 * 		baro bias
 *
 * measurement vector:
 * 		gps.height
 * 		pressure.height
 * 		acceleration
 *
 * transition matrix F:
 * 		1	dt	0	0
 * 		0	1	dt	0
 * 		0	0	1	0
 * 		0	0	0	1
 *
 * 	measurement matrix H:
 * 		1	0	0	0
 * 		1	0	0	1
 * 		0	0	-1	0
 */

#ifndef IMUPOSITION_H_
#define IMUPOSITION_H_

#include <stdint.h>
#include "gps.h"

// IMU sensors
#define SENSOR_ACCELEROMETER					0
#define	 SENSOR_GPS								1
#define SENSOR_BAROMETER						2

// kalman filter parameters
// !! THESE VALUES HAVE BEEN MOVED TO THE CONFIG STRUCT IN EEPROM.H !!
//#define P_KALMAN_ACC_NOISE						2000.0f
//#define P_KALMAN_GPS_VERTICAL_NOISE				200.0f
//#define P_KALMAN_GPS_HORIZONTAL_NOISE			10.0f
//#define P_KALMAN_GPS_VELOCITY_NOISE				5.0f
//#define P_KALMAN_BAROMETER_NOISE				30.0f
//
//#define P_KALMAN_STATE_ACC_NOISE				50.0f
//#define P_KALMAN_STATE_GPS_VERTICAL_NOISE		3.0f
//#define P_KALMAN_STATE_GPS_HORIZONTAL_NOISE		10.0f
//#define P_KALMAN_STATE_GPS_VELOCITY_NOISE		5.0f
//#define P_KALMAN_STATE_VERTICAL_VELOCITY_NOISE	50.0f
//#define P_KALMAN_STATE_BARO_OFFSET_NOISE		0.01f

/*
 * position, velocity and acceleration in local navigation frame
 * (x-axis pointing north, y-axis pointing east and z-axis pointing down)
 */
struct verticalKalman {
	float x[4];
	float *q[4];
	float *z[3];
	float *r[3];
	float p[4][4];
	uint32_t timestamp;
};
struct horizontalKalman {
	float x[3];
	float *q[3];
	float *z[3];
	float *r[3];
	float p[3][3];
	uint32_t timestamp;
};

struct {
	struct {
		float X, Y, Z;
	} acceleration;
	struct {
		float X, Y, Z;
	} velocity;
	float X, Y, Z;
	struct horizontalKalman horizontalX, horizontalY;
	struct verticalKalman vertikalZ;
} position;

/*
 * initializes kalman filters
 */
void imuPosition_Init(void);

/*
 * sets all state vectors to the measured values.
 * Must be called after an offset correction like
 * gps_SetHomePosition or pressure_SetHeight
 */
void imuPosition_Reset(void);

/*
 * updates the position data based on the values from one IMU sensor
 */
void imuPosition_Update(uint8_t sensor);

/*
 * transfers the state variables from the kalman filters to
 * the position/velocity/acceleration struct
 */
void imuPosition_CopyFilterValues(void);

/*
 * internal kalman filter functions
 */
void imuPosition_HorizontalUpdate(struct horizontalKalman *kal, uint8_t sensor);
void imuPosition_VerticalUpdate(struct verticalKalman *kal, uint8_t sensor);

#endif /* IMUPOSITION_H_ */
