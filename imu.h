/*
 * imu.h
 *
 *  Created on: Nov 5, 2013
 *      Author: jan
 */

#ifndef IMU_H_
#define IMU_H_

#include "kalman.h"

struct Attitude{
	float roll, pitch, yaw;
	float sinRoll, sinPitch, sinYaw;
	float cosRoll, cosPitch, cosYaw;
	uint32_t timestamp;
};

extern struct Attitude attitude;

/*
 * performs a kalman filter update and calculates new values
 * for roll, pitch and yaw. For proper function this method
 * has to be called with the frequency given by
 * KALMAN_UPDATE_INTERVAL
 */
void imu_Update(void);
/*
 * writes the imu data to the computer
 */
void imu_Print(void);

#endif /* IMU_H_ */
