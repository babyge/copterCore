/*
 * imu.c
 *
 *  Created on: Nov 5, 2013
 *      Author: jan
 */

#include "imu.h"

/*
 * performs a kalman filter update and calculates new values
 * for roll, pitch and yaw. For proper function this method
 * has to be called with the frequency given by
 * KALMAN_UPDATE_INTERVAL
 */
void imu_Update(void) {
	attitude.timestamp = time_GetMillis();
	// update sensor values
	if (internali2c.accAvailable == SET)
		acc_Update();
	if (internali2c.gyroAvailable == SET)
		gyro_Update();
	if (internali2c.magAvailable == SET)
		mag_Update();
	// update kalman state
	kalman_Update();
	/*
	 * calculation of roll, pitch and yaw.
	 * see http://circuitcellar.com/featured/implement-a-tilt-and-interference-compensated-electronic-compass/
	 * (note:in this code the accelerometer values are inverted)
	 */
	attitude.roll = atan2f(-kalman.x[KALMAN_X_ACC_Y],
			-kalman.x[KALMAN_X_ACC_Z]);
	attitude.sinRoll = sinf(attitude.roll);
	attitude.cosRoll = cosf(attitude.roll);
	float Gz = -kalman.x[KALMAN_X_ACC_Y] * attitude.sinRoll
			- kalman.x[KALMAN_X_ACC_Z] * attitude.cosRoll;
	attitude.pitch = atan2f(kalman.x[KALMAN_X_ACC_X], Gz);
	attitude.sinPitch = sinf(attitude.pitch);
	attitude.cosPitch = cosf(attitude.pitch);
	float Bfy = kalman.x[KALMAN_X_MAG_Y] * attitude.cosRoll
			- kalman.x[KALMAN_X_MAG_Z] * attitude.sinRoll;
	float Bfx = kalman.x[KALMAN_X_MAG_X] * attitude.cosPitch
			+ kalman.x[KALMAN_X_MAG_Y] * attitude.sinRoll * attitude.sinPitch
			+ kalman.x[KALMAN_X_MAG_Z] * attitude.cosRoll * attitude.sinPitch;
	attitude.yaw = atan2f(-Bfy, Bfx);
	attitude.sinYaw = sinf(attitude.yaw);
	attitude.cosYaw = cosf(attitude.yaw);
}
/*
 * writes the imu data to the computer
 */
void imu_Print(void) {
	stdComm_puts("attitude pitch:");
	stdComm_PrintValue(attitude.pitch * RAD_TO_DEG);
	stdComm_puts(" roll:");
	stdComm_PrintValue(attitude.roll * RAD_TO_DEG);
	stdComm_puts(" yaw:");
	stdComm_PrintValue(attitude.yaw * RAD_TO_DEG);
	usart_putcStdComm(0x0A);
}

