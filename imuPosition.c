/*
 * imuPosition.c
 *
 *  Created on: Nov 6, 2013
 *      Author: jan
 */

#include "imuPosition.h"

struct Position position;

/*
 * initializes kalman filters
 */
void imuPosition_Init(void) {
	// measurement vector
	position.horizontalX.z[0] = &gps.position.X;
	position.horizontalX.z[1] = &gps.velocity.X;
	position.horizontalX.z[2] = &position.acceleration.X;

	position.horizontalY.z[0] = &gps.position.Y;
	position.horizontalY.z[1] = &gps.velocity.Y;
	position.horizontalY.z[2] = &position.acceleration.Y;

	position.vertikalZ.z[0] = &gps.position.Z;
	position.vertikalZ.z[1] = &pressure.height;
	position.vertikalZ.z[2] = &position.acceleration.Z;

	// state noise
	position.horizontalX.q[0] = &config.positionKalman.CovStateGPSHorizontal; //P_KALMAN_STATE_GPS_HORIZONTAL_NOISE;
	position.horizontalX.q[1] = &config.positionKalman.CovStateGPSVelocity; //P_KALMAN_STATE_GPS_VELOCITY_NOISE;
	position.horizontalX.q[2] = &config.positionKalman.CovStateAcc; //P_KALMAN_STATE_ACC_NOISE;

	position.horizontalY.q[0] = &config.positionKalman.CovStateGPSHorizontal; //P_KALMAN_STATE_GPS_HORIZONTAL_NOISE;
	position.horizontalY.q[1] = &config.positionKalman.CovStateGPSVelocity; //P_KALMAN_STATE_GPS_VELOCITY_NOISE;
	position.horizontalY.q[2] = &config.positionKalman.CovStateAcc; //P_KALMAN_STATE_ACC_NOISE;

	position.vertikalZ.q[0] = &config.positionKalman.CovStateGPSVertical; //P_KALMAN_STATE_GPS_VERTICAL_NOISE;
	position.vertikalZ.q[1] = &config.positionKalman.CovStateVerticalVelocity; //P_KALMAN_STATE_VERTICAL_VELOCITY_NOISE;
	position.vertikalZ.q[2] = &config.positionKalman.CovStateAcc; //P_KALMAN_STATE_ACC_NOISE;
	position.vertikalZ.q[3] = &config.positionKalman.CovStateBaroOffset; //P_KALMAN_STATE_BARO_OFFSET_NOISE;

	// sensor noise
	position.horizontalX.r[0] = &config.positionKalman.CovMeasGPSHorizontal; //P_KALMAN_GPS_HORIZONTAL_NOISE;
	position.horizontalX.r[1] = &config.positionKalman.CovMeasGPSVelocity; //P_KALMAN_GPS_VELOCITY_NOISE;
	position.horizontalX.r[2] = &config.positionKalman.CovMeasAcc; //P_KALMAN_ACC_NOISE;

	position.horizontalY.r[0] = &config.positionKalman.CovMeasGPSHorizontal; //P_KALMAN_GPS_HORIZONTAL_NOISE;
	position.horizontalY.r[1] = &config.positionKalman.CovMeasGPSVelocity; //P_KALMAN_GPS_VELOCITY_NOISE;
	position.horizontalY.r[2] = &config.positionKalman.CovMeasAcc; //P_KALMAN_ACC_NOISE;

	position.vertikalZ.r[0] = &config.positionKalman.CovMeasGPSVertical; //P_KALMAN_GPS_VERTICAL_NOISE;
	position.vertikalZ.r[1] = &config.positionKalman.CovMeasBarometer; //P_KALMAN_BAROMETER_NOISE;
	position.vertikalZ.r[2] = &config.positionKalman.CovMeasAcc; //P_KALMAN_ACC_NOISE;
}

/*
 * sets all state vectors to the measured values.
 * Must be called after an offset correction like
 * gps_SetHomePosition or pressure_SetHeight
 */
void imuPosition_Reset(void) {
	// reset Kalman filters
	position.horizontalX.x[0] = *position.horizontalX.z[0];
	position.horizontalX.x[1] = *position.horizontalX.z[1];
	position.horizontalX.x[2] = *position.horizontalX.z[2];

	position.horizontalY.x[0] = *position.horizontalY.z[0];
	position.horizontalY.x[1] = *position.horizontalY.z[1];
	position.horizontalY.x[2] = *position.horizontalY.z[2];

	position.vertikalZ.x[0] = *position.vertikalZ.z[0];
	position.vertikalZ.x[1] = 0;
	position.vertikalZ.x[2] = *position.vertikalZ.z[2];

	// update position values
	imuPosition_CopyFilterValues();
}

/*
 * updates the position data based on the values from one IMU sensor
 */
void imuPosition_Update(uint8_t sensor) {
	if (sensor == SENSOR_ACCELEROMETER) {
		/*
		 * calculate acceleration in the local navigation frame
		 * by applying reversed yaw, pitch and roll rotations to
		 * the accelerometer values
		 */
		position.acceleration.X = accelerometer.X * attitude.cosPitch
				* attitude.cosYaw
				+ accelerometer.Y
						* (attitude.cosYaw * attitude.sinPitch
								* attitude.sinRoll
								- attitude.cosRoll * attitude.sinYaw)
				+ accelerometer.Z
						* (attitude.cosRoll * attitude.cosYaw
								* attitude.sinPitch
								+ attitude.sinRoll * attitude.sinYaw);
		position.acceleration.Y = accelerometer.X * attitude.cosPitch
				* attitude.sinYaw
				+ accelerometer.Y
						* (attitude.cosRoll * attitude.cosYaw
								+ attitude.sinPitch * attitude.sinRoll
										* attitude.sinYaw)
				+ accelerometer.Z
						* (attitude.cosRoll * attitude.sinPitch
								* attitude.sinYaw
								- attitude.cosYaw * attitude.sinRoll);
		position.acceleration.Z = accelerometer.X * -attitude.sinPitch
				+ accelerometer.Y * attitude.cosPitch * attitude.sinRoll
				+ accelerometer.Z * attitude.cosPitch * attitude.cosRoll;
		// subtract the effect of gravity
		position.acceleration.Z += 1.0f;
		/*
		 * update Kalman Filters
		 */
		imuPosition_HorizontalUpdate(&position.horizontalX,
		SENSOR_ACCELEROMETER);
		imuPosition_HorizontalUpdate(&position.horizontalX,
		SENSOR_ACCELEROMETER);
		imuPosition_VerticalUpdate(&position.vertikalZ, SENSOR_ACCELEROMETER);
	} else if (sensor == SENSOR_BAROMETER) {
		imuPosition_VerticalUpdate(&position.vertikalZ, SENSOR_BAROMETER);
	} else if (sensor == SENSOR_GPS) {
		imuPosition_HorizontalUpdate(&position.horizontalX, SENSOR_GPS);
		imuPosition_HorizontalUpdate(&position.horizontalX, SENSOR_GPS);
		// TODO may include GPS data at some point but for now it is far too imprecise
		//imuPosition_VerticalUpdate(&position.vertikalZ, SENSOR_GPS);
	}

	// update position values
	imuPosition_CopyFilterValues();
}

/*
 * transfers the state variables from the kalman filters to
 * the position/velocity/acceleration struct
 */
void imuPosition_CopyFilterValues(void) {
	position.X = position.horizontalX.x[0];
	position.Y = position.horizontalY.x[0];
	position.Z = position.vertikalZ.x[0];

	position.velocity.X = position.horizontalX.x[1];
	position.velocity.Y = position.horizontalY.x[1];
	position.velocity.Z = position.vertikalZ.x[1];

	position.acceleration.X = position.horizontalX.x[2];
	position.acceleration.Y = position.horizontalY.x[2];
	position.acceleration.Z = position.vertikalZ.x[2];
}

/*
 * internal kalman filter functions
 */
void imuPosition_HorizontalUpdate(struct horizontalKalman *kal, uint8_t sensor) {
	// calculate cycle time in seconds
	uint32_t timeDifference = time_Get100us() - kal->timestamp;
	kal->timestamp = time_Get100us();
	float cycleTime = (float) timeDifference * 0.0001f;
	/****************************************************************
	 * state propagation
	 ***************************************************************/
	/*
	 * x = F*x
	 */
	kal->x[0] += kal->x[1] * cycleTime;
	kal->x[1] += kal->x[2] * cycleTime;
	/****************************************************************
	 * covariance propagation
	 * P = F*P*FT + Q
	 ***************************************************************/
	float pBuf[3][3];
	/*
	 * partial matrix multiplikation
	 * pBuf = F*P
	 */
	uint8_t i;
	for (i = 0; i < 3; i++) {
		// row 1
		pBuf[0][i] = kal->p[0][i] + kal->p[1][i] * cycleTime;
		// row 2
		pBuf[1][i] = kal->p[1][i] + kal->p[2][i] * cycleTime;
		// row 3
		pBuf[2][i] = kal->p[2][i];
	}
	/*
	 * P = bBuf*FT
	 */
	for (i = 0; i < 3; i++) {
		// column 1
		kal->p[i][0] = pBuf[i][0] + pBuf[i][1] * cycleTime;
		// column 2
		kal->p[i][1] = pBuf[i][1] + pBuf[i][2] * cycleTime;
		// column 3
		kal->p[i][2] = pBuf[i][2];
	}
	/*
	 * P = P + Q
	 */
	kal->p[0][0] += *kal->q[0];
	kal->p[1][1] += *kal->q[1];
	kal->p[2][2] += *kal->q[2];
	if (sensor == SENSOR_ACCELEROMETER) {
		/****************************************************************
		 * gain calculation
		 * K = P*HT*(H*P*HT+R)^-1
		 ***************************************************************/
		/*
		 * in this case, measurement matrix H:
		 * 	0	0	1
		 *
		 * 	H*P*HT = P[2][2]
		 * 	       /P[0][2]\
		 * 	P*HT = |P[1][2]|
		 * 	       \P[2][2]/
		 * 	s := (H*P*HT+R)^-1 = 1/(P[2][2]+r[2])
		 */
		float s = 1.0f / (kal->p[2][2] + *kal->r[2]);
		float k[3];
		k[0] = kal->p[0][2] * s;
		k[1] = kal->p[1][2] * s;
		k[2] = kal->p[2][2] * s;
		/****************************************************************
		 * state update
		 * x = x+K*(z-H*x)
		 ***************************************************************/
		/*
		 * innovation dz = z-H*x
		 * here: dz = z[2] - x[2]
		 */
		float dz = *(kal->z[2]) - kal->x[2];
		kal->x[0] += k[0] * dz;
		kal->x[1] += k[1] * dz;
		kal->x[2] += k[2] * dz;
		/****************************************************************
		 * covariance update
		 * P = P - K*(H*P)
		 ***************************************************************/
		/*
		 * cor := K*(H*P)
		 * here:
		 * cor(i,j) = k[i]*p[2][j]
		 */
		uint8_t j;
		float cor[3][3];
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				cor[i][j] = k[i] * kal->p[2][j];
			}
		}
		/*
		 * P = P - cor
		 */
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				kal->p[i][j] -= cor[i][j];
			}
		}
	} else if (sensor == SENSOR_GPS) {
		/****************************************************************
		 * gain calculation
		 * K = P*HT*(H*P*HT+R)^-1
		 ***************************************************************/
		/*
		 * in this case, measurement matrix H:
		 * 	1	0	0
		 * 	0	1	0
		 *
		 * 	s := (H*P*HT+R)^-1
		 */
		float s[2][2];
		s[0][0] = kal->p[0][0] + *kal->r[0];
		s[0][1] = kal->p[0][1];
		s[1][0] = kal->p[1][0];
		s[1][1] = kal->p[1][1] + *kal->r[1];
		// calculate inverse s
		float invDetS = 1.0f / (s[0][0] * s[1][1] - s[0][1] * s[1][0]);
		float s00 = s[0][0];
		s[0][0] = s[1][1] * invDetS;
		s[0][1] = -s[0][1] * invDetS;
		s[1][0] = -s[1][0] * invDetS;
		s[1][1] = s00 * invDetS;
		float k[3][2];
		/*
		 * k = P*HT*S
		 * P*HT = left two-thirds of P
		 */
		for (i = 0; i < 3; i++) {
			k[i][0] = kal->p[i][0] * s[0][0] + kal->p[i][1] * s[1][0];
			k[i][1] = kal->p[i][0] * s[0][1] + kal->p[i][1] * s[1][1];
		}
		/****************************************************************
		 * state update
		 * x = x+K*(z-H*x)
		 ***************************************************************/
		/*
		 * innovation dz = z-H*x
		 */
		float dz[2];
		dz[0] = *(kal->z[0]) - kal->x[0];
		dz[1] = *(kal->z[1]) - kal->x[1];
		// x += K*dz
		kal->x[0] += k[0][0] * dz[0] + k[0][1] * dz[1];
		kal->x[1] += k[1][0] * dz[0] + k[1][1] * dz[1];
		kal->x[2] += k[2][0] * dz[0] + k[2][1] * dz[1];
		/****************************************************************
		 * covariance update
		 * P = P - K*(H*P)
		 ***************************************************************/
		/*
		 * cor := K*(H*P)
		 */
		float cor[3][3];
		uint8_t j;
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				cor[i][j] = k[i][0] * kal->p[0][j] + k[i][1] * kal->p[1][j];
			}
		}
		/*
		 * P = P - cor
		 */
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				kal->p[i][j] -= cor[i][j];
			}
		}
	}
}
void imuPosition_VerticalUpdate(struct verticalKalman *kal, uint8_t sensor) {
	// calculate cycle time in seconds
	uint32_t timeDifference = time_Get100us() - kal->timestamp;
	kal->timestamp = time_Get100us();
	float cycleTime = (float) timeDifference * 0.0001f;
	/****************************************************************
	 * state propagation
	 ***************************************************************/
	/*
	 * x = F*x
	 */
	kal->x[0] += kal->x[1] * cycleTime;
	kal->x[1] += kal->x[2] * cycleTime;
	/****************************************************************
	 * covariance propagation
	 * P = F*P*FT + Q
	 ***************************************************************/
	float pBuf[4][4];
	/*
	 * partial matrix multiplikation
	 * pBuf = F*P
	 */
	uint8_t i;
	for (i = 0; i < 4; i++) {
		// row 1
		pBuf[0][i] = kal->p[0][i] + kal->p[1][i] * cycleTime;
		// row 2
		pBuf[1][i] = kal->p[1][i] + kal->p[2][i] * cycleTime;
		// row 3+4
		pBuf[2][i] = kal->p[2][i];
		pBuf[3][i] = kal->p[3][i];
	}
	/*
	 * P = bBuf*FT
	 */
	for (i = 0; i < 4; i++) {
		// column 1
		kal->p[i][0] = pBuf[i][0] + pBuf[i][1] * cycleTime;
		// column 2
		kal->p[i][1] = pBuf[i][1] + pBuf[i][2] * cycleTime;
		// column 3+4
		kal->p[i][2] = pBuf[i][2];
		kal->p[i][3] = pBuf[i][3];
	}
	/*
	 * P = P + Q
	 */
	kal->p[0][0] += *kal->q[0];
	kal->p[1][1] += *kal->q[1];
	kal->p[2][2] += *kal->q[2];
	kal->p[2][3] += *kal->q[3];
	/****************************************************************
	 * gain calculation
	 * K = P*HT*(H*P*HT+R)^-1
	 ***************************************************************/
	float k[4], s;
	switch (sensor) {
	case SENSOR_ACCELEROMETER:
		/*
		 * measurement matrix H:
		 * 	0	0	-1	0
		 */
		s = 1.0f / (kal->p[2][2] + *kal->r[2]);
		k[0] = -kal->p[0][2] * s;
		k[1] = -kal->p[1][2] * s;
		k[2] = -kal->p[2][2] * s;
		k[3] = -kal->p[3][2] * s;
		break;
	case SENSOR_GPS:
		/*
		 * measurement matrix H:
		 * 	1	0	0	0
		 */
		s = 1.0f / (kal->p[0][0] + *kal->r[0]);
		k[0] = kal->p[0][0] * s;
		k[1] = kal->p[1][0] * s;
		k[2] = kal->p[2][0] * s;
		k[3] = kal->p[3][0] * s;
		break;
	case SENSOR_BAROMETER:
		/*
		 * measurement matrix H:
		 * 	1	0	0	1
		 */
		s = 1.0f
				/ (kal->p[0][0] + kal->p[3][0] + kal->p[0][3] + kal->p[3][3]
						+ *kal->r[1]);
		k[0] = (kal->p[0][0] + kal->p[3][0]) * s;
		k[1] = (kal->p[0][1] + kal->p[3][1]) * s;
		k[2] = (kal->p[0][2] + kal->p[3][2]) * s;
		k[3] = (kal->p[0][3] + kal->p[3][3]) * s;
		break;
	}
	/****************************************************************
	 * state update
	 * x = x+K*(z-H*x)
	 ***************************************************************/
	/*
	 * innovation dz = z-H*x
	 * here: dz = z[2] - x[2]
	 */
	float dz = 0;
	switch (sensor) {
	case SENSOR_ACCELEROMETER:
		dz = *(kal->z[2]) + kal->x[2];
		break;
	case SENSOR_GPS:
		dz = *(kal->z[0]) - kal->x[0];
		break;
	case SENSOR_BAROMETER:
		dz = *(kal->z[1]) - kal->x[0] - kal->x[3];
		break;
	}
	kal->x[0] += k[0] * dz;
	kal->x[1] += k[1] * dz;
	kal->x[2] += k[2] * dz;
	kal->x[3] += k[3] * dz;
	/****************************************************************
	 * covariance update
	 * P = P - K*(H*P)
	 ***************************************************************/
	/*
	 * cor := K*(H*P)
	 */
	uint8_t j;
	float cor[4][4];
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			switch (sensor) {
			case SENSOR_ACCELEROMETER:
				cor[i][j] = k[i] * -kal->p[2][j];
				break;
			case SENSOR_GPS:
				cor[i][j] = k[i] * kal->p[0][j];
				break;
			case SENSOR_BAROMETER:
				cor[i][j] = k[i] * (kal->p[0][j] + kal->p[3][j]);
				break;
			}
		}
	}
	/*
	 * P = P - cor
	 */
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			kal->p[i][j] -= cor[i][j];
		}
	}
}
