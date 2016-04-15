/*
 * kalman.c
 *
 *  Created on: Nov 1, 2013
 *      Author: jan
 */

#include "kalman.h"

/*
 * initializes the kalman matrices
 */
void kalman_Init(void) {
	// set measurement vector
	kalman.z[KALMAN_Z_ACC_X] = &accelerometer.X;
	kalman.z[KALMAN_Z_ACC_Y] = &accelerometer.Y;
	kalman.z[KALMAN_Z_ACC_Z] = &accelerometer.Z;
	kalman.z[KALMAN_Z_MAG_X] = &magnetometer.X;
	kalman.z[KALMAN_Z_MAG_Y] = &magnetometer.Y;
	kalman.z[KALMAN_Z_MAG_Z] = &magnetometer.Z;
	kalman.z[KALMAN_Z_GYRO_X] = &gyro.X;
	kalman.z[KALMAN_Z_GYRO_Y] = &gyro.Y;
	kalman.z[KALMAN_Z_GYRO_Z] = &gyro.Z;
	// set measurement noise
	kalman.r[KALMAN_Z_ACC_X] = &config.attitudeKalman.CovMeasAcc; //KALMAN_ACC_NOISE;
	kalman.r[KALMAN_Z_ACC_Y] = &config.attitudeKalman.CovMeasAcc; //KALMAN_ACC_NOISE;
	kalman.r[KALMAN_Z_ACC_Z] = &config.attitudeKalman.CovMeasAcc; //KALMAN_ACC_NOISE;
	kalman.r[KALMAN_Z_MAG_X] = &config.attitudeKalman.CovMeasMag; //KALMAN_MAG_NOISE;
	kalman.r[KALMAN_Z_MAG_Y] = &config.attitudeKalman.CovMeasMag; //KALMAN_MAG_NOISE;
	kalman.r[KALMAN_Z_MAG_Z] = &config.attitudeKalman.CovMeasMag; //KALMAN_MAG_NOISE;
	kalman.r[KALMAN_Z_GYRO_X] = &config.attitudeKalman.CovMeasGyro; //KALMAN_GYRO_NOISE;
	kalman.r[KALMAN_Z_GYRO_Y] = &config.attitudeKalman.CovMeasGyro; //KALMAN_GYRO_NOISE;
	kalman.r[KALMAN_Z_GYRO_Z] = &config.attitudeKalman.CovMeasGyro; //KALMAN_GYRO_NOISE;
	// kalman state noise
	kalman.q[KALMAN_X_ACC_X] = &config.attitudeKalman.CovStateAcc;//KALMAN_STATE_ACC_NOISE;
	kalman.q[KALMAN_X_ACC_Y] = &config.attitudeKalman.CovStateAcc;//KALMAN_STATE_ACC_NOISE;
	kalman.q[KALMAN_X_ACC_Z] = &config.attitudeKalman.CovStateAcc;//KALMAN_STATE_ACC_NOISE;
	kalman.q[KALMAN_X_MAG_X] = &config.attitudeKalman.CovStateMag;//KALMAN_STATE_MAG_NOISE;
	kalman.q[KALMAN_X_MAG_Y] = &config.attitudeKalman.CovStateMag;//KALMAN_STATE_MAG_NOISE;
	kalman.q[KALMAN_X_MAG_Z] = &config.attitudeKalman.CovStateMag;//KALMAN_STATE_MAG_NOISE;
	kalman.q[KALMAN_X_GYRO_X] = &config.attitudeKalman.CovStateGyro;//KALMAN_STATE_GYRO_NOISE;
	kalman.q[KALMAN_X_GYRO_Y] = &config.attitudeKalman.CovStateGyro;//KALMAN_STATE_GYRO_NOISE;
	kalman.q[KALMAN_X_GYRO_Z] = &config.attitudeKalman.CovStateGyro;//KALMAN_STATE_GYRO_NOISE;
	kalman.q[KALMAN_X_BIAS_X] = &config.attitudeKalman.CovStateGyroBias;//KALMAN_STATE_BIAS_NOISE;
	kalman.q[KALMAN_X_BIAS_Y] = &config.attitudeKalman.CovStateGyroBias;//KALMAN_STATE_BIAS_NOISE;
	kalman.q[KALMAN_X_BIAS_Z] = &config.attitudeKalman.CovStateGyroBias;//KALMAN_STATE_BIAS_NOISE;
}
/*
 * calculates the next iteration of the kalman filter
 */
void kalman_Update(void) {
	// calculate time-interval since last update
	uint32_t currentTime = time_Get100us();
	kalman.timeDiff = (currentTime - kalman.timestamp) * 0.0001f;
	kalman.timestamp = currentTime;
	kalman_StatePropagation();
	kalman_CovariancePropagation();
	kalman_CalculateGain();
	kalman_StateUpdate();
	kalman_CovarianceUpdate();
	//GPIOB->BSRRH = GPIO_Pin_8;
}
/*
 * overwrites the kalman state with the current measured values
 */
void kalman_Reset() {
	// copy values from measurement vector into kalman state
	kalman.x[KALMAN_X_ACC_X] = *kalman.z[KALMAN_Z_ACC_X];
	kalman.x[KALMAN_X_ACC_Y] = *kalman.z[KALMAN_Z_ACC_Y];
	kalman.x[KALMAN_X_ACC_Z] = *kalman.z[KALMAN_Z_ACC_Z];
	kalman.x[KALMAN_X_MAG_X] = *kalman.z[KALMAN_Z_MAG_X];
	kalman.x[KALMAN_X_MAG_Y] = *kalman.z[KALMAN_Z_MAG_Y];
	kalman.x[KALMAN_X_MAG_Z] = *kalman.z[KALMAN_Z_MAG_Z];
	kalman.x[KALMAN_X_GYRO_X] = *kalman.z[KALMAN_Z_GYRO_X];
	kalman.x[KALMAN_X_GYRO_Y] = *kalman.z[KALMAN_Z_GYRO_Y];
	kalman.x[KALMAN_X_GYRO_Z] = *kalman.z[KALMAN_Z_GYRO_Z];
	kalman.x[KALMAN_X_BIAS_X] = 0;
	kalman.x[KALMAN_X_BIAS_Y] = 0;
	kalman.x[KALMAN_X_BIAS_Z] = 0;
	kalman.timestamp = time_Get100us();
}
void kalman_Print(void) {
	stdComm_puts("kalman Xa:");
	stdComm_PrintValue(kalman.x[0] * 1000);
	stdComm_puts(" Ya:");
	stdComm_PrintValue(kalman.x[1] * 1000);
	stdComm_puts(" Za:");
	stdComm_PrintValue(kalman.x[2] * 1000);
	stdComm_puts(" Xm:");
	stdComm_PrintValue(kalman.x[3] * 1000);
	stdComm_puts(" Ym:");
	stdComm_PrintValue(kalman.x[4] * 1000);
	stdComm_puts(" Zm:");
	stdComm_PrintValue(kalman.x[5] * 1000);
	stdComm_puts(" Xg:");
	stdComm_PrintValue(kalman.x[6] * RAD_TO_DEG);
	stdComm_puts(" Yg:");
	stdComm_PrintValue(kalman.x[7] * RAD_TO_DEG);
	stdComm_puts(" Zg:");
	stdComm_PrintValue(kalman.x[8] * RAD_TO_DEG);
	stdComm_puts(" Xb:");
	stdComm_PrintValue(kalman.x[9] * RAD_TO_DEG);
	stdComm_puts(" Yb:");
	stdComm_PrintValue(kalman.x[10] * RAD_TO_DEG);
	stdComm_puts(" Zb:");
	stdComm_PrintValue(kalman.x[11] * RAD_TO_DEG);
	usart_putcStdComm(0x0A);
}
// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(float* vector, float* delta) {
	float tmp[3] = { vector[0], vector[1], vector[2] };
	vector[2] += -delta[1] * tmp[0] - delta[0] * tmp[1];
	vector[0] += delta[1] * tmp[2] + delta[2] * tmp[1];
	vector[1] += delta[0] * tmp[2] - delta[2] * tmp[0];
}
void kalman_StatePropagation(void) {
	//GPIOB->BSRRL = GPIO_Pin_8;
	float scaledVelocity[3];
	// calculate changes in rad
	scaledVelocity[0] = kalman.x[KALMAN_X_GYRO_X] * kalman.timeDiff;
	scaledVelocity[1] = kalman.x[KALMAN_X_GYRO_Y] * kalman.timeDiff;
	scaledVelocity[2] = kalman.x[KALMAN_X_GYRO_Z] * kalman.timeDiff;
	// rotation of the acceleration vector
	rotateV(&kalman.x[0], scaledVelocity);
	// rotation of the magnetic field vector
	rotateV(&kalman.x[3], scaledVelocity);
}
void kalman_CovariancePropagation(void) {
	//GPIOB->BSRRH = GPIO_Pin_8;
	/*
	 * overall equation: P- = F*P+*FT + Q
	 *
	 * P1 = F*P+
	 * partial matrix multiplication since most elements in F are 0
	 */
	float p1[KALMAN_STATES][KALMAN_STATES];
	uint8_t i;
	for (i = 0; i < KALMAN_STATES; i++) {
		// row 1
		p1[0][i] = kalman.pValues[KALMAN_X_ACC_X][i]
				+ kalman.pValues[KALMAN_X_ACC_Y][i] * kalman.x[KALMAN_X_GYRO_Z]
						* kalman.timeDiff
				+ kalman.pValues[KALMAN_X_ACC_Z][i] * kalman.x[KALMAN_X_GYRO_Y]
						* kalman.timeDiff
				+ kalman.pValues[KALMAN_X_GYRO_Y][i] * kalman.x[KALMAN_X_ACC_Z]
						* kalman.timeDiff
				+ kalman.pValues[KALMAN_X_GYRO_Z][i] * kalman.x[KALMAN_X_ACC_Y]
						* kalman.timeDiff;
		// row 2
		p1[1][i] = kalman.pValues[KALMAN_X_ACC_Y][i]
				- kalman.pValues[KALMAN_X_ACC_X][i] * kalman.x[KALMAN_X_GYRO_Z]
						* kalman.timeDiff
				+ kalman.pValues[KALMAN_X_ACC_Z][i] * kalman.x[KALMAN_X_GYRO_X]
						* kalman.timeDiff
				+ kalman.pValues[KALMAN_X_GYRO_X][i] * kalman.x[KALMAN_X_ACC_Z]
						* kalman.timeDiff
				- kalman.pValues[KALMAN_X_GYRO_Z][i] * kalman.x[KALMAN_X_ACC_X]
						* kalman.timeDiff;
		// row 3
		p1[2][i] = kalman.pValues[KALMAN_X_ACC_Z][i]
				- kalman.pValues[KALMAN_X_ACC_X][i] * kalman.x[KALMAN_X_GYRO_Y]
						* kalman.timeDiff
				- kalman.pValues[KALMAN_X_ACC_Y][i] * kalman.x[KALMAN_X_GYRO_X]
						* kalman.timeDiff
				- kalman.pValues[KALMAN_X_GYRO_X][i] * kalman.x[KALMAN_X_ACC_Y]
						* kalman.timeDiff
				- kalman.pValues[KALMAN_X_GYRO_Y][i] * kalman.x[KALMAN_X_ACC_X]
						* kalman.timeDiff;
		// row 4
		p1[3][i] = kalman.pValues[KALMAN_X_MAG_X][i]
				+ kalman.pValues[KALMAN_X_MAG_Y][i] * kalman.x[KALMAN_X_GYRO_Z]
						* kalman.timeDiff
				+ kalman.pValues[KALMAN_X_MAG_Z][i] * kalman.x[KALMAN_X_GYRO_Y]
						* kalman.timeDiff
				+ kalman.pValues[KALMAN_X_GYRO_Y][i] * kalman.x[KALMAN_X_MAG_Z]
						* kalman.timeDiff
				+ kalman.pValues[KALMAN_X_GYRO_Z][i] * kalman.x[KALMAN_X_MAG_Y]
						* kalman.timeDiff;
		// row 5
		p1[4][i] = kalman.pValues[KALMAN_X_MAG_Y][i]
				- kalman.pValues[KALMAN_X_MAG_X][i] * kalman.x[KALMAN_X_GYRO_Z]
						* kalman.timeDiff
				+ kalman.pValues[KALMAN_X_MAG_Z][i] * kalman.x[KALMAN_X_GYRO_X]
						* kalman.timeDiff
				+ kalman.pValues[KALMAN_X_GYRO_X][i] * kalman.x[KALMAN_X_MAG_Z]
						* kalman.timeDiff
				- kalman.pValues[KALMAN_X_GYRO_Z][i] * kalman.x[KALMAN_X_MAG_X]
						* kalman.timeDiff;
		// row 6
		p1[5][i] = kalman.pValues[KALMAN_X_MAG_Z][i]
				- kalman.pValues[KALMAN_X_MAG_X][i] * kalman.x[KALMAN_X_GYRO_Y]
						* kalman.timeDiff
				- kalman.pValues[KALMAN_X_MAG_Y][i] * kalman.x[KALMAN_X_GYRO_X]
						* kalman.timeDiff
				- kalman.pValues[KALMAN_X_GYRO_X][i] * kalman.x[KALMAN_X_MAG_Y]
						* kalman.timeDiff
				- kalman.pValues[KALMAN_X_GYRO_Y][i] * kalman.x[KALMAN_X_MAG_X]
						* kalman.timeDiff;
		// rows 7-12 are unchanged since the lower right part of f is the identity matrix
		p1[6][i] = kalman.pValues[6][i];
		p1[7][i] = kalman.pValues[7][i];
		p1[8][i] = kalman.pValues[8][i];
		p1[9][i] = kalman.pValues[9][i];
		p1[10][i] = kalman.pValues[10][i];
		p1[11][i] = kalman.pValues[11][i];
	}
	/*
	 * P2 = P1*FT
	 * again partial matrix transformation
	 */
	for (i = 0; i < KALMAN_STATES; i++) {
		// column 1
		kalman.pValues[i][0] = p1[i][KALMAN_X_ACC_X]
				+ p1[i][KALMAN_X_ACC_Y] * kalman.x[KALMAN_X_GYRO_Z]
						* kalman.timeDiff
				+ p1[i][KALMAN_X_ACC_Z] * kalman.x[KALMAN_X_GYRO_Y]
						* kalman.timeDiff
				+ p1[i][KALMAN_X_GYRO_Y] * kalman.x[KALMAN_X_ACC_Z]
						* kalman.timeDiff
				+ p1[i][KALMAN_X_GYRO_Z] * kalman.x[KALMAN_X_ACC_Y]
						* kalman.timeDiff;
		// column 2
		kalman.pValues[i][1] = p1[i][KALMAN_X_ACC_Y]
				- p1[i][KALMAN_X_ACC_X] * kalman.x[KALMAN_X_GYRO_Z]
						* kalman.timeDiff
				+ p1[i][KALMAN_X_ACC_Z] * kalman.x[KALMAN_X_GYRO_X]
						* kalman.timeDiff
				+ p1[i][KALMAN_X_GYRO_X] * kalman.x[KALMAN_X_ACC_Z]
						* kalman.timeDiff
				- p1[i][KALMAN_X_GYRO_Z] * kalman.x[KALMAN_X_ACC_X]
						* kalman.timeDiff;
		// column 3
		kalman.pValues[i][2] = p1[i][KALMAN_X_ACC_Z]
				- p1[i][KALMAN_X_ACC_X] * kalman.x[KALMAN_X_GYRO_Y]
						* kalman.timeDiff
				- p1[i][KALMAN_X_ACC_Y] * kalman.x[KALMAN_X_GYRO_X]
						* kalman.timeDiff
				- p1[i][KALMAN_X_GYRO_X] * kalman.x[KALMAN_X_ACC_Y]
						* kalman.timeDiff
				- p1[i][KALMAN_X_GYRO_Y] * kalman.x[KALMAN_X_ACC_X]
						* kalman.timeDiff;
		// column 4
		kalman.pValues[i][3] = p1[i][KALMAN_X_MAG_X]
				+ p1[i][KALMAN_X_MAG_Y] * kalman.x[KALMAN_X_GYRO_Z]
						* kalman.timeDiff
				+ p1[i][KALMAN_X_MAG_Z] * kalman.x[KALMAN_X_GYRO_Y]
						* kalman.timeDiff
				+ p1[i][KALMAN_X_GYRO_Y] * kalman.x[KALMAN_X_MAG_Z]
						* kalman.timeDiff
				+ p1[i][KALMAN_X_GYRO_Z] * kalman.x[KALMAN_X_MAG_Y]
						* kalman.timeDiff;
		// column 5
		kalman.pValues[i][4] = p1[i][KALMAN_X_MAG_Y]
				- p1[i][KALMAN_X_MAG_X] * kalman.x[KALMAN_X_GYRO_Z]
						* kalman.timeDiff
				+ p1[i][KALMAN_X_MAG_Z] * kalman.x[KALMAN_X_GYRO_X]
						* kalman.timeDiff
				+ p1[i][KALMAN_X_GYRO_X] * kalman.x[KALMAN_X_MAG_Z]
						* kalman.timeDiff
				- p1[i][KALMAN_X_GYRO_Z] * kalman.x[KALMAN_X_MAG_X]
						* kalman.timeDiff;
		// column 6
		kalman.pValues[i][5] = p1[i][KALMAN_X_MAG_Z]
				- p1[i][KALMAN_X_MAG_X] * kalman.x[KALMAN_X_GYRO_Y]
						* kalman.timeDiff
				- p1[i][KALMAN_X_MAG_Y] * kalman.x[KALMAN_X_GYRO_X]
						* kalman.timeDiff
				- p1[i][KALMAN_X_GYRO_X] * kalman.x[KALMAN_X_MAG_Y]
						* kalman.timeDiff
				- p1[i][KALMAN_X_GYRO_Y] * kalman.x[KALMAN_X_MAG_X]
						* kalman.timeDiff;
		// columns 7-12 are unchanged since the lower right part of f is the identity matrix
		kalman.pValues[i][6] = p1[i][6];
		kalman.pValues[i][7] = p1[i][7];
		kalman.pValues[i][8] = p1[i][8];
		kalman.pValues[i][9] = p1[i][9];
		kalman.pValues[i][10] = p1[i][10];
		kalman.pValues[i][11] = p1[i][11];
	}
	/*
	 * P- = P2+Q
	 * partial matrix addition since Q has only elements != 0 on the diagonal axis
	 */
	for (i = 0; i < KALMAN_STATES; i++) {
#ifdef KALMAN_FORCE_SYMMETRY_P
		if (kalman.pValues[i][i] < kalman.q[i])
		kalman.pValues[i][i] = kalman.q[i];
		else
#endif
		kalman.pValues[i][i] += *kalman.q[i];
	}
#ifdef KALMAN_FORCE_SYMMETRY_P
	/*
	 * averaging P with PT to avoid distortion (P must always be symmetrical)
	 */
	for (i = 0; i < KALMAN_STATES - 1; i++) {
		uint8_t j;
		for (j = i + 1; j < KALMAN_STATES; j++) {
			float avg = kalman.pValues[i][j] + kalman.pValues[j][i];
			avg *= 0.5;
			kalman.pValues[i][j] = kalman.pValues[j][i] = avg;
		}
	}
#endif
}
void kalman_CalculateGain(void) {
	//GPIOB->BSRRL = GPIO_Pin_8;
	/*
	 * overall equation: K = P-*HT*(H*P-*HT+R)^-1
	 * with intermediate matrices:
	 * Buf = P-*HT
	 * S = H*P-*HT+R
	 *
	 * Buf = P-*HT
	 */
	float bufValues[KALMAN_STATES][KALMAN_MEASUREMENTS];
	uint8_t i;
	for (i = 0; i < KALMAN_STATES; i++) {
		// columns 1-6: Buf is identical to P-
		bufValues[i][0] = kalman.pValues[i][0];
		bufValues[i][1] = kalman.pValues[i][1];
		bufValues[i][2] = kalman.pValues[i][2];
		bufValues[i][3] = kalman.pValues[i][3];
		bufValues[i][4] = kalman.pValues[i][4];
		bufValues[i][5] = kalman.pValues[i][5];
		// columns 7-9: remove bias (see measurement matrix H)
		bufValues[i][6] = kalman.pValues[i][6] + kalman.pValues[i][9];
		bufValues[i][7] = kalman.pValues[i][7] + kalman.pValues[i][10];
		bufValues[i][8] = kalman.pValues[i][8] + kalman.pValues[i][11];
	}
	/*
	 * S = H*P-*HT + R
	 * => S = H*Buf + R
	 */
	float sValues[KALMAN_MEASUREMENTS][KALMAN_MEASUREMENTS];
	// S1 = H*Buf
	for (i = 0; i < KALMAN_MEASUREMENTS; i++) {
		// rows 1-6: S1 is identical to Buf
		sValues[0][i] = bufValues[0][i];
		sValues[1][i] = bufValues[1][i];
		sValues[2][i] = bufValues[2][i];
		sValues[3][i] = bufValues[3][i];
		sValues[4][i] = bufValues[4][i];
		sValues[5][i] = bufValues[5][i];
		// rows 7-9: remove bias (see measurement matrix H)
		sValues[6][i] = bufValues[6][i] + bufValues[9][i];
		sValues[7][i] = bufValues[7][i] + bufValues[10][i];
		sValues[8][i] = bufValues[8][i] + bufValues[11][i];
	}
	// S = S1 + R
	for (i = 0; i < KALMAN_MEASUREMENTS; i++) {
		sValues[i][i] += *kalman.r[i];
	}
	// variance for the accelerometer and magnetometer depends on the
	// magnitude of the measured vector (-> should ideally be 1, deviation
	// from this value indicates measurement error -> greater variance)
	// TODO this uses config values directly -> consider pointer to these values in the kalman filter struct
	float errorAcc = (accelerometer.magnitude - 1.0f);
	errorAcc *= errorAcc;
	float errorMag = (magnetometer.magnitude - 1.0f);
	errorMag *= errorMag;
	for (i = 0; i < 3; i++) {
		sValues[KALMAN_X_ACC_X + i][KALMAN_X_ACC_X + i] += errorAcc
				* config.attitudeKalman.CovAccError;
		sValues[KALMAN_X_MAG_X + i][KALMAN_X_MAG_X + i] += errorMag
				* config.attitudeKalman.CovMagError;
	}
	// sinv = s^-1
	float sinvValues[KALMAN_MEASUREMENTS][KALMAN_MEASUREMENTS];
	/*
	 * this implementation of the matrix inversion is actually quite a bit
	 * slower than the one in the DSP library. But the DSP library fails to
	 * invert matrices who are close to a diagonal matrix (e.i. nearly all
	 * entries outside the main diagonal are zero)
	 */
	kalman_InverseMatrix(&sValues[0][0], &sinvValues[0][0],
	KALMAN_MEASUREMENTS);

	/*
	 * K = P-*HT*(H*P-*HT+R)^-1
	 * => K = P-*HT*SInv
	 * => K = Buf*SInv
	 */
	// manual matrix multiplication
	for (i = 0; i < KALMAN_STATES; i++) {
		uint8_t j;
		for (j = 0; j < KALMAN_MEASUREMENTS; j++) {
			uint8_t k;
			kalman.kValues[i][j] = 0;
			for (k = 0; k < KALMAN_MEASUREMENTS; k++) {
				kalman.kValues[i][j] += bufValues[i][k] * sinvValues[k][j];
			}
		}
	}
}
void kalman_StateUpdate(void) {
	//GPIOB->BSRRH = GPIO_Pin_8;
	/*
	 * overall equation: x+ = x-+K*(z-H*x-)
	 * with intermediate result:
	 * innovation dz = z-H*x
	 */
	float dz[KALMAN_MEASUREMENTS];
	uint8_t i;
	/*
	 * dz = z-H*x-
	 */
	// acceleration and magnetic field are directly measured
	dz[KALMAN_Z_ACC_X] = *kalman.z[KALMAN_Z_ACC_X] - kalman.x[KALMAN_X_ACC_X];
	dz[KALMAN_Z_ACC_Y] = *kalman.z[KALMAN_Z_ACC_Y] - kalman.x[KALMAN_X_ACC_Y];
	dz[KALMAN_Z_ACC_Z] = *kalman.z[KALMAN_Z_ACC_Z] - kalman.x[KALMAN_X_ACC_Z];
	dz[KALMAN_Z_MAG_X] = *kalman.z[KALMAN_Z_MAG_X] - kalman.x[KALMAN_X_MAG_X];
	dz[KALMAN_Z_MAG_Y] = *kalman.z[KALMAN_Z_MAG_Y] - kalman.x[KALMAN_X_MAG_Y];
	dz[KALMAN_Z_MAG_Z] = *kalman.z[KALMAN_Z_MAG_Z] - kalman.x[KALMAN_X_MAG_Z];
	// gyro value is biased
	dz[KALMAN_Z_GYRO_X] = *kalman.z[KALMAN_Z_GYRO_X]
			- (kalman.x[KALMAN_X_GYRO_X] + kalman.x[KALMAN_X_BIAS_X]);
	dz[KALMAN_Z_GYRO_Y] = *kalman.z[KALMAN_Z_GYRO_Y]
			- (kalman.x[KALMAN_X_GYRO_Y] + kalman.x[KALMAN_X_BIAS_Y]);
	dz[KALMAN_Z_GYRO_Z] = *kalman.z[KALMAN_Z_GYRO_Z]
			- (kalman.x[KALMAN_X_GYRO_Z] + kalman.x[KALMAN_X_BIAS_Z]);
	/*
	 * x+ = x-+K*dz
	 * => x += K*dz
	 */
	for (i = 0; i < KALMAN_STATES; i++) {
		uint8_t j;
		for (j = 0; j < KALMAN_MEASUREMENTS; j++) {
			kalman.x[i] += kalman.kValues[i][j] * dz[j];
		}
	}
}
void kalman_CovarianceUpdate(void) {
	//GPIOB->BSRRL = GPIO_Pin_8;
	/*
	 * overall equation: P+ = P- -K*(H*P-)
	 * with intermediate result:
	 * Buf = H*P-
	 * Cor = K*Buf
	 */
	float bufValues[KALMAN_MEASUREMENTS][KALMAN_STATES];
	uint8_t i;
	// Buf = H*P-
	for (i = 0; i < KALMAN_STATES; i++) {
		// rows 1-6: S1 is identical to Buf
		bufValues[0][i] = kalman.pValues[0][i];
		bufValues[1][i] = kalman.pValues[1][i];
		bufValues[2][i] = kalman.pValues[2][i];
		bufValues[3][i] = kalman.pValues[3][i];
		bufValues[4][i] = kalman.pValues[4][i];
		bufValues[5][i] = kalman.pValues[5][i];
		// rows 7-9: remove bias (see measurement matrix H)
		bufValues[6][i] = kalman.pValues[6][i] + kalman.pValues[9][i];
		bufValues[7][i] = kalman.pValues[7][i] + kalman.pValues[10][i];
		bufValues[8][i] = kalman.pValues[8][i] + kalman.pValues[11][i];
	}
	float corValues[KALMAN_STATES][KALMAN_STATES];
	/* manual matrix multiplikation (for speed comparisation)
	 * -> this results in nearly 3 times faster multiplication, the DSP
	 * library must be pretty slow
	 */
	for (i = 0; i < KALMAN_STATES; i++) {
		uint8_t j;
		for (j = 0; j < KALMAN_STATES; j++) {
			uint8_t k;
			corValues[i][j] = 0;
			for (k = 0; k < KALMAN_MEASUREMENTS; k++) {
				corValues[i][j] += kalman.kValues[i][k] * bufValues[k][j];
			}
		}
	}
	/*
	 * P+ = P- -Cor
	 * => P -= Cor
	 */
	for (i = 0; i < KALMAN_STATES; i++) {
		uint8_t j;
		for (j = 0; j < KALMAN_STATES; j++) {
			kalman.pValues[i][j] -= corValues[i][j];
		}
	}
#ifdef KALMAN_FORCE_SYMMETRY_P
	/*
	 * averaging P with PT to avoid distortion (P must always be symmetrical)
	 */
	for (i = 0; i < KALMAN_STATES - 1; i++) {
		uint8_t j;
		for (j = i + 1; j < KALMAN_STATES; j++) {
			float avg = kalman.pValues[i][j] + kalman.pValues[j][i];
			avg *= 0.5;
			kalman.pValues[i][j] = kalman.pValues[j][i] = avg;
		}
		if(kalman.pValues[i][i]<kalman.q[i])
		kalman.pValues[i][i] = kalman.q[i];
	}
	if(kalman.pValues[8][8]<kalman.q[8])
	kalman.pValues[8][8] = kalman.q[8];
#endif
}
/*
 * matrix inversion using Gauss Jordan Elimination Method
 * @param a two-dimensional array containing the input matrix
 * @param b two-dimensional array containing the output matrix
 * @param n size of the matrices
 */
void kalman_InverseMatrix(float *a, float *b, uint8_t n) {
	// set b to the identity matrix
	uint8_t i = 0, j = 0, p = 0, q = 0;
	float tem = 0, temp = 0, temp1 = 0, temp2 = 0, temp4 = 0, temp5 = 0;
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			if (i == j)
				b[i * n + j] = 1.0f;
			else
				b[i * n + j] = 0.0f;
		}
	}
	// matrix inversion
	// code taken from: http://www.Planet-Source-Code.com/vb/scripts/ShowCode.asp?txtCodeId=13618&lngWId=3
	for (i = 0; i < n; i++) {
		temp = a[i * n + i];
		if (temp < 0)
			temp = temp * (-1);
		p = i;
		for (j = i + 1; j < n; j++) {
			if (a[j * n + i] < 0)
				tem = a[j * n + i] * (-1);
			else
				tem = a[j * n + i];
			if (temp < 0)
				temp = temp * (-1);
			if (tem > temp) {
				p = j;
				temp = a[j * n + i];
			}
		}
		//row exchange in both the matrix
		for (j = 0; j < n; j++) {
			temp1 = a[i * n + j];
			a[i * n + j] = a[p * n + j];
			a[p * n + j] = temp1;
			temp2 = b[i * n + j];
			b[i * n + j] = b[p * n + j];
			b[p * n + j] = temp2;
		}
		//dividing the row by a[i][i]
		temp4 = a[i * n + i];
		for (j = 0; j < n; j++) {
			a[i * n + j] = (float) a[i * n + j] / temp4;
			b[i * n + j] = (float) b[i * n + j] / temp4;
		}
		//making other elements 0 in order to make the matrix a[][] an indentity matrix and obtaining a inverse b[][] matrix
		for (q = 0; q < n; q++) {
			if (q == i)
				continue;
			temp5 = a[q * n + i];
			for (j = 0; j < n; j++) {
				a[q * n + j] = a[q * n + j] - (temp5 * a[i * n + j]);
				b[q * n + j] = b[q * n + j] - (temp5 * b[i * n + j]);
			}
		}
	}
}
