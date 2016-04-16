/*
 * 9-state extended kalman filter
 * state vector:
 * Xa, Ya, Za acceleration
 * Xm, Ym, Zm magnetic field
 * Xg, Yg, Zg angular velocity
 * Xb, Yb, Zb gyro bias
 *
 *	measurement vector:
 *	Xa, Ya, Za accelerometer
 *	Xm, Ym, Zm magnetometer
 *	Xg, Yg, Zg gyro
 *
 * transistion function f:
 * Xa = Xa + Za * Yg + Ya * Zg
 * Ya = Ya + Za * Xg - Xa * Zg
 * Za = Za - Xa * Yg - Ya * Xg
 * Xm = Xm + Zm * Yg + Ym * Zg
 * Ym = Ym + Zm * Xg - Xm * Zg
 * Zm = Zm - Xm * Yg - Ym * Xg
 * Xg = Xg
 * Yg = Yg
 * Zg = Zg
 * Xb = Xb
 * Yb = Yb
 * Zb = Zb
 *
 * transition matrix (Jacobian matrix of f):
 * 	1	Zg	Yg	0	0	0	0	Za	Ya	0	0	0
 * 	-Zg	1	Xg	0	0	0	Za	0	-Xa	0	0	0
 * 	-Yg	-Xg	1	0	0	0	-Ya	-Xa	0	0	0	0
 * 	0	0	0	1	Zg	Yg	0	Zm	Ym	0	0	0
 * 	0	0	0	-Zg	1	Xg	Zm	0	-Xm	0	0	0
 * 	0	0	0	-Yg	-Xg	1	-Ym	-Xm	0	0	0	0
 * 	0	0	0	0	0	0	1	0	0	0	0	0
 * 	0	0	0	0	0	0	0	1	0	0	0	0
 * 	0	0	0	0	0	0	0	0	1	0	0	0
 * 	0	0	0	0	0	0	0	0	0	1	0	0
 * 	0	0	0	0	0	0	0	0	0	0	1	0
 * 	0	0	0	0	0	0	0	0	0	0	0	1
 *
 * 	measurement matrix H:
 * 	1	0	0	0	0	0	0	0	0	0	0	0
 * 	0	1	0	0	0	0	0	0	0	0	0	0
 * 	0	0	1	0	0	0	0	0	0	0	0	0
 * 	0	0	0	1	0	0	0	0	0	0	0	0
 * 	0	0	0	0	1	0	0	0	0	0	0	0
 * 	0	0	0	0	0	1	0	0	0	0	0	0
 * 	0	0	0	0	0	0	1	0	0	1	0	0
 * 	0	0	0	0	0	0	0	1	0	0	1	0
 * 	0	0	0	0	0	0	0	0	1	0	0	1
 *
 * 	each iteration takes approximately 265us to calculate
 * 	(at 168Mhz, FPU = softfp)
 *
 * 	there is still some optimization potential, especially
 * 	in the matrix inversion routine which consumes over 50%
 * 	of the time
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "accelerometer.h"
#include "gyro.h"
#include "magnetometer.h"


//#define KALMAN_UPDATE_INTERVAL 1.0f/I2C_CYCLE_FREQUENCY
#define KALMAN_STATES			12
#define KALMAN_MEASUREMENTS		9

/*
 * uncomment to force matrix P to be symmetrical
 * (done by averaging with PT and applying minimal values
 * to the diagonal elements)
 * this feature might be necessary to avoid distortion of
 * the P and consecutively K matrix due to floating-point
 * issues.
 * activate, if the filter gets unstable after some time
 */
//#define KALMAN_FORCE_SYMMETRY_P

/*
 * kalman noises. The filter can be tuned by changing this values.
 * brief introduction:
 * each value represents the expected noise on the state/sensor.
 * some basic tips for tuning the filter:
 * - ACC_NOISE should be pretty big, especially when the are vibrations
 * in the system
 * - STATE_BIAS_NOISE should be very small since the gyro only drifts
 * really slow in comparison to the sensor values
 *
 * !! THESE VALUES HAVE BEEN MOVED TO THE CONFIG STRUCT IN EEPROM.H !!
 */
//#define KALMAN_ACC_NOISE			50000.0f
//#define KALMAN_GYRO_NOISE			3.0f
//#define KALMAN_MAG_NOISE			30000.0f
//#define KALMAN_STATE_ACC_NOISE		2.0f
//#define KALMAN_STATE_GYRO_NOISE		2.0f
//#define KALMAN_STATE_MAG_NOISE		2.0f
//#define KALMAN_STATE_BIAS_NOISE		0.002f


/*
 * 	the following definitions simplify some changes in the filter.
 * 	it is NOT sufficient to change this definitions, there are still
 * 	some parts hardcoded (i.e. partial matrix multiplications)
 */
// x vector definitions
#define KALMAN_X_ACC_X			0
#define KALMAN_X_ACC_Y			1
#define KALMAN_X_ACC_Z			2
#define KALMAN_X_MAG_X			3
#define KALMAN_X_MAG_Y			4
#define KALMAN_X_MAG_Z			5
#define KALMAN_X_GYRO_X			6
#define KALMAN_X_GYRO_Y			7
#define KALMAN_X_GYRO_Z			8
#define KALMAN_X_BIAS_X			9
#define KALMAN_X_BIAS_Y			10
#define KALMAN_X_BIAS_Z			11
// z vector definitions
#define KALMAN_Z_ACC_X			0
#define KALMAN_Z_ACC_Y			1
#define KALMAN_Z_ACC_Z			2
#define KALMAN_Z_MAG_X			3
#define KALMAN_Z_MAG_Y			4
#define KALMAN_Z_MAG_Z			5
#define KALMAN_Z_GYRO_X			6
#define KALMAN_Z_GYRO_Y			7
#define KALMAN_Z_GYRO_Z			8

struct Kalman{
	float x[KALMAN_STATES];
	float *q[KALMAN_STATES];
	float *z[KALMAN_MEASUREMENTS];
	float *r[KALMAN_MEASUREMENTS];
	float pValues[KALMAN_STATES][KALMAN_STATES];
	float kValues[KALMAN_STATES][KALMAN_MEASUREMENTS];
	uint32_t timestamp;
	float timeDiff;
};

extern struct Kalman kalman;

/*
 * initializes the kalman matrices
 */
void kalman_Init(void);
/*
 * calculates the next iteration of the kalman filter
 */
void kalman_Update(void);
/*
 * overwrites the kalman state with the current measured values
 */
void kalman_Reset();

void kalman_Print(void);

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(float* vector, float* delta);

/*
 * internal steps of the kalman filter
 */
void kalman_StatePropagation(void);
void kalman_CovariancePropagation(void);
void kalman_CalculateGain(void);
void kalman_StateUpdate(void);
void kalman_CovarianceUpdate(void);

/*
 * matrix inversion using Gauss Jordan Elimination Method
 * @param a array containing the input matrix
 * @param b array containing the output matrix
 * @param n size of the matrices
 */
void kalman_InverseMatrix(float *a, float *b, uint8_t n);

#endif /* KALMAN_H_ */
