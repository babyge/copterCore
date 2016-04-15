/*
 * This file contains function to calculate the single motor velocities
 * based on the controller outputs (power, roll, yaw, pitch
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "hal.h"
#include "hott.h"

// number of maximal supported motors
#define MOTOR_MAXNUM		12

// minimal and maximal motor value during flight
#define MOTOR_MINVELOCITY	10
#define MOTOR_MAXVELOCITY	255

// bitmask values for motor interface
#define MOTOR_USE_I2C		1
#define MOTOR_USE_PPM		2
#define MOTOR_USE_CAN		4

// configuration of the ppm outputs (if activated)
// servo position at zero motor velocity
#define MOTOR_PPMOFFSET		-1000
// motor output scaling (should result in ~2000 dynamic range)
#define MOTOR_PPMSCALE		8

struct {
	// motor velocities
	int16_t velocity[MOTOR_MAXNUM];
	// motor currents in 100mA
	uint8_t rawCurrent[MOTOR_MAXNUM];
	// in mA
	int16_t current[MOTOR_MAXNUM];
	uint16_t currentOffset[MOTOR_MAXNUM];
	// indicates a motor specific error during communication
	uint8_t error[MOTOR_MAXNUM];
	// indicates a general error during communication
	uint8_t genericError;
	uint32_t errorMessageTimer;
	struct {
		uint8_t active;
		uint8_t PWM[MOTOR_MAXNUM];
	} manual;
} motor;

///*
// * calculates the necessary PWM value based on the current
// * battery voltage and the thrust required
// */
//uint8_t motor_ThrustToPWM(float thrust, float voltage);

/*
 * mixes the motor velocities according to the mixer table
 */
void motor_Mixer(float power, float roll, float pitch, float yaw);

/*
 * disconnects the motors from the control loop and overwrites the speed
 * with manually given PWM values (effect lasts until motor_setAuto is called)
 */
void motor_setManual(uint8_t *pwmlist);

/*
 * reconnects the motors with the control loop
 */
void motor_setAuto(void);

/*
 * sends the current velocities to the motors
 */
void motor_Update(void);
#endif /* MOTOR_H_ */
