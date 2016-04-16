#include "motor.h"

struct Motor motor;

///*
// * calculates the necessary PWM value based on the current
// * battery voltage and the thrust required
// */
//uint8_t motor_ThrustToPWM(float thrust, float voltage) {
//
//}

/*
 * mixes the motor velocities according to the mixer table
 */
void motor_Mixer(float power, float roll, float pitch, float yaw) {
	uint8_t i;
	for (i = 0; i < config.motor.num; i++) {
		// motor value is a combination of the four channels and the mixer table
		motor.velocity[i] = power * config.motor.mixer[i][0]
				+ roll * config.motor.mixer[i][1]
				+ pitch * config.motor.mixer[i][2]
				+ yaw * config.motor.mixer[i][3];
		// restrain motor value when motor is used
		if (config.motor.mixer[i][0] != 0 || config.motor.mixer[i][1] != 0
				|| config.motor.mixer[i][2] != 0
				|| config.motor.mixer[i][3] != 0) {
			if (motor.velocity[i] < MOTOR_MINVELOCITY)
				motor.velocity[i] = MOTOR_MINVELOCITY;
			else if (motor.velocity[i] > MOTOR_MAXVELOCITY)
				motor.velocity[i] = MOTOR_MAXVELOCITY;
		}
	}
}

/*
 * disconnects the motors from the control loop and overwrites the speed
 * with manually given PWM values (effect lasts until motor_setAuto is called)
 */
void motor_setManual(uint8_t *pwmlist){
	uint8_t i;
	for(i=0;i<config.motor.num;i++){
		motor.manual.PWM[i] = pwmlist[i];
	}
	motor.manual.active = 1;
}

/*
 * reconnects the motors with the control loop
 */
void motor_setAuto(void){
	motor.manual.active = 0;
}

/*
 * sends the current velocities to the motors
 */
void motor_Update(void) {
	if (config.motor.output & MOTOR_USE_I2C) {
		externalI2C_TriggerUpdate();
		uint8_t i;
		for (i = 0; i < config.motor.num; i++) {
			if(motor.manual.active)
				motor.velocity[i] = motor.manual.PWM[i];
			if (motor.velocity[i] > 0) {
				// motor is running -> calculate current
				motor.current[i] = (uint16_t) motor.rawCurrent[i] * 100
						- motor.currentOffset[i];
				if (motor.current[i] < 0)
					motor.current[i] = 0;
			} else {
				// motor stopped -> update current offset
				motor.currentOffset[i] = (motor.currentOffset[i] * 15
						+ motor.rawCurrent[i] * 100) >> 4;
				motor.current[i] = 0;
			}
		}
	}
	if (config.motor.output & MOTOR_USE_PPM) {
		uint8_t i;
		for (i = 0; i < config.motor.num; i++) {
			servo_SetPosition(i,
					motor.velocity[i] * MOTOR_PPMSCALE + MOTOR_PPMOFFSET);
		}
	}
// TODO CAN
	// check for errors in communication (only available when using CAN or I2C)
	if (config.motor.output & (MOTOR_USE_CAN | MOTOR_USE_I2C)) {
		uint8_t i, logErrors = 0;
		if (time_TimerElapsed(&motor.errorMessageTimer)) {
			time_SetTimer(&motor.errorMessageTimer, 500);
			logErrors = 1;
		}
		char errorMessage[] = "ERROR: no communication with motor XX";
		uint8_t genericErrorBuf = 0;
		for (i = 0; i < config.motor.num; i++) {
			if (motor.error[i] != 0) {
				genericErrorBuf = 1;
				hott.speak = SPEAK_ERR_MOTOR;
				if (logErrors) {
					if (i < 10) {
						errorMessage[35] = (i + 1) + '0';
						errorMessage[36] = 0;
					} else {
						errorMessage[35] = (i + 1) / 10 + '0';
						errorMessage[36] = (i + 1) % 10 + '0';
					}
					log_LogFileEntry(errorMessage);
				}
			}
		}
		motor.genericError = genericErrorBuf;
	}
}
