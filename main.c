/*
 * main.c
 *
 *  Created on: Oct 30, 2013
 *      Author: jan
 */

#include "main.h"

void _exit(int a) {
	while (1) {
	};
}

int main(void) {
	/******************************************************************
	 * begin peripheral initializations
	 *****************************************************************/
	SysTick_Config(168000);
	SystemCoreClockUpdate();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	led_Init();
	led_Cmd(0, LED_ON);
	time_Init();
	usart_Init();
	stdComm_puts("Boot...\n");

	// give external hardware enough time to boot
	time_Waitms(300);

	buzzer_Init();
	servo_Init();
	adc_Init();
	externalADC_Init();
	i2c_Init();

	externalI2C_Init();
	time_Waitms(10);

	led_Cmd(0, LED_OFF);
	/******************************************************************
	 * end peripheral initializations
	 *****************************************************************/

	/******************************************************************
	 * begin hardware initializations
	 *****************************************************************/
	led_Cmd(1, LED_ON);

	log_Init();
	i2c_InitSensors();
	//gps_Init();

	// buzzer indicates completed hardware initialisation
	buzzer_Signal(1);
	led_Cmd(1, LED_OFF);
	/******************************************************************
	 * end hardware initializations
	 *****************************************************************/

	// start main program
	copter_MainProgram();

	// should never reach this point
	while (1) {
	}

}
