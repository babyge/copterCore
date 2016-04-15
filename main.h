/*
 * Overview peripheral usages:
 *
 * USART:
 * hardware initialization of all U(S)ARTS is done in HAL/usart.h
 * 1 receiver.h 				RC-receiver (only RX)
 * 2 stdComm.h					standard communication port
 * 3 hott.h						HoTT-telemetry protocol
 * 4 missing implementation		external sensor data (e.g. absolute tilt data)
 * 5 unused
 * 6 gps.h						GPS module connector
 *
 * I2C:
 * 1 internalI2C.h				internal i2c bus for imu sensors + eeprom
 * 2 externalI2C.h				external i2c bus for BlCtrl
 * 3 unused
 *
 * Timer:
 * 1 servo.h					PWM generation for 4 servos
 * 2 buzzer.h					timing for buzzer tones
 * 3 servo.h					PWM generation for 4 servos
 * 4 servo.h					PWM generation for 4 servos
 * 5 usart.h					timer interrupt 1/ms for HoTT timing
 * 6 internalI2C.h				timing for constant sensor reading
 * 7 time.h						system time generation
 * 8 copter.h					generation of interrupts for copter_ImportantTasks
 * 9 externalI2C.h				timeout detection for external I2C bus
 * 10 unused
 * 11 unused
 * 12 unused
 * 13 unused
 * 14 unused
 *
 * DMA1:
 * Stream:
 * 0 cpal_i2c_hal_stm32f4xx.h	internal i2c RX stream
 * 1 unused
 * 2 unused
 * 3 unused
 * 4 unused
 * 5 unused
 * 6 cpal_i2c_hal_stm32f4xx.h	internal i2c TX stream
 * 7 unused
 *
 * DMA2:
 * Stream:
 * 0 battery.h					ADC1 stream, containing battery values
 * 1 unused
 * 2 unused
 * 3 unused
 * 4 unused
 * 5 unused
 * 6 unused
 * 7 unused
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stdcomm.h"
#include "Peripheral/cpal.h"
#include "Peripheral/cpal_i2c.h"
#include "hal.h"
#include "time.h"
#include "arm_math.h"
#include "imu.h"
#include "battery.h"
#include "receiver.h"
#include "copter.h"
#include "stepresponse.h"
#include <stdlib.h>

//#define DEBUG_UART

void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

#endif /* MAIN_H_ */
