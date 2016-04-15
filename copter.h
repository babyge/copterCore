/*
 * contains the main program. The main program is started after all
 * hardware has been initialized properly
 */
#ifndef COPTER_H_
#define COPTER_H_

#include "hal.h"
#include "controller.h"
#include "imuPosition.h"
#include "battery.h"
#include "gps.h"
#include "pressure.h"
#include "externalSensors.h"
#include "accelerometer.h"
#include "magnetometer.h"
#include "gyro.h"
#include "hott.h"
#include "sensorDeviation.h"

void copter_MainProgram(void);

/*
 * handles all time-critical tasks, such as sensor and controller updates.
 * This function should be called in an interrupt at a regular interval (e.g. 10 times/ms).
 * The priority of this interrupt should be as low as possible as this
 * function can take quite some time when multiple tasks are scheduled
 * at one iteration
 */
void copter_ImportantTasks(void);
/*
 * handles all non time-critical tasks, such as writing data to the SD card,
 * update HoTT-data and the handling of the PC communication. It should be called
 * whenever nothing important has to be done (e.g. in the main-loop)
 */
void copter_UnimportantTasks(void);

/*
 * checks and adjusts if necessary certain config values
 */
void copter_ConstrainConfig(void);

#endif /* COPTER_H_ */
