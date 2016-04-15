#include "copter.h"

uint32_t printTimer, printTiltTimer, batteryTimer, motorTimer, logFlushTimer,
		checkCardTimer, flightLogTimer;
uint32_t Timer10ms;

uint8_t bootComplete = 0;

void copter_MainProgram(void) {
	/******************************************************************
	 * begin software initializations
	 *****************************************************************/
	led_Cmd(2, LED_ON);

	// load configuration values
	eeprom_LoadConfig();
	if (eeprom.configLoaded == RESET) {
		// failure while loading config
		// don't proceed without config as this might be dangerous
		log_LogFileEntry("SYSTEM START ABORTED DUE TO CRITICAL ERROR");
		log_FlushBufferedEntries();
		log_SyncFileSystem();
		// stop system boot here
		while (1)
			asm volatile("NOP");
	}
	// check plausibility of certain config values
	copter_ConstrainConfig();

	// load sensor profile data
	pressure_LoadTempProfile();

	// initialize filters
	kalman_Init();
	kalman_Reset();

	imuPosition_Init();
	imuPosition_Reset();
	// initialize controllers
	flightState_Init();
	controller_Init();

	/*
	 * check battery
	 */
	uint8_t i;
	for (i = 0; i < 100; i++) {
		battery_Update();
		time_Waitms(1);
	}
	battery_CalcCellNumber();
	// buzzer indicates number of detected battery cells
	buzzer_Signal(battery.cells);
	char logentry[] = "detected X battery cells";
	logentry[9] = battery.cells + '0';
	log_LogFileEntry(logentry);

	time_SetTimer(&printTimer, 0);
	time_SetTimer(&printTiltTimer, 0);
	time_SetTimer(&batteryTimer, 0);
	time_SetTimer(&motorTimer, 2000);
	time_SetTimer(&logFlushTimer, 2000);
	time_SetTimer(&checkCardTimer, 10);
	time_SetTimer(&Timer10ms, 5);

	led_Cmd(2, LED_OFF);
	/******************************************************************
	 * end software initializations
	 *****************************************************************/
	/*
	 * system booted completely
	 */

	led_Cmd(3, LED_ON);

	// set timer to trigger GPS initialization
	uint32_t GPSInitTimer;
	time_SetTimer(&GPSInitTimer, 3000);

	bootComplete = 1;

	while (1) {
#ifdef SIMULATION
		Simulation::Instance()->Handler();
#endif
		copter_UnimportantTasks();
		if (time_TimerElapsed(&GPSInitTimer)) {
			log_LogFileEntry("initializing GPS...");
			gps_Init();
			log_LogFileEntry("GPS init sequence sent");
			GPSInitTimer = UINT32_MAX;
		}
	}
}

/*
 * handles all time-critical tasks, such as sensor and controller updates.
 * This function should be called in an interrupt at a regular interval (e.g. 10 times/ms).
 * The priority of this interrupt should be as low as possible as this
 * function can take quite some time when multiple tasks are scheduled
 * at one iteration
 * !Function called from HAL interrupt in time.c!
 */
void copter_ImportantTasks(void) {
	if (bootComplete) {
		/******************************************************
		 * section 1: retrieve sensor data
		 *****************************************************/
		// check for new imu sensor data
		if (i2c_NewSensorData()) {
			// update all sensor, kalman and imu data
			imu_Update();
			if (logfile.kalmanLogRunning)
				log_KalmanLogAddEntry();
			imuPosition_Update(SENSOR_ACCELEROMETER);
			if (deviation.measurementRunning
					&& deviation.sensor < DEVIATION_BARO) {
				// deviation measurement of one of the acc/mag or gyro sensor is running
				deviation_NewData();
			}
		}
		// check for new pressure data (using external ADC)
		if (externalADC_Ready()) {
			externalADC_ReadData();
			pressure_Update();
			imuPosition_Update(SENSOR_BAROMETER);
			if (deviation.measurementRunning
					&& deviation.sensor == DEVIATION_BARO) {
				// deviation measurement of the baro sensor is running
				deviation_NewData();
			}
		}
		// check for new RC data
		if (receiver.newData) {
			receiver_DecodeSwitches();
			if (!stepresponse.active)
				flightState_Update();
			receiver_UpdateServos();
			receiver.newData = RESET;
		}
		// check for new gps data
		if (gps_FrameComplete()) {
			gps_EvaluateFrame();
			if (gps.FixValid)
				imuPosition_Update(SENSOR_GPS);
		}

		/******************************************************
		 * section 2: controller update
		 *****************************************************/
		controller_Update();

		/******************************************************
		 * section 3: motor update
		 *****************************************************/
		if (time_TimerElapsed(&motorTimer)) {
			time_SetTimer(&motorTimer, 2);
			if (!stepresponse.active)
				flightState_SetMotors();
			else
				StepResponse_SetMotors();
			motor_Update();
		}
	}
}
/*
 * handles all non time-critical tasks, such as writing data to the SD card,
 * update HoTT-data and the handling of the PC communication. It should be called
 * whenever nothing important has to be done (e.g. in the main-loop)
 */
void copter_UnimportantTasks(void) {
	/******************************************************
	 * section 4: check for timeouts
	 *****************************************************/
	// buffer timestamps (might change in interrupt AFTER time_GetMillis
	// -> would lead to negative time since last sensor update)
	uint32_t recTime = receiver.timestamp;
	uint32_t presTime = pressure.timestamp;
	uint32_t gpsTime = gps.timestamp;
	uint32_t externalSensorTime = externalSensor.timestamp;
	uint32_t accUpdateTime = accelerometer.timestampUpdate;
	uint32_t accChangeTime = accelerometer.timestampChange;
	uint32_t magUpdateTime = magnetometer.timestampUpdate;
	uint32_t magChangeTime = magnetometer.timestampChange;
	uint32_t gyroUpdateTime = gyro.timestampUpdate;
	uint32_t gyroChangeTime = gyro.timestampChange;
	uint32_t currentTime = time_GetMillis();
	// IMU sensor data
	// accelerometer
	if (currentTime - accUpdateTime >= 10 && accelerometer.valid == SET) {
		accelerometer.valid = RESET;
		hott.speak = SPEAK_ERR_SENSOR;
		log_LogFileEntry("TIMEOUT: no accelerometer data");
	}
	if (currentTime - accChangeTime >= 30 && accelerometer.valid == SET) {
		accelerometer.valid = RESET;
		hott.speak = SPEAK_ERR_SENSOR;
		log_LogFileEntry("ERROR: accelerometer data frozen");
	}
	// magnetometer
	if (currentTime - magUpdateTime >= 10 && magnetometer.valid == SET) {
		magnetometer.valid = RESET;
		hott.speak = SPEAK_ERR_SENSOR;
		log_LogFileEntry("TIMEOUT: no magnetometer data");
	}
	if (currentTime - magChangeTime >= 30 && magnetometer.valid == SET) {
		magnetometer.valid = RESET;
		hott.speak = SPEAK_ERR_SENSOR;
		log_LogFileEntry("ERROR: magnetometer data frozen");
	}
	// gyro
	if (currentTime - gyroUpdateTime >= 10 && gyro.valid == SET) {
		gyro.valid = RESET;
		hott.speak = SPEAK_ERR_SENSOR;
		log_LogFileEntry("TIMEOUT: no gyro data");
	}
	if (currentTime - gyroChangeTime >= 30 && gyro.valid == SET) {
		gyro.valid = RESET;
		hott.speak = SPEAK_ERR_SENSOR;
		log_LogFileEntry("ERROR: gyro data frozen");
	}
	// receiver timeout
	if (currentTime - recTime >= 30 && receiver.valid == SET) {
		receiver.valid = RESET;
		hott.speak = SPEAK_ERR_RECEIVER;
		log_LogFileEntry("TIMEOUT: no RC data");
		flightState_Update();
		led_Cmd(0, LED_ON);
	}
	// external ADC timeout
	if (currentTime - presTime >= 100 && pressure.valid == SET) {
		pressure.valid = RESET;
		hott.speak = SPEAK_ERR_SENSOR;
		log_LogFileEntry("TIMEOUT: no barometer data");
//			// attempt to restart the ADC
//			// caution: this blocks further execution for at least 200us!
//			externalADC_Init();
	}
	// gps timeout
	if (currentTime - gpsTime >= 150 && gps.available == SET) {
		gps.available = RESET;
		gps.FixValid = RESET;
		hott.speak = SPEAK_ERR_GPS;
		log_LogFileEntry("TIMEOUT: no GPS data");
		led_Cmd(1, LED_ON);
		led_Cmd(2, LED_ON);
	}
	// external sensor timeout
	if (currentTime - externalSensorTime >= 150
			&& externalSensor.valid == SET) {
		externalSensor.valid = RESET;
		log_LogFileEntry("TIMEOUT: no external tilt angle");
	}

	/******************************************************
	 * section 5: secondary (not so important) tasks
	 *****************************************************/
	// check battery
	if (time_TimerElapsed(&batteryTimer)) {
		time_SetTimer(&batteryTimer, 20);
		battery_Update();
		if (battery_GetStatus() == ERROR) {
			buzzer_Signal(2);
			hott.speak = SPEAK_UNDERVOLTAGE;
		}
	}
	// update HoTT data
	if (hott.updateRequest) {
		hott_Update();
	}
	// react to received messages
	uint8_t messageID = stdcomm_MessageReady();
	if (messageID > 0) {
		stdComm_MessageHandler(messageID);
	}

	// update step response state-machine
	if (stepresponse.active) {
		StepResponse_Update();
	}

	// all stuff that has to happen roughly each 10ms
	if (time_TimerElapsed(&Timer10ms)) {
		time_SetTimer(&Timer10ms, 10);
		distance_Update();
		flightState_UpdateHowerpower();
	}
	/******************************************************
	 * section 6: data logging
	 *****************************************************/
	// check for card availability
	if (time_TimerElapsed(&checkCardTimer)) {
		time_SetTimer(&checkCardTimer, 10);
		log_CheckIfCardAvailable();
	}
	// logfile handlers
	log_FlushBufferedEntries();
	if (time_TimerElapsed(&logFlushTimer)) {
		time_SetTimer(&logFlushTimer, 2000);
		log_SyncFileSystem();
		// send pressure sensor data via uart
//		char buffer[20];
//		itoASCII(pressure.ADCValue, buffer);
//		stdComm_puts(buffer);
//		stdComm_putc(';');
//		itoASCII(pressure.ADCTempCompensated, buffer);
//		stdComm_puts(buffer);
//		stdComm_putc(';');
//		ftoa(pressure.temp, buffer);
//		stdComm_puts(buffer);
//		stdComm_putc('\n');
	}
	// flightLog handlers
	if (logfile.flightLogStartRequest) {
		log_StartFlightLog();
		time_SetTimer(&flightLogTimer, 0);
	}
	if (logfile.flightLogStopRequest)
		log_StopFlightLog();
	if (logfile.flightLogRunning && time_TimerElapsed(&flightLogTimer)) {
		if (time_Get100us() - flightLogTimer > 100 * config.flightLogInterval) {
			// by this time there already exist 10 pending flightLog lines -> flightLogInterval is too short
			// reset timer to current time
			time_SetTimer(&flightLogTimer, config.flightLogInterval);
			log_LogFileEntry(
					"WARNING: flightLog interval is too short. Data has been dropped");
		} else {
			// set next flightLog timing
			flightLogTimer += config.flightLogInterval * 10;
		}
		log_FlightLogAddEntry();
	}
	/*
	 * kalmanLog handler
	 */
	log_KalmanLogFlushEntries();
	if (logfile.kalmanLogStartRequest) {
		log_StartKalmanLog();
	}
	if (logfile.kalmanLogStopRequest)
		log_StopKalmanLog();
	/******************************************************
	 * section 7: sensor calibration
	 *****************************************************/
	if (flightState.motorOff) {
		// calibration only possible with motors switched off
		if (receiver_GetChannel(CHANNEL_POWER) > 75
				&& receiver_GetChannel(CHANNEL_YAW) < -75) {
			// calibrate gyro
			gyro_Calibrate();
		} else if (receiver_GetChannel(CHANNEL_POWER) > 75
				&& receiver_GetChannel(CHANNEL_YAW) > 75) {
			// calibrate accelerometer
			acc_Calibrate();
		} else if (receiver_GetChannel(CHANNEL_PITCH) > 75
				&& receiver_GetChannel(CHANNEL_ROLL) < -75) {
			// calibrate magnetometer
			if (magnetometer.calibration.state == STOP)
				mag_Calibrate(START);
		} else if (receiver_GetChannel(CHANNEL_PITCH) > 75
				&& receiver_GetChannel(CHANNEL_ROLL) > 75) {
			// evaluate magnetometer calibration
			if (magnetometer.calibration.state == START)
				mag_Calibrate(STOP);
		}
	}
}

/*
 * checks and adjusts if necessary certain config values
 */
void copter_ConstrainConfig(void) {
	if (config.motor.num > MOTOR_MAXNUM)
		// too many motors in config
		config.motor.num = MOTOR_MAXNUM;
	if (config.motor.output & ~(MOTOR_USE_CAN | MOTOR_USE_I2C | MOTOR_USE_PPM))
		// unknown option specified (probably EEPROM cleared)
		// -> deactivate all outputs to be safe
		config.motor.output = 0;
	if (config.flightLogInterval == 0)
		// no infinitely fast flightlog please
		config.flightLogInterval = 1;
	// low pass filter setting must be between 0 and 1
	if (config.alphaAcc < 0 || config.alphaAcc > 1)
		config.alphaAcc = 0;
	if (config.alphaGyro < 0 || config.alphaGyro > 1)
		config.alphaGyro = 0;
	if (config.alphaMag < 0 || config.alphaMag > 1)
		config.alphaMag = 0;
	// constrain howerpower to maximum motor velocity
	if (config.howerPower < MOTOR_MINVELOCITY)
		config.howerPower = MOTOR_MINVELOCITY;
	if (config.howerPower > MOTOR_MAXVELOCITY)
		config.howerPower = MOTOR_MAXVELOCITY;
}

