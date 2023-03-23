/*
 * cBot.c
 *
 * Copyright (C) 2021  Stefan Hoermann (mail@stefan-hoermann.de)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */


#include "cBot.h"

#include "button.h"
#include "buzzer.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


buttons_t buttons;
button_t *buttonRight, *buttonDown, *buttonLeft, *buttonUp;

buzzer_t buzzerObj;
buzzer_t *buzzer = &buzzerObj;

ws2812b_t rgbLedObj;
ws2812b_t *rgbLeds = &rgbLedObj;

u8g2_t displayObj;
u8g2_t *display = &displayObj;


// TODO:
// - serial interface


// ----- Buttons --------------------------------------------------------------
int isPressed(buttonId b) {
	if ( b == BUTTON_UP ) return buttonUp->isPressed;
	if ( b == BUTTON_DOWN ) return buttonDown->isPressed;
	if ( b == BUTTON_RIGHT ) return buttonRight->isPressed;
	if ( b == BUTTON_LEFT ) return buttonLeft->isPressed;
	else return 0;
}
// ----- Buttons --------------------------------------------------------------


// ----- Sound ----------------------------------------------------------------
void beep(uint16_t frequency, uint16_t durationMs) {
	if ( isPlaying() ) stopPlaying();
	buzzer_setFrequency(buzzer->htim, buzzer->channel, frequency);
	HAL_Delay(durationMs);
	buzzer_setFrequency(buzzer->htim, buzzer->channel, 0);
}

void playNote(uint16_t frequency, uint16_t durationMs) {
	buzzer_playNote(buzzer, frequency, durationMs);
}

void playPause(uint16_t durationMs) {
	buzzer_playNote(buzzer, 0, durationMs);
}

int isPlaying() {
	return buzzer_isPlaying(buzzer);
}

void stopPlaying() {
	buzzer_stopPlaying(buzzer);
}
// ----- Sound ----------------------------------------------------------------


// ----- SSD1306 I2C OLED Display ---------------------------------------------
const uint8_t SSD1306_I2C_ADDRESS = 0x3c;
uint8_t buffer[32], dataSize = 0;

uint8_t u8x8_gpio_and_delay_STM32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	switch(msg)
	{
	case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
		break;							// can be used to setup pins
	case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
		HAL_Delay(arg_int);
		break;
	default:
		u8x8_SetGPIOResult(u8x8, 1);			// default return value
		break;
	}
	return 1;
}

uint8_t u8x8_byte_STM32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	switch(msg)
	{
	case U8X8_MSG_BYTE_SEND:
		for ( int i = 0; i < arg_int; i++ ) {
			buffer[dataSize] = ((uint8_t *)arg_ptr)[i];
			dataSize++;
		}
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		dataSize = 0;
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDRESS << 1, buffer, dataSize, 0xffff);
		break;
	default:
		return 0;
	}
	return 1;
}
// ----- SSD1306 I2C OLED Display ---------------------------------------------


// ----- Servos ---------------------------------------------------------------
void setServo(uint8_t servoId, uint16_t position) {

	// limit position to 1000
	if ( position > 1000 ) position = 1000;

	// compute compare value
	uint16_t compareValue = 1000 + position;

	// set compare value
	switch (servoId) {
	case 1:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compareValue);
		break;
	case 2:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compareValue);
		break;
	case 3:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, compareValue);
		break;
	}

}

void servo_init() {
	setServo(1, SERVO_MAX / 2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	setServo(2, SERVO_MAX / 2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	setServo(3, SERVO_MAX / 2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}
// ----- Servos ---------------------------------------------------------------


// ----- Range Sensors --------------------------------------------------------
const uint16_t rangeSensorCycleLength = 750; // -> 75ms
const uint16_t rangeInvalid = 8000;

rangeSensor_t rangeSensor[3];

void rangeSensor_init() {
	rangeSensor[0].triggerPort = rangeSensor0Trigger_GPIO_Port;
	rangeSensor[0].triggerPin = rangeSensor0Trigger_Pin;
	rangeSensor[0].echoPin = rangeSensor0Echo_Pin;
	rangeSensor[0].counter = 0;
	rangeSensor[0].rangeMM = rangeInvalid;
	rangeSensor[1].triggerPort = rangeSensor1Trigger_GPIO_Port;
	rangeSensor[1].triggerPin = rangeSensor1Trigger_Pin;
	rangeSensor[1].echoPin = rangeSensor1Echo_Pin;
	rangeSensor[1].counter = rangeSensorCycleLength / 3;
	rangeSensor[1].rangeMM = rangeInvalid;
	rangeSensor[2].triggerPort = rangeSensor2Trigger_GPIO_Port;
	rangeSensor[2].triggerPin = rangeSensor2Trigger_Pin;
	rangeSensor[2].echoPin = rangeSensor2Echo_Pin;
	rangeSensor[2].counter = rangeSensorCycleLength * 2 / 3;
	rangeSensor[2].rangeMM = rangeInvalid;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint16_t timeStamp = __HAL_TIM_GET_COUNTER(&htim1);


	for ( int sensorId = 0; sensorId < 3; sensorId++ ) {
		if ( GPIO_Pin == rangeSensor[sensorId].echoPin ) {
			if ( rangeSensor[sensorId].echoStartCoarse == 0 ) {
				rangeSensor[sensorId].echoStartCoarse = rangeSensor[sensorId].counter;
				rangeSensor[sensorId].echoStartExact = timeStamp;
			}
			else {
				rangeSensor[sensorId].echoStopCoarse = rangeSensor[sensorId].counter;
				rangeSensor[sensorId].echoStopExact = timeStamp;

				uint16_t echoDurationCoarse = rangeSensor[sensorId].echoStopCoarse - rangeSensor[sensorId].echoStartCoarse;
				uint16_t echoDurationExact = (rangeSensor[sensorId].echoStopExact + 20000 - rangeSensor[sensorId].echoStartExact) % 20000;
				uint16_t rangeMM = (uint32_t)echoDurationExact * 3432 / 10000 / 2;

				if ( (echoDurationCoarse > 180) || (rangeMM > 3000) ) {
					rangeSensor[sensorId].rangeMM = rangeInvalid;
				}
				else {
					rangeSensor[sensorId].rangeMM = rangeMM;
				}

				//				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			}

		}
	}

}

int getRangeMm(sensorId id) {
	if ( id == SENSOR_LEFT ) return rangeSensor[0].rangeMM;
	else if ( id == SENSOR_MIDDLE ) return rangeSensor[1].rangeMM;
	else if ( id == SENSOR_RIGHT ) return rangeSensor[2].rangeMM;
	else return 0;
}
// ----- Range Sensors --------------------------------------------------------


// ----- Light Sensors --------------------------------------------------------
#define LIGHT_SENSOR_MEAN_SIZE 500
uint16_t lightSensorAdcBuffer[2] = {0, 0};
uint32_t lightSensorMeanBuffer[2] = {0, 0};
uint16_t lightSensorMeanCounter = 0;
uint16_t lightSensorData[2];

int getLightValue(sensorId id) {
	if ( id == SENSOR_LEFT ) return lightSensorData[0];
	else if ( id == SENSOR_RIGHT ) return lightSensorData[1];
	else return 0;
}
// ----- Light Sensors --------------------------------------------------------


// ----- RGB LEDs -------------------------------------------------------------
uint8_t rgbLedUpdateFlag = 0, rgbLedUpdateWait = 0;

void clearLeds() {
	ws2812b_clear(rgbLeds);
}

uint32_t getColorRGB(uint8_t r, uint8_t g, uint8_t b) {
	return ws2812b_colorRGB(r, g, b);
}

uint32_t getColorHSV(uint16_t h, uint8_t s, uint8_t v) {
	return ws2812b_colorHSV(h, s, v);
}

void setLed(uint16_t id, uint32_t color) {
	ws2812b_setColor(rgbLeds, id, color);
}

void updateLeds() {
	//	ws2812b_update(rgbLeds);
	rgbLedUpdateFlag = 1;
}
// ----- RGB LEDs -------------------------------------------------------------


// ----- Motors ---------------------------------------------------------------
const uint32_t stepperIrqCyclesPerSecond = 10000;
const uint16_t stepperPattern[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};
const uint32_t stepperMaxInactivePeriods = 10000; //stepperIrqCyclesPerSecond;
const uint32_t stepperStepValue = 16777216;
const float StepperStepsPerRevolution = 2 * 2048.0f; // times two, because motor is driven in half steps
uint32_t isStepperOn = 1, stepperInactivePeriods = 0;
uint32_t motorIncrementL = 0, motorIncrementR = 0; // 120 max aus dem Stand
uint32_t motorValueL = 0, motorValueR = 0;

const float WHEEL_RADIUS = 0.0197; // unit: m; also known as: dist per rad
const float WHEEL_BASE = 0.0945; // unit: m
const float RPM_TO_RAD_PER_S = 2 * M_PI / 60;
float rpmMax = 22;
float rpmStart = 2;
uint8_t accellerationSteps = 10;
float accellerationStepDuration = 0.020;



motorStatus_t motorStatus;

void motorInit() {
	motorStatus.isLocked = 0;
	motorStatus.isRpm = 0;
	motorStatus.nextUpdate = 0;
	motorStatus.firstMotorRpm = NULL;
	motorStatus.lastMotorRpm = NULL;
}

void addMotorRpm(float rpmL, float rpmR, uint16_t ms) {

	// turn off RPM mode
	motorStatus.isRpm = 0;

	// create motor RPM object
	motorRpm_t *motorRpm = malloc(sizeof(motorRpm_t));
	motorRpm->rpmLeft = rpmL;
	motorRpm->rpmRight = rpmR;
	motorRpm->durationMs = ms;
	motorRpm->nextMotorRpm = NULL;

	// lock motor RPM queue
	motorStatus.isLocked = 1;

	// add motor RPM object
	if ( motorStatus.lastMotorRpm == NULL ) {
		motorStatus.firstMotorRpm = motorRpm;
		motorStatus.lastMotorRpm = motorRpm;
	}
	else {
		motorStatus.lastMotorRpm->nextMotorRpm = motorRpm;
		motorStatus.lastMotorRpm = motorRpm;
	}

	// unlock motor RPM queue
	motorStatus.isLocked = 0;
}

// computes the RPM values for both wheels according to the linear velocity and the angular rate
// linear velocity in m/s
// angular rate in rad/s
// see also: https://robotics.stackexchange.com/questions/18048/inverse-kinematics-for-differential-robot-knowing-linear-and-angular-velocities
void getRpmFromVelocity(float *left_rpm, float *right_rpm, float linear_velocity, float angular_rate) {
	*left_rpm  = (linear_velocity - 0.5f*angular_rate*WHEEL_BASE)/(RPM_TO_RAD_PER_S * WHEEL_RADIUS);
	*right_rpm = (linear_velocity + 0.5f*angular_rate*WHEEL_BASE)/(RPM_TO_RAD_PER_S * WHEEL_RADIUS);
}

void addRobotVel(float linear, float angular, uint16_t ms) {
	float rpmL, rpmR;
	getRpmFromVelocity(&rpmL, &rpmR, linear, angular);
	addMotorRpm(rpmL, rpmR, ms);
}

void setMotorIncrement(float rpmL, float rpmR) {
	motorIncrementL = (int32_t)(rpmL / 60 * StepperStepsPerRevolution / stepperIrqCyclesPerSecond * stepperStepValue + 0.5);
	motorIncrementR = (int32_t)(rpmR / 60 * StepperStepsPerRevolution / stepperIrqCyclesPerSecond * stepperStepValue + 0.5);
}

void setMotorRpm(float rpmL, float rpmR) {

	// empty motor RPM queue
	if ( motorStatus.firstMotorRpm != NULL ) {
		motorStatus.isLocked = 1;
		while ( motorStatus.firstMotorRpm != NULL ) {
			motorRpm_t *motorRpm = motorStatus.firstMotorRpm;
			motorStatus.firstMotorRpm = motorRpm->nextMotorRpm;
			free(motorRpm);
		}
		motorStatus.lastMotorRpm = NULL;
		motorStatus.isLocked = 0;
	}

	// turn on RPM mode
	motorStatus.isRpm = 1;

	// set RPM values
	setMotorIncrement(rpmL, rpmR);
}

void stopMotor() {
	setMotorRpm(0, 0);
}

void accelleratedMove(float rpmL, float rpmR, float duration) {

	float stepRpm, stepRpmL, stepRpmR, speedFactor;
	uint16_t stepDurationMs = (uint16_t)(accellerationStepDuration * 1000 + 0.5);
	motorRpm_t *firstMotorRpmObject = NULL;

	uint8_t accellarationStep = 0;
	while ( duration > 0 ) {
		stepRpm = rpmStart + (rpmMax - rpmStart) / accellerationSteps * accellarationStep;
		speedFactor = stepRpm / rpmMax;
		stepRpmL = rpmL * speedFactor;
		stepRpmR = rpmR * speedFactor;

		if ( (accellarationStep < accellerationSteps) && (duration > 3 * accellerationStepDuration * speedFactor) ) {

			// schedule motor RPM
			addMotorRpm(stepRpmL, stepRpmR, stepDurationMs);

			// create motor RPM object
			motorRpm_t *motorRpm = malloc(sizeof(motorRpm_t));
			motorRpm->rpmLeft = stepRpmL;
			motorRpm->rpmRight = stepRpmR;
			motorRpm->durationMs = stepDurationMs;
			motorRpm->nextMotorRpm = firstMotorRpmObject;
			firstMotorRpmObject = motorRpm;

			// update duration
			duration = duration - 2 * accellerationStepDuration * speedFactor;
		}
		else {

			// schedule motor RPM
			addMotorRpm(stepRpmL, stepRpmR, (uint16_t)(duration/speedFactor*1000 + 0.5));

			// update duration
			duration = 0;
		}

		accellarationStep++;
	}

	// slowdown
	while ( firstMotorRpmObject != NULL ) {

		// detach motor RPM object from list
		motorRpm_t *motorRpm = firstMotorRpmObject;
		firstMotorRpmObject = motorRpm->nextMotorRpm;

		// schedule motor RPM
		addMotorRpm(motorRpm->rpmLeft, motorRpm->rpmRight, motorRpm->durationMs);

		// dispose motor RPM object
		free(motorRpm);
	}

}

void driveStrait(float distance) {
	float velocity = 0.1, rpm;

	// compute velocity and rpm
	getRpmFromVelocity(&rpm, &rpm, velocity, 0);
	velocity = velocity * rpmMax / rpm;
	rpm = rpmMax;

	// adjust sign of velocity and rpm
	if ( distance < 0 ) {
		velocity = -velocity;
		rpm = -rpm;
	}

	// schedule motor RPM
	//	addMotorRpm(rpm, rpm, (uint16_t)(distance / velocity * 1000 + 0.5));
	accelleratedMove(rpm, rpm, distance / velocity);
}

void turn(float angle) {
	float angularRate = M_PI_4;
	float rpmL, rpmR;

	// compute angular rate and rpm
	getRpmFromVelocity(&rpmL, &rpmR, 0, angularRate);
	angularRate = angularRate * rpmMax / rpmR;
	rpmR = rpmMax;
	rpmL = -rpmMax;

	// adjust sign of angular rate and rpm
	if ( angle < 0 ) {
		angularRate = -angularRate;
		rpmR = -rpmR;
		rpmL = -rpmL;
	}

	// schedule motor RPM
	//	addMotorRpm(rpmL, rpmR, (uint16_t)(angle/180*M_PI / angularRate * 1000 + 0.5));
	accelleratedMove(rpmL, rpmR, angle/180*M_PI / angularRate);
}

void driveArc(float radius, float angle) {
	float distance, duration;
	float velocity = 0.1, angularRate;
	float rpmL, rpmR, f;

	// compute distance
	distance = fabs(2 * M_PI * radius * angle / 360);

	// compute duration
	duration = distance / velocity;

	// compute angular rate
	angularRate = fabs(angle/180*M_PI / duration);

	// compute motor RPM
	getRpmFromVelocity(&rpmL, &rpmR, velocity, angularRate);

	// scale motor RPM and duration
	f = rpmMax / rpmR;
	rpmR = rpmMax;
	rpmL = rpmL * f;
	duration = duration / f;

	// consider signs
	if ( radius < 0 ) {
		float t = rpmL;
		rpmL = rpmR;
		rpmR = t;
	}
	if ( angle < 0 ) {
		rpmL = -rpmL;
		rpmR = -rpmR;
	}

	// schedule motor RPM
	//	addMotorRpm(rpmL, rpmR, (uint16_t)(duration * 1000 + 0.5));
	accelleratedMove(rpmL, rpmR, duration);
}

int isMoving() {
	if ( (motorStatus.firstMotorRpm != NULL) || (motorIncrementL != 0) || (motorIncrementR != 0) ) return 1;
	else return 0;
}

void motorUpdate() {

	// get current time
	uint32_t currentTick = HAL_GetTick();

	if ( currentTick >= motorStatus.nextUpdate ) {

		// don't update, if motor RPM queue is locked
		if ( motorStatus.isLocked ) return;

		// get first tone handle
		motorRpm_t *motorRpm = motorStatus.firstMotorRpm;

		if ( motorRpm != NULL ) {

			// detach motor RPM from queue
			motorStatus.firstMotorRpm = motorRpm->nextMotorRpm;
			if ( motorStatus.firstMotorRpm == NULL ) {
				motorStatus.lastMotorRpm = NULL;
			}

			// set motor RPM
			setMotorIncrement(motorRpm->rpmLeft, motorRpm->rpmRight);

			// set next update time
			motorStatus.nextUpdate = currentTick + motorRpm->durationMs;

			// dispose motor RPM object
			free(motorRpm);
		}
		else {
			if ( !motorStatus.isRpm ) {
				// stop motor
				motorIncrementL = 0;
				motorIncrementR = 0;
			}
		}

	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if ( htim == &htim4 ) {

		// determine motor status: are the motors on or off?
		if ( (motorIncrementL != 0) || (motorIncrementR != 0) ) {
			isStepperOn = 1;
			stepperInactivePeriods = 0;
		}
		else if ( isStepperOn ) {
			stepperInactivePeriods++;
			if ( stepperInactivePeriods > stepperMaxInactivePeriods ) {
				isStepperOn = 0;
			}
		}

		// update motor value and bit patterns
		if ( isStepperOn ) {

			motorValueL += motorIncrementL;
			motorValueR -= motorIncrementR;

			GPIOA->ODR = (GPIOA->ODR & 0xFF00) |
					stepperPattern[(motorValueL / stepperStepValue) & 0x07] << 4|
					stepperPattern[(motorValueR / stepperStepValue) & 0x07];

		}
		else {
			GPIOA->ODR = GPIOA->ODR & 0xFF00;
		}

		// create trigger signal for range sensors
		for ( int sensorId = 0; sensorId < 3; sensorId++ ) {
			rangeSensor[sensorId].counter = (rangeSensor[sensorId].counter + 1) % rangeSensorCycleLength;
			switch ( rangeSensor[sensorId].counter ) {
			case 0:
				// handle unfinished range sensings
				if ( rangeSensor[sensorId].echoStopCoarse == 0 ) {
					rangeSensor[sensorId].rangeMM = rangeInvalid;
				}

				// reset range sensing
				rangeSensor[sensorId].echoStartCoarse = 0;
				rangeSensor[sensorId].echoStopCoarse = 0;

				// trigger range sensing
				HAL_GPIO_WritePin(rangeSensor[sensorId].triggerPort, rangeSensor[sensorId].triggerPin, GPIO_PIN_SET);
				break;
			case 1:
				HAL_GPIO_WritePin(rangeSensor[sensorId].triggerPort, rangeSensor[sensorId].triggerPin, GPIO_PIN_RESET);
				break;
			}
		}

		// acquire and compute light sensor data
		lightSensorMeanBuffer[0] += lightSensorAdcBuffer[0];
		lightSensorMeanBuffer[1] += lightSensorAdcBuffer[1];
		lightSensorMeanCounter++;
		if ( lightSensorMeanCounter >= LIGHT_SENSOR_MEAN_SIZE ) {
			lightSensorData[0] = (uint16_t)(lightSensorMeanBuffer[0] / LIGHT_SENSOR_MEAN_SIZE);
			lightSensorData[1] = (uint16_t)(lightSensorMeanBuffer[1] / LIGHT_SENSOR_MEAN_SIZE);
			lightSensorMeanBuffer[0] = 0;
			lightSensorMeanBuffer[1] = 0;
			lightSensorMeanCounter = 0;
			//			rgbLedUpdateWait++;
		}
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&lightSensorAdcBuffer, 2);

		// update RGB LEDs
		if ( rgbLedUpdateWait == 0 ) {
			if ( rgbLedUpdateFlag == 1 ) {
				ws2812b_update(rgbLeds);
			}
		}
		else if ( rgbLedUpdateWait > 0 ) {
			rgbLedUpdateWait--;
		}

		// Todo: heart beat -> yes, but with disable option!
		//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
}

void motor_setSpeed(float rpmL, float rpmR) {
	int32_t *mil = (int32_t*)&motorIncrementL, *mir = (int32_t*)&motorIncrementR;
	*mil = (int32_t)(rpmL / 60 * StepperStepsPerRevolution / stepperIrqCyclesPerSecond * stepperStepValue + 0.5);
	*mir = (int32_t)(rpmR / 60 * StepperStepsPerRevolution / stepperIrqCyclesPerSecond * stepperStepValue + 0.5);
}

void motor_stop() {
	motorIncrementL = 0;
	motorIncrementR = 0;
}

void motor_acceleratedForward(float rpmMin, float rpmMax, float rpmStep, int stepDurationMs, int fullSpeedMs) {
	int stepCount = (int)((rpmMax - rpmMin) / rpmStep);

	// acceleration
	for ( int i = 0; i < stepCount; i++ ) {
		float rpm = rpmMin + rpmStep * i;
		motor_setSpeed(rpm, rpm);
		HAL_Delay(stepDurationMs);
	}

	// full speed
	motor_setSpeed(rpmMax, rpmMax);
	HAL_Delay(fullSpeedMs);

	// braking
	for ( int i = stepCount - 1; i >= 0; i-- ) {
		float rpm = rpmMin + rpmStep * i;
		motor_setSpeed(rpm, rpm);
		HAL_Delay(stepDurationMs);
	}

}
// ----- Motors ---------------------------------------------------------------



void cBot_init(void) {

	// init LED
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	// init buttons
	button_init(&buttons, 50);
	buttonRight = button_add(&buttons, Button1_GPIO_Port, Button1_Pin, BUTTON_ACTIVE_LOW);
	buttonDown = button_add(&buttons, Button2_GPIO_Port, Button2_Pin, BUTTON_ACTIVE_LOW);
	buttonLeft = button_add(&buttons, Button3_GPIO_Port, Button3_Pin, BUTTON_ACTIVE_LOW);
	buttonUp = button_add(&buttons, Button4_GPIO_Port, Button4_Pin, BUTTON_ACTIVE_LOW);

	// init buzzer
	buzzer_init(buzzer, &htim3, TIM_CHANNEL_1, 10);

	// init RGB LEDs
	ws2812b_init(rgbLeds, 10, &htim2, TIM_CHANNEL_1);
	ws2812b_clear(rgbLeds);
	ws2812b_update(rgbLeds);
	HAL_Delay(20);

	// init display
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(display, U8G2_R0, u8x8_byte_STM32_hw_i2c, u8x8_gpio_and_delay_STM32);
	u8g2_InitDisplay(display);
	u8g2_SetPowerSave(display, 0);
	u8g2_ClearBuffer(display);
	u8g2_SendBuffer(display);

	// init servo
	servo_init();

	// init serial communication
	//	HAL_Delay(300);
	//	sercom_init(&hostCom, USART1, 64, 64);
	//	sercom_transmitStr(&hostCom, "cBot v 0.1\n\r");

	// init range sensors
	rangeSensor_init();

	// init motors
	HAL_TIM_Base_Start_IT(&htim4);



}

