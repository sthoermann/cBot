/*
 * cBot.h
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

#ifndef SRC_CBOT_H_
#define SRC_CBOT_H_


#include <math.h>

#include "main.h"

#include "buzzerNotes.h"
#include "ws2812b.h"

#include "u8g2.h"


#define SERVO_MAX 1000


typedef enum {
	BUTTON_UP,
	BUTTON_DOWN,
	BUTTON_RIGHT,
	BUTTON_LEFT
} buttonId;

typedef enum {
	SENSOR_LEFT,
	SENSOR_MIDDLE,
	SENSOR_RIGHT
} sensorId;


typedef struct {
	GPIO_TypeDef *triggerPort;
	uint16_t triggerPin;
	uint16_t echoPin;
	uint16_t counter;
	uint16_t echoStartCoarse;
	uint16_t echoStartExact;
	uint16_t echoStopCoarse;
	uint16_t echoStopExact;
	uint16_t rangeMM;
} rangeSensor_t;


typedef struct motorRpm {
	float rpmLeft;
	float rpmRight;
	uint16_t durationMs;
	struct motorRpm *nextMotorRpm;
} motorRpm_t;


typedef struct {
	uint8_t isLocked;
	uint8_t isRpm;
	uint32_t nextUpdate;
	motorRpm_t *firstMotorRpm;
	motorRpm_t *lastMotorRpm;
} motorStatus_t;


void cBot_init(void);

void setServo(uint8_t servoId, uint16_t position);

int isPressed(buttonId b);

void beep(uint16_t frequency, uint16_t durationMs);
void playNote(uint16_t frequency, uint16_t durationMs);
void playPause(uint16_t durationMs);
int isPlaying();
void stopPlaying();

int getRangeMm(sensorId id);
int getLightValue(sensorId id);

void clearLeds();
uint32_t getColorRGB(uint8_t r, uint8_t g, uint8_t b);
uint32_t getColorHSV(uint16_t h, uint8_t s, uint8_t v);
void setLed(uint16_t id, uint32_t color);
void updateLeds();

void addMotorRpm(float rpmL, float rpmR, uint16_t ms);
void addRobotVel(float linear, float angular, uint16_t ms);
void getRpmFromVelocity(float *left_rpm, float *right_rpm, float linear_velocity, float angular_rate); // linear velocity in m/s; angular rate in radians/s
void setMotorRpm(float rpmL, float rpmR);
void stopMotor();
void driveStrait(float distance); // distance in m
void driveArc(float radius, float angle); // radius in m, angle in degree
void turn(float angle); // angle in degree
void accelleratedMove(float rpmL, float rpmR, float duration);
int isMoving();



#endif /* SRC_CBOT_H_ */
