/*
 * cBotApp.c
 *
 * WallBot
 *
 * Mit diesem Programm kann der cBot einer Wand auf der linken Seite folgen.
 * Der linke Sensor ist dafür im 90° Winkel nach links zu stellen, der rechte ca. 45° nach links.
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

#include "cBotApp.h"

#include <stdio.h>


#define WALL_DISTANCE 90
#define OFFSET_SENSOR_RIGHT 30
#define OFFSET_SENSOR_LEFT 45
#define MAX_RANGE 250

void updateDisplay() {

	// read range values
	int rangeLeft = getRangeMm(SENSOR_LEFT);
	int rangeMiddle = getRangeMm(SENSOR_MIDDLE);
	int rangeRight = getRangeMm(SENSOR_RIGHT);

	// create range text
	char text[64], rangeTextLeft[16], rangeTextMiddle[16], rangeTextRight[16];
	if ( rangeLeft <= MAX_RANGE ) sprintf(rangeTextLeft, "%3d", rangeLeft);
	else sprintf(rangeTextLeft, "---");
	if ( rangeMiddle <= MAX_RANGE ) sprintf(rangeTextMiddle, "%3d", rangeMiddle);
	else sprintf(rangeTextMiddle, "---");
	if ( rangeRight <= MAX_RANGE ) sprintf(rangeTextRight, "%3d", rangeRight);
	else sprintf(rangeTextRight, "---");
	sprintf(text, "%s|%s|%s", rangeTextLeft, rangeTextMiddle, rangeTextRight);

	// update display
	u8g2_ClearBuffer(display);
	u8g2_SetDrawColor(display, 1);
	u8g2_SetFont(display, u8g2_font_t0_22b_mr);
	u8g2_DrawStr(display, (128 - u8g2_GetStrWidth(display, text))/2, 63, text);
	u8g2_SendBuffer(display);

}

void init() {
	// initialize your cBot here
}


#define ERROR_HISTORY_SIZE 100

int isWaiting = 1;
int errorHistory[ERROR_HISTORY_SIZE], historyId;

void resetErrorHistory() {
	for ( int i = ERROR_HISTORY_SIZE - 1; i >= 0; i-- ) errorHistory[i] = 0;
	historyId = 0;
}

void loop() {

	// read range values
	int rangeLeft = getRangeMm(SENSOR_LEFT);
	int rangeMiddle = getRangeMm(SENSOR_MIDDLE);
	int rangeRight = getRangeMm(SENSOR_RIGHT);

	if ( !isWaiting ) {
		if ( (rangeLeft > MAX_RANGE) && (rangeMiddle > MAX_RANGE) && (rangeRight > MAX_RANGE) ) {
			driveArc((WALL_DISTANCE + OFFSET_SENSOR_LEFT) / 1000.0, 30); // radius in m, angle in degree
			while ( isMoving() );
			resetErrorHistory();
		}
		else
			if ( (rangeMiddle <= WALL_DISTANCE) || (rangeRight - OFFSET_SENSOR_RIGHT <= WALL_DISTANCE) ) {
			turn(-15);
			while ( isMoving() );
			resetErrorHistory();
		}
//		else if (rangeLeft <= MAX_RANGE) {

			// determine error
			int error = WALL_DISTANCE - rangeLeft;
			if ( error < -WALL_DISTANCE ) error = -WALL_DISTANCE;

			// get old error and store current error
			int errorOld = errorHistory[historyId];
			errorHistory[historyId] = error;
			historyId = (historyId + 1) % ERROR_HISTORY_SIZE;
			if ( historyId == 0 ) HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

			float angularSpeed = 0.002 * error + 0.002 * (error - errorOld);

			float rpmLeft, rpmRight;
			getRpmFromVelocity(&rpmLeft, &rpmRight, 0.02, -angularSpeed); // linear velocity in m/s; angular rate in radians/s
			setMotorRpm(rpmLeft, rpmRight);
//		}
	}
	else {
		setMotorRpm(0, 0);
	}

	// start or stop robot
	if ( isPressed(BUTTON_RIGHT) ) {
		isWaiting = 1 - isWaiting;
		if ( !isWaiting ) resetErrorHistory();
		while ( isPressed(BUTTON_RIGHT) );	// wait until key is released
	}

	updateDisplay();

}


