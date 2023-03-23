/*
 * cBotApp.c
 *
 * SensorBot
 *
 * Dieses Programm gibt periodisch die Sensorwerte auf dem Display aus.
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


#define BOTTOM_LED_VALUE 128

void init() {
	// turn on bottom LEDs
	for ( int i = 0; i < 8; i++ ) {
		setLed(i, getColorRGB(0, 0, 0));
	}
	setLed(8, getColorRGB(BOTTOM_LED_VALUE, BOTTOM_LED_VALUE, BOTTOM_LED_VALUE));
	setLed(9, getColorRGB(BOTTOM_LED_VALUE, BOTTOM_LED_VALUE, BOTTOM_LED_VALUE));
	updateLeds();
}

void loop() {
	char text[16];

	// init display buffer
	u8g2_ClearBuffer(display);
	u8g2_SetDrawColor(display, 1);

	// draw middle left
	sprintf(text, "%dmm", getRangeMm(SENSOR_LEFT));
	u8g2_SetFont(display, u8g2_font_t0_22b_mr);
	u8g2_DrawStr(display, 0, (64 + u8g2_GetAscent(display))/2, text);

	// draw middle range
	sprintf(text, "%dmm", getRangeMm(SENSOR_MIDDLE));
	u8g2_SetFont(display, u8g2_font_t0_22b_mr);
	u8g2_DrawStr(display, (128 - u8g2_GetStrWidth(display, text))/2, u8g2_GetAscent(display), text);

	// draw right range
	sprintf(text, "%dmm", getRangeMm(SENSOR_RIGHT));
	u8g2_SetFont(display, u8g2_font_t0_22b_mr);
	u8g2_DrawStr(display, 128 - u8g2_GetStrWidth(display, text), (64 + u8g2_GetAscent(display))/2, text);

	// draw light sensor values
	sprintf(text, "%d | %d", getLightValue(SENSOR_LEFT), getLightValue(SENSOR_RIGHT));
	u8g2_SetFont(display, u8g2_font_t0_22b_mr);
	u8g2_DrawStr(display, (128 - u8g2_GetStrWidth(display, text))/2, 63, text);

	// update display
	u8g2_SendBuffer(display);
}


