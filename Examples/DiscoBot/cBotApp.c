/*
 * cBotApp.c
 *
 * DiscoBot
 *
 * Dieses Programm versetzt cBot in den Disco-Modus:
 * - Die RGB-Leds leuchten in allen Farben.
 * - Sobald ein gewisser Abstand unterschritten wird oder der rechte Taster gedrückt wird, ...
 * - ... wird der Mario Song abgespielt und ...
 * - ... der Roboter fährt in Kreisbögen vorwärts und rückwärts oder dreht sich mal links- mal rechts herum.
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
#include "marioSong.h"

#include <stdlib.h>


#define ACTIVATION_RANGE 100

void init() {
	// init display
	char text[] = "DiscoBot";
	u8g2_ClearBuffer(display);
	u8g2_SetDrawColor(display, 1);
	u8g2_SetFont(display, u8g2_font_fub20_tf); // u8g2_font_10x20_mr) / u8g2_font_t0_22b_mr / u8g2_font_crox5tb_tf
	u8g2_DrawStr(display, (128 - u8g2_GetStrWidth(display, text))/2, (64 + u8g2_GetAscent(display))/2, text);
	u8g2_SendBuffer(display);
}

int hueStart = 0;
int side = 1;

void loop() {

	// update LEDs
	hueStart = (hueStart - 12 + 1536) % 1536;
	for ( int i = 0; i < 10; i++ ) {
		setLed(i, getColorHSV((hueStart + 128 * i) % 1536, 100, 10));
	}
	updateLeds();

	// complete dance move, if song is over
	if ( isMoving() && !isPlaying() ) {
		// do nothing
	}

	// start disco, if button is pressed or object is detected closer than ...
	else if ( !isPlaying() && ( isPressed(BUTTON_RIGHT) || (getRangeMm(SENSOR_LEFT) <= ACTIVATION_RANGE) ||
			(getRangeMm(SENSOR_MIDDLE) <= ACTIVATION_RANGE) || (getRangeMm(SENSOR_RIGHT) <= ACTIVATION_RANGE) ) ) {
		playMario();
		while ( isPressed(BUTTON_RIGHT) );	// wait until key is released
	}

	// perform dancing moves
	else if ( isPlaying() && !isMoving() ) {
		if ( rand() % 100 < 50 ) {
			driveArc(0.15 * side, 20);
			driveArc(0.15 * side, -20);
		}
		else {
			driveArc(0.03 * side, 360);
		}
		side = -side;
	}

	// wait for some time
	HAL_Delay(20);

}


