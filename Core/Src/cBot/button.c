/*
 * button.c
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


#include "button.h"


void button_init(buttons_t *buttons, uint16_t offDelayMs) {
	buttons->offDelayMs = offDelayMs;
	buttons->lastUpdate = HAL_GetTick();
	buttons->firstButton = NULL;
}

button_t* button_add(buttons_t *buttons, GPIO_TypeDef *port, uint16_t pin, button_type_t type) {

	// create button object
	button_t *button = malloc(sizeof(button_t));
	button->port = port;
	button->pin = pin;
	button->type = type;
	button->offTime = buttons->offDelayMs;
	button->isPressed = 0;

	// add as first button element
	button->nextButton = buttons->firstButton;
	buttons->firstButton = button;

	return button;
}

void button_update(buttons_t *buttons) {
	uint32_t currentTick = HAL_GetTick();
	if ( buttons->lastUpdate != currentTick ) {
		buttons->lastUpdate = currentTick;

		button_t *button = buttons->firstButton;
		while ( button != NULL ) {

			if ( ((button->type == BUTTON_ACTIVE_LOW) && (HAL_GPIO_ReadPin(button->port, button->pin) == GPIO_PIN_RESET)) ||
					((button->type == BUTTON_ACTIVE_HIGH) && (HAL_GPIO_ReadPin(button->port, button->pin) == GPIO_PIN_SET)) ) {
				button->isPressed = 1;
				button->offTime = 0;
			}
			else if ( button->isPressed ) {
				button->offTime++;
				if ( button->offTime >= buttons->offDelayMs ) {
					button->isPressed = 0;
					button->offTime = buttons->offDelayMs;
				}
			}

			button = button->nextButton;
		}

	}
}


