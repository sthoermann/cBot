/*
 * button.h
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

#ifndef SRC_BUTTON_H_
#define SRC_BUTTON_H_


#include <stdint.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"


typedef enum {
	BUTTON_ACTIVE_LOW = 0u,
	BUTTON_ACTIVE_HIGH
} button_type_t;

typedef struct button {
	GPIO_TypeDef *port;
	uint16_t pin;
	button_type_t type;
	uint16_t offTime;
	uint8_t isPressed;
	struct button *nextButton;
} button_t;


typedef struct {
	uint16_t offDelayMs;
	uint32_t lastUpdate;
	button_t *firstButton;
} buttons_t;


void button_init(buttons_t *buttons, uint16_t offDelayMs);
button_t* button_add(buttons_t *buttons, GPIO_TypeDef *port, uint16_t pin, button_type_t type);
void button_update(buttons_t *buttons);


#endif /* SRC_BUTTON_H_ */
