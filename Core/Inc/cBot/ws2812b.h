/*
 * ws2812b.h
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

#ifndef WS2812B_H_
#define WS2812B_H_

#include <stdint.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"


typedef struct {
	uint16_t size;
	uint8_t *rgbData;
	uint8_t timingLow;
	uint8_t timingHigh;
	TIM_HandleTypeDef *htim;
	uint32_t channel;
} ws2812b_t;


void ws2812b_init(ws2812b_t *ws2812b, uint16_t size, TIM_HandleTypeDef *htim, uint32_t channel);
void ws2812b_deInit(ws2812b_t *ws2812b);
void ws2812b_clear(ws2812b_t *ws2812b);
uint32_t ws2812b_colorRGB(uint8_t r, uint8_t g, uint8_t b);
uint32_t ws2812b_colorHSV(uint16_t h, uint8_t s, uint8_t v);
void ws2812b_setColor(ws2812b_t *ws2812b, uint16_t id, uint32_t color);
void ws2812b_update(ws2812b_t *ws2812b);


#endif /* WS2812B_H_ */







