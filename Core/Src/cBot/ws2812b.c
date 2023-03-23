/*
 * ws2812b.c
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

#include <ws2812b.h>

void ws2812b_init(ws2812b_t *ws2812b, uint16_t size, TIM_HandleTypeDef *htim, uint32_t channel) {

	// setup
	ws2812b->size = size;
	ws2812b->rgbData = malloc(size * 24 + 1);
	ws2812b->timingLow = (uint8_t)((htim->Init.Period + 1) / 3);
	ws2812b->timingHigh = (uint8_t)((htim->Init.Period + 1) * 2 / 3);
	ws2812b->htim = htim;
	ws2812b->channel = channel;

	// clear rgb data
	ws2812b_clear(ws2812b);

	// append reset timing to the end of PWM sequence
	ws2812b->rgbData[size * 24] = 0;
}

void ws2812b_deInit(ws2812b_t *ws2812b) {
	free(ws2812b->rgbData);
}

void ws2812b_clear(ws2812b_t *ws2812b) {
	for ( int i = ws2812b->size * 24 -1; i >= 0; i-- ) {
		ws2812b->rgbData[i] = ws2812b->timingLow;
	}
}

uint32_t ws2812b_colorRGB(uint8_t r, uint8_t g, uint8_t b) {
	return (uint32_t)g << 16 | (uint32_t)r << 8 | (uint32_t)b;
}

uint32_t ws2812b_colorHSV(uint16_t h, uint8_t s, uint8_t v) {
	uint32_t r = 0, g = 0, b = 0, base;

	// value (0..1535)
	if ( h < 256 ) {
		r = 255;
		g = h;
		b = 0;
	} else if ( h < 512 ) {
		r = 255 - (h - 256);
		g = 255;
		b = 0;
	} else if ( h < 768 ) {
		r = 0;
		g = 255;
		b = h - 512;
	} else if ( h < 1024 ) {
		r = 0;
		g = 255 - (h - 768);
		b = 255;
	} else if ( h < 1280 ) {
		r = h - 1024;
		g = 0;
		b = 255;
	} else if ( h < 1536 ) {
		r = 255;
		g = 0;
		b = 255 - (h - 1280);
	}

	// saturation (0..100%)
	base = (100 - s) * 255 / 100;
	r = r * s / 100 + base;
	g = g * s / 100 + base;
	b = b * s / 100 + base;

	// value (0..100%)
	r = (r * v) / 100;
	g = (g * v) / 100;
	b = (b * v) / 100;

	return (uint32_t)g << 16 | (uint32_t)r << 8 | (uint32_t)b;
}

//uint32_t ws2812b_colorHSV(uint16_t h, uint8_t s, uint8_t v) {
//	uint32_t r = 0, g = 0, b = 0, base;
//
//	// value (0..359)
//	if ( h < 60 ) {
//		r = 255;
//		b = 0;
//		g = 4.25 * h;
//	} else if ( h < 120 ) {
//		g = 255;
//		b = 0;
//		r = 255 - (4.25 * (h - 60));
//	} else if ( h < 180 ) {
//		r = 0;
//		g = 255;
//		b = 4.25 * (h-120);
//	} else if ( h < 240 ) {
//		r = 0;
//		b = 255;
//		g = 255 - (4.25 * (h - 180));
//	} else if ( h < 300 ) {
//		g = 0;
//		b = 255;
//		r = 4.25 * (h-240);
//	} else if ( h < 360 ) {
//		r = 255;
//		g = 0;
//		b = 255 - (4.25 * (h - 300));
//	}
//
//	// saturation (0..100%)
//	base = (100 - s) * 255 / 100;
//	r = r * s / 100 + base;
//	g = g * s / 100 + base;
//	b = b * s / 100 + base;
//
//	// value (0..100%)
//	r = (r * v) / 100;
//	g = (g * v) / 100;
//	b = (b * v) / 100;
//
//	return (uint32_t)g << 16 | (uint32_t)r << 8 | (uint32_t)b;
//}


void ws2812b_setColor(ws2812b_t *ws2812b, uint16_t id, uint32_t color) {
	uint8_t *buf = ws2812b->rgbData + (id % ws2812b->size) * 24;
	for ( uint32_t i  = 0; i < 24; i++) {
		*(buf++) = (color & 0x800000) ? ws2812b->timingHigh : ws2812b->timingLow;
		color = color << 1;
	}
}

void ws2812b_update(ws2812b_t *ws2812b) {
	HAL_TIM_Base_Stop(ws2812b->htim);
	__HAL_TIM_SET_COUNTER(ws2812b->htim, 0);
	HAL_TIM_PWM_Start_DMA(ws2812b->htim, ws2812b->channel, (uint32_t *)ws2812b->rgbData, ws2812b->size * 24 + 1);
	HAL_TIM_Base_Start(ws2812b->htim);
}





