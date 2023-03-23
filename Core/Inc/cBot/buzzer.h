/*
 * buzzer.h
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

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_


#include <stdint.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"


typedef struct tone {
	uint16_t frequency;
	uint16_t durationMs;
	struct tone *nextTone;
} tone_t;


typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	uint16_t pauseMs;
	uint8_t isTone;
	uint8_t isLocked;
	uint32_t nextUpdate;
	tone_t *firstTone;
	tone_t *lastTone;
} buzzer_t;


void buzzer_init(buzzer_t *buzzer, TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pauseMs);
void buzzer_setFrequency(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t frequency);
void buzzer_playNote(buzzer_t *buzzer, uint16_t frequency, uint16_t durationMs);
int buzzer_isPlaying(buzzer_t *buzzer);
void buzzer_stopPlaying(buzzer_t *buzzer);
void buzzer_update(buzzer_t *buzzer);


#endif /* INC_BUZZER_H_ */








































