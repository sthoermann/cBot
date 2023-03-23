/*
 * buzzer.c
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

#include "buzzer.h"

void buzzer_init(buzzer_t *buzzer, TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pauseMs) {
	buzzer->htim = htim;
	buzzer->channel = channel;
	buzzer->pauseMs = pauseMs;
	buzzer->isTone = 0;
	buzzer->isLocked = 0;
	buzzer->nextUpdate = 0;
	buzzer->firstTone = NULL;
	buzzer->lastTone = NULL;
}

void buzzer_setFrequency(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t frequency) {
	if ( frequency > 0 ) {
		__HAL_TIM_SET_AUTORELOAD(htim, SystemCoreClock / htim->Init.Prescaler / frequency / 2);
		__HAL_TIM_SET_COUNTER(htim, 0);
		HAL_TIM_OC_Start(htim, channel);
	}
	else {
		HAL_TIM_OC_Stop(htim, channel);
	}
}

void buzzer_playNote(buzzer_t *buzzer, uint16_t frequency, uint16_t durationMs) {

	// create tone object
	tone_t *tone = malloc(sizeof(tone_t));
	tone->frequency = frequency;
	tone->durationMs = durationMs;
	tone->nextTone = NULL;

	// lock tone queue
	buzzer->isLocked = 1;

	// add tone object
	if ( buzzer->lastTone == NULL ) {
		buzzer->firstTone = tone;
		buzzer->lastTone = tone;
	}
	else {
		buzzer->lastTone->nextTone = tone;
		buzzer->lastTone = tone;
	}

	// unlock tone queue
	buzzer->isLocked = 0;
}

int buzzer_isPlaying(buzzer_t *buzzer) {
	if ( buzzer->firstTone != NULL ) return 1;
	if ( buzzer->nextUpdate >= HAL_GetTick() ) return 1;
	return 0;
}

void buzzer_stopPlaying(buzzer_t *buzzer) {

	// empty tone queue
	if ( buzzer->firstTone != NULL ) {
		buzzer->isLocked = 1;
		while ( buzzer->firstTone != NULL ) {
			tone_t *tone = buzzer->firstTone;
			buzzer->firstTone = tone->nextTone;
			free(tone);
		}
		buzzer->lastTone = NULL;
		buzzer->isLocked = 0;
	}

	// turn off buzzer
	buzzer_setFrequency(buzzer->htim, buzzer->channel, 0);
	buzzer->isTone = 0;

	// reset update time
	buzzer->nextUpdate = 0;

}

tone_t* buzzer_getNextTone(buzzer_t *buzzer) {

	// get first tone handle
	tone_t *tone = buzzer->firstTone;

	// detach tone from queue, if one is present
	if ( tone != NULL ) {
		buzzer->firstTone = tone->nextTone;
		if ( buzzer->firstTone == NULL ) {
			buzzer->lastTone = NULL;
		}
	}

	return tone;
}

void buzzer_update(buzzer_t *buzzer) {

	// get current time
	uint32_t currentTick = HAL_GetTick();

	if ( currentTick >= buzzer->nextUpdate ) {

		// don't update, if tone queue is locked
		if ( buzzer->isLocked ) return;

		if ( buzzer->isTone ) {
			// turn off buzzer and pause for a certain time
			buzzer_setFrequency(buzzer->htim, buzzer->channel, 0);
			buzzer->nextUpdate = buzzer->nextUpdate + buzzer->pauseMs;
			buzzer->isTone = 0;
		}
		else {

			// get next tone
			tone_t *tone = buzzer_getNextTone(buzzer);

			if ( tone != NULL ) {

				// sync with current time for new tones
				if ( currentTick - buzzer->nextUpdate > buzzer->pauseMs ) {
					buzzer->nextUpdate = currentTick;
				}

				// play tone
				if ( tone->durationMs > buzzer->pauseMs ) {
					buzzer_setFrequency(buzzer->htim, buzzer->channel, tone->frequency);
					buzzer->nextUpdate = buzzer->nextUpdate + tone->durationMs - buzzer->pauseMs;
					buzzer->isTone = 1;
				}
				else {
					buzzer->nextUpdate = buzzer->nextUpdate + tone->durationMs;
				}

				// dispose tone object
				free(tone);
			}
		}

	}

}






























