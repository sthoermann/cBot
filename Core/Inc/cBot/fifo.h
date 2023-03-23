/*
 * fifo.h
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

#ifndef FIFO_H_
#define FIFO_H_


#include <stdint.h>
#include <stdlib.h>


typedef struct {
	uint8_t *buffer;
	uint16_t bufferSize;
	volatile uint16_t items;
	volatile uint16_t readPtr;
	volatile uint16_t writePtr;
} fifo_t;


void fifo_init(fifo_t* fifo, uint16_t size);
void fifo_deInit(fifo_t* fifo);
void fifo_putByte(fifo_t* fifo, uint8_t data);
uint16_t fifo_bytesAvailable(fifo_t* fifo);
uint8_t fifo_isFull(fifo_t* fifo);
uint8_t fifo_getByte(fifo_t* fifo);



#endif /* FIFO_H_ */
