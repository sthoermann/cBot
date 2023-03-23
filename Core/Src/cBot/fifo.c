/*
 * fifo.c
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


#include <fifo.h>



void fifo_init(fifo_t* fifo, uint16_t size) {
	fifo->buffer = malloc(size);
	fifo->bufferSize = size;
	fifo->items = 0;
	fifo->readPtr = 0;
	fifo->writePtr = 0;
}

void fifo_deInit(fifo_t* fifo) {
	free(fifo->buffer);
}


void fifo_putByte(fifo_t* fifo, uint8_t data) {
	if ( fifo->items < fifo->bufferSize ) {
		fifo->buffer[fifo->writePtr] = data;
		fifo->items++;
		fifo->writePtr = (fifo->writePtr + 1) % fifo->bufferSize;
	}
}


uint16_t fifo_bytesAvailable(fifo_t* fifo) {
	return fifo->items;
}



uint8_t fifo_getByte(fifo_t* fifo) {

	// return 0, if no data available
	if ( fifo->items == 0 ) return 0;

	// retrieve data and update management information
	uint8_t data = fifo->buffer[fifo->readPtr];
	fifo->items--;
	fifo->readPtr = (fifo->readPtr + 1) % fifo->bufferSize;

	return data;
}

uint8_t fifo_isFull(fifo_t* fifo) {
	return fifo->items == fifo->bufferSize;
}





