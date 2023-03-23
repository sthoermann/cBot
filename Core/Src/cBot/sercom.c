/*
 * sercom.c
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


#include "sercom.h"


void sercom_startTransmission(sercom_t *sercom);


void sercom_init(sercom_t *sercom, USART_TypeDef *uart, uint16_t rxBufferSize, uint16_t txBufferSize) {
	// initialize data object
	sercom->uart = uart;
	fifo_init(&sercom->rxFifo, rxBufferSize);
	fifo_init(&sercom->txFifo, txBufferSize);
	sercom->isTxActive = 0;
	sercom->lineCount = 0;

	// enable interrupts
	LL_USART_EnableIT_RXNE(uart);
	LL_USART_EnableIT_TC(uart);
}


void sercom_irqHandler(sercom_t *sercom) {
	if (LL_USART_IsActiveFlag_RXNE(sercom->uart)) {

		// read byte from receive register
		uint8_t b = LL_USART_ReceiveData8(sercom->uart);

		if ( !fifo_isFull(&sercom->rxFifo) ) {

			// store byte in rx buffer
			fifo_putByte(&sercom->rxFifo, b);

			// update line counter
			if ( b == SERCOM_LINE_BREAK ) {
				sercom->lineCount++;
			}

		}
		// Todo: How to report buffer overrun?

	}
	else if (LL_USART_IsActiveFlag_TC(sercom->uart)) {

		// clear interrupt flag
		LL_USART_ClearFlag_TC(sercom->uart);

		// transmit next byte, if available
		if ( fifo_bytesAvailable(&sercom->txFifo) ) {
			LL_USART_TransmitData8(sercom->uart, fifo_getByte(&sercom->txFifo));
		}
		else {
			sercom->isTxActive = 0;
		}

	}
}

void sercom_startTransmission(sercom_t *sercom) {
	if ( !sercom->isTxActive && fifo_bytesAvailable(&sercom->txFifo) ) {
		LL_USART_TransmitData8(sercom->uart, fifo_getByte(&sercom->txFifo));
		sercom->isTxActive = 1;
	}
}

void sercom_transmitStr(sercom_t *sercom, char *str) {

	// disable data transmission while filling the tx buffer
	LL_USART_DisableIT_TC(sercom->uart);

	// filling the tx buffer
	uint8_t b;
	while ( (b = (uint8_t)*str++) != 0 ) {

		// wait for space in buffer, if required
		if ( fifo_isFull(&sercom->txFifo) ) {
			sercom_startTransmission(sercom);
			LL_USART_EnableIT_TC(sercom->uart);
			while ( fifo_isFull(&sercom->txFifo) ) ;
			LL_USART_DisableIT_TC(sercom->uart);
		}

		fifo_putByte(&sercom->txFifo, b);
	}

	// start transmission, if not yet running
	sercom_startTransmission(sercom);

	// enable further transmissions
	LL_USART_EnableIT_TC(sercom->uart);

}

uint16_t sercom_linesAvailable(sercom_t *sercom) {
	return sercom->lineCount;
}

uint16_t sercom_bytesAvailable(sercom_t *sercom) {
	return sercom->rxFifo.items;
}

uint16_t sercom_readLine(sercom_t *sercom, char *line, uint16_t maxLen) {
	uint16_t len = 0;
	if ( sercom->lineCount > 0 ) {

		// disable receive interrupt while reading data from Rx queue
		LL_USART_DisableIT_RXNE(sercom->uart);

		// read line from Rx queue
		maxLen--;
		uint8_t b;
		do {
			b = fifo_getByte(&sercom->rxFifo);
			if ( b == SERCOM_LINE_BREAK ) {
				sercom->lineCount--;
			}
			else {
				line[len] = (char)b;
				len++;
			}
		} while ( (b != SERCOM_LINE_BREAK) && (len < maxLen) );

		// enable receive interrupt
		LL_USART_EnableIT_RXNE(sercom->uart);

	}
	line[len] = 0;
	return len;
}

uint8_t sercom_readByte(sercom_t *sercom) {

	// wait until data available
	while ( !fifo_bytesAvailable(&sercom->rxFifo) ) ;

	// disable receive interrupt while reading data from Rx queue
	LL_USART_DisableIT_RXNE(sercom->uart);

	// read byte
	uint8_t b = fifo_getByte(&sercom->rxFifo);

	// correct line counter
	if ( b == SERCOM_LINE_BREAK ) {
		sercom->lineCount--;
	}

	// enable receive interrupt
	LL_USART_EnableIT_RXNE(sercom->uart);

	return b;
}


