/*
 * sercom.h
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

#ifndef SERCOM_H_
#define SERCOM_H_


#include <stm32f103xb.h>
#include <stm32f1xx_ll_usart.h>

#include "fifo.h"


#define SERCOM_LINE_BREAK '\n'


typedef struct {
	USART_TypeDef *uart;
	fifo_t rxFifo, txFifo;
	volatile uint8_t isTxActive;
	volatile uint16_t lineCount;
} sercom_t;


void sercom_init(sercom_t *uartCom, USART_TypeDef *uart, uint16_t rxBufferSize, uint16_t txBufferSize);
void sercom_irqHandler(sercom_t *uartCom);
void sercom_transmitStr(sercom_t *uartCom, char *str);
uint16_t sercom_bytesAvailable(sercom_t *uartCom);
uint16_t sercom_linesAvailable(sercom_t *uartCom);
uint8_t sercom_readByte(sercom_t *uartCom);
uint16_t sercom_readLine(sercom_t *uartCom, char *line, uint16_t maxLen);


#endif /* SERCOM_H_ */
