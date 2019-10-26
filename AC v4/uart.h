/*
 * uart.h
 *
 *  Created on: 10 wrz 2019
 *      Author: n0name
 */

#ifndef UART_H_
#define UART_H_


void UART_Init(uint8_t ubrr);
void UART_Transmit(uint8_t data);
uint8_t UART_Receive(void);
void UART_StringTransmit(uint8_t * wsk);

#endif /* UART_H_ */
