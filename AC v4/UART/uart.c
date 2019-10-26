/*
 * uart.c
 *
 *  Created on: 10 wrz 2019
 *      Author: Adam P.
 */
#include <avr/io.h>
#include "uart.h"

void UART_Init(uint8_t ubrr){
	UBRRH = (uint8_t) (ubrr>>8);
	UBRRL = (uint8_t) ubrr;

	UCSRB = (1<<RXEN) | (1<<TXEN);
	UCSRC = (1<<URSEL) | (1<<USBS) | (3<<UCSZ0);
}

void UART_Transmit(uint8_t data){
	while(!(UCSRA & (1<<UDRE)));
	UDR = data;
}

uint8_t UART_Receive(void){
	while(!(UCSRA & (1<<RXC)));
	return UDR;
}

void UART_StringTransmit(uint8_t * wsk){
	while(*wsk){
		UART_Transmit(*wsk);
		wsk++;
	}
}

