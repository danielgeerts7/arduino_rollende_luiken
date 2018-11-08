/*
 * serial.c
 *
 * Created: 06-Nov-18 14:29:58
 *  Author: Daniel Geerts
 */

#include "serial.h"

void uart_init(void) {
	
	UBRR0H = 0;
	UBRR0L = UBBRVAL;
	UCSR0A = 0;
	UCSR0B = _BV(TXEN0) | _BV(RXEN0);
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}

// function to receive data
unsigned char uart_recieve (void)
{
	while(!(UCSR0A) & (1<<RXC0));                  // wait while data is being received
	return UDR0;                                   // return 8-bit data
}

unsigned char uartrecieve(unsigned char *x, unsigned char size)
{
	unsigned char i = 0;

	if (size == 0) return 0;            // return 0 if no space

	while (i < size - 1) {              // check space is available (including additional null char at end)
		unsigned char c;
		while ( !(UCSR0A & (1<<RXC0)) );  // wait for another char - WARNING this will wait forever if nothing is received
		c = UDR0;
		if (c == '\0') break;           // break on NULL character
		x[i] = c;                       // write into the supplied buffer
		x[i] = UDR0;
		i++;
	}
	x[i] = 0;                           // ensure string is null terminated

	return i + 1;                       // return number of characters written
}

// function to send data
void uart_transmit_char (unsigned char data)	// send as char, example: data=126 -> python receives '~'
{
	while (!( UCSR0A & (1<<UDRE0)));            // wait while register is free
	UDR0 = data;
}

void uart_transmit_int(unsigned int data) {	// send as int, example: data=3 -> python receives '3'
	float honderdtal = data / 100;
	if (honderdtal >= 1) {
		honderdtal += 48;
		uart_transmit_char(honderdtal);
	}
	
	float tiental = data % 100;
	tiental = tiental / 10;
	if (tiental >= 1) {
		tiental += 48;
		uart_transmit_char(tiental);
	}
	
	int rest = data % 10;
	rest += 48;
	uart_transmit_char(rest);
}

void serialSend(char* sendString){
	for (int i = 0; i < strlen(sendString); i++){
		while (( UCSR0A & (1<<UDRE0))  == 0){};
		UDR0 = sendString[i];
	}
}