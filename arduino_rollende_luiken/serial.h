/*
 * serial.h
 *
 * Created: 06-Nov-18 14:31:13
 *  Author: Daniel Geerts
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>

#define F_CPU 16E6
#define BAUD 19200
#define UBBRVAL 51

#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

void uart_init(void);
unsigned char uart_recieve(void);
void uart_transmit_char(unsigned char data);
void uart_transmit_int(unsigned int data);
void serialSend(char* sendString);



#endif /* SERIAL_H_ */