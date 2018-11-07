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
#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)
#define BAUDRATE (F_CPU/(16*(UBRR_VAL+1)))

#define UBBRVAL 51

#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

void uart_init(void);
unsigned char uart_recieve(void);
void uart_transmit(unsigned char data);
void serialSend(char* sendString);



#endif /* SERIAL_H_ */