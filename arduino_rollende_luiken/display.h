/*
 * display.h
 *
 *  Author: Hanzehogeschool
 */ 

#include <avr/io.h>

#ifndef DISPLAY_H_INCLUDED
#define DISPLAY_H_INCLUDED

#define HIGH 0x1
#define LOW 0x0

#define BEGIN 0x1
#define END 0x0

void reset_display();
void show_distance(uint16_t cm);
void sendCommand(uint8_t value);
void write(uint8_t pin, uint8_t val);
void shiftOut (uint8_t val);

// PORTB
const static uint8_t data = 0;		// UNO Pin 8
const static uint8_t clock = 1;		// UNO Pin 9
const static uint8_t strobe = 2;	// UNO Pin 10

#endif