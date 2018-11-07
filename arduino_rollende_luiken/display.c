/*
 * display.c
 *
 *  Author: Hanzehogeschool
 */ 

/* 
 * TM1638 - LED&KEY
 * 
 * DIO : uno 8  (PB0) data
 * CLK : uno 9  (PB1) clock
 * STB : uno 10 (PB2) strobe
 *
 */

#include "display.h"

void reset_display()
{
    // clear memory - all 16 addresses
    sendCommand(0x40); // set auto increment mode
    write(strobe, LOW);
    shiftOut(0xc0);   // set starting address to 0
    for(uint8_t i = 0; i < 16; i++)
    {
        shiftOut(0x00);
    }
    write(strobe, HIGH);
    sendCommand(0x89);  // activate and set brightness to medium
}

void show_distance(uint16_t cm)
{
	if (cm <= 0) {
		// display text: error || ultrasoon sensor not working
		//cm = 666;
	}
                        /*0*/ /*1*/ /*2*/ /*3*/ /*4*/ /*5*/ /*6*/ /*7*/ /*8*/ /*9*/
    uint8_t digits[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};
    uint8_t ar[8] = {0};
    uint8_t digit = 0, i = 0;
    uint8_t temp, nrs, spaces;
    
    // cm=1234 -> ar[0..7] = {4,3,2,1,0,0,0,0}
    while (cm > 0 && i < 8) {
        digit = cm % 10;
        ar[i] = digit;
        cm = cm / 10;
        i++;
    }

    nrs = i;      // 4 digits
    spaces = 8-i; // 4 leading spaces  
    
    // invert array -> ar[0..7] = {0,0,0,0,1,2,3,4}
    uint8_t n = 7;
    for (i=0; i<4; i++) {
        temp = ar[i];
        ar[i] = ar[n];
        ar[n] = temp;
        n--;
    }
    
    write(strobe, LOW);
    shiftOut(0xc0); // set starting address = 0
    // leading spaces
    for (i=0; i<8; i++) {
        if (i < spaces) {
            shiftOut(0x00);
        } else {
            shiftOut(digits[ar[i]]);
        }           
        shiftOut(0x00); // the dot
    }
    
    write(strobe, HIGH);
}

void sendCommand(uint8_t value)
{
    write(strobe, LOW);
    shiftOut(value);
    write(strobe, HIGH);
}

// write value to pin
void write(uint8_t pin, uint8_t val)
{
    if (val == LOW) {
        PORTB &= ~(_BV(pin)); // clear bit
    } else {
        PORTB |= _BV(pin); // set bit
    }
}

// shift out value to data
void shiftOut (uint8_t val)
{
    uint8_t i;
    for (i = 0; i < 8; i++)  {
        write(clock, LOW);   // bit valid on rising edge
        write(data, val & 1 ? HIGH : LOW); // lsb first
        val = val >> 1;
        write(clock, HIGH);
    }
}
