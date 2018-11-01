/*
 * arduino_rollende_luiken.c
 *
 * Created: 30-Oct-18 12:21:19
 *  Author: Daniel Geerts
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "display.h"

const int trigPin = 0;		// Trigger		PD0
const int echoPin = 3;		// Echo			PD3
const int RED_LED = 4;		// Red LED		PD4
const int YELLOW_LED = 5;	// Yellow LED	PD5
const int GREEN_LED = 6;	// Green LED	PD6

volatile uint16_t gv_counter; // 16 bit counter value
volatile uint8_t gv_echo; // a flag

void init_ports(void)
{
	// Set Trigger to OUTPUT, Echo to INPUT, Red LED to OUTPUT, Yellow LED to OUTPUT, Green LED to OUTPUT
	DDRD = (1<<trigPin) | (0<<echoPin) | (1<<RED_LED) | (1<<YELLOW_LED) | (1<<GREEN_LED);
	
	// Set Clock to OUTPUT, Strobe to OUTPUT, Data to OUTPUT
	DDRB = (1<<strobe) | (1<<clock) | (1<<data);
	
	sendCommand(0x89); // activate and set brightness to medium
}

void init_timer(void)
{
	// prescaling : max time = 2^16/16E6 = 4.1 ms, 4.1 >> 2.3, so no prescaling required
	// normal mode, no prescale, stop timer
	TCCR1A = 0;
	TCCR1B = 0;
}

void init_ext_int(void)
{
	// any change triggers ext interrupt 1
	EICRA = (1 << ISC10);
	EIMSK = (1 << INT1);
}


uint16_t calc_cm(uint16_t counter)
{	
	// Min 2cm - Max 70cm
	uint16_t microSec = counter / 16;
	return (microSec/58.2);
}


int main(void)
{
	init_timer();	 // Enable timer interrupts
	init_ports();    // Enable ports
	init_ext_int();	 // Turn required interrupts on
	reset_display(); // Clear display
	sei();			 // Set interrupt flag
	
	_delay_ms(50);	// Make sure everything is initialized
	
    while(1)
    {
		gv_echo = BEGIN;		// Set gv_echo to BEGIN
		PORTD |= _BV(trigPin);	// Set trigPin to 1 -> send pulse
		_delay_us(12);			// Wait for pulse to complete
		PORTD = 0x00;			// Clear PORTD (trigPin & LEDs)
		
		_delay_ms(30);			// Wait to make sure the signal of the pulse has been returned to echo
		
		uint16_t distance = calc_cm(gv_counter);
		show_distance(distance);
		
		if (distance < 10) {
			PORTD |= (1<<GREEN_LED);		// Set green LED to 1
		} else if (distance < 30) {
			PORTD |= (1<<YELLOW_LED);		// Set yellow LED to 1
		} else {
			PORTD |= (1<<RED_LED);			// Set red LED to 1
		}
		
		_delay_ms(500);
    }
}

ISR (INT1_vect)
{
	if (gv_echo == BEGIN) {
		TCNT1 = 0;
		TCCR1B = _BV(CS10);
		gv_echo = END;
	} else {
		TCCR1B = 0;
		gv_counter = TCNT1;
	}
}