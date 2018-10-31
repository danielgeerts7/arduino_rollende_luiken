/*
 * arduino_rollende_luiken.c
 *
 * Created: 30-Oct-18 12:21:19
 *  Author: DanielMAC
 */ 


#include <avr/io.h>
#include <util/delay.h>

const int trigPin = 2; // Trigger		PD2
const int echoPin = 3; // Echo			PD3
// Red LED		PD4
// Yellow LED	PD5
// Green LED	PD6

int main(void)
{
	DDRD = 0b01110100; // Set Trigger as OUTPUT and ECHO as INPUT && set all LEDs as OUTPUT
	
	long duration;
	int distance;
	
    while(1)
    {
		duration = PORTD3;
		distance = duration*0.034/2;
		
		if (distance < 10) {
			//PORTD = PORTD & 0b01000000;
			PORTD ^= (1<<6);
		} else if (distance < 20) {
			//PORTD = PORTD & 0b00100000;
			PORTD ^= (1<<5);
		} else {
			//PORTD = PORTD & 0b00010000;
			PORTD ^= (1<<4);
		}
    }
}