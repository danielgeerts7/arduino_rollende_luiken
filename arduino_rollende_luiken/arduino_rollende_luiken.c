/*
 * arduino_rollende_luiken.c
 *
 * Created: 30-Oct-18 12:21:19
 *  Author: DanielMAC
 */ 


#include <avr/io.h>

const int trigPin = 2; // Trigger		PD2
const int echoPin = 3; // Echo			PD3
// Red LED		PD4
// Yellow LED	PD5
// Green LED	PD6

int main(void)
{
	DDRD = 0b01110100; // Set Trigger as OUTPUT and ECHO as INPUT && set all LEDs as OUTPUT
	
    while(1)
    {
        //TODO:: Please write your application code
    }
}