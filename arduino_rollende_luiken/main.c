/*
 * arduino_rollende_luiken.c
 *
 * Created: 30-Oct-18 12:21:19
 *  Author: Daniel Geerts && Florian Molenaars
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "display.h"
#include "schedular.h"


const int trigPin = 0;		// Trigger		PD0
const int echoPin = 3;		// Echo			PD3
const int RED_LED = 4;		// Red LED		PD4
const int YELLOW_LED = 5;	// Yellow LED	PD5
const int GREEN_LED = 6;	// Green LED	PD6
const int LightSensor = 0;  // lightsensor  PA0

volatile uint16_t gv_counter; // 16 bit counter value
volatile uint8_t gv_echo; // a flag
volatile uint16_t light_sensitivity = -1; //temperature
volatile uint16_t distance; //distance of roller shutter
uint8_t distant_max = 65;	//max distant roller shutter
uint8_t distant_min = 5;	//min distant roller shutter
uint8_t light_min = 40;		//min light intensity
uint8_t light_max = 130;	//max light intensity

// setting up mode for arduino
typedef enum{ROLLING_UP= 0, ROLLING_DOWN = 1, WAITING = 2} mode_t;
	mode_t mode = WAITING;

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
void init_adc()
{
	// ref=Vcc, left adjust the result (8 bit resolution),
	// select channel 0 (PC0 = input)
	ADMUX = (1<<REFS0)|(1<<ADLAR);
	// enable the ADC & prescale = 128
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}


uint8_t get_adc_value()
{
	ADCSRA |= (1<<ADSC); // start conversion
	loop_until_bit_is_clear(ADCSRA, ADSC);
	return ADCH; // 8-bit resolution, left adjusted
}


void check_light()
{
	static int send_info = 0; // 0 - false, 1 - true
	uint8_t temp = light_sensitivity;
	light_sensitivity = get_adc_value();
	_delay_ms(100);
	
	if (light_sensitivity > 5)
	{
		//if (temp > 0) {
			//light_sensitivity = (light_sensitivity + temp) / 2;
		//}
		if (light_sensitivity >= light_max) {
			mode = ROLLING_DOWN;
		} else if (light_sensitivity <= light_min) {
			mode = ROLLING_UP;
		} else {
			mode = WAITING;
		}
		
	}
		if (send_info == 1)
		{
			// Send all information to Python 'De Centrale'
			send_info = 0;
		} 
		else 
		{
			send_info = 1;
		}					
}


void roll_down(void)
{
	while(distance <= distant_max){
		PORTD ^= (1 << YELLOW_LED);;
		
		if(distance >= distant_max || distance <= distant_min)
		{
			PORTD &= ~(1 << YELLOW_LED);
		}
		
		show_distance(distance);
		distance += 1;
		_delay_ms(1000);
	}
	
	_delay_ms(5000);
	mode = WAITING;
}	


void roll_up(void)
{
	while(distance >= distant_min){
		PORTD |= (1 << YELLOW_LED);
		
		if(distance >= distant_max || distance <= distant_min)
		{
			PORTD &= ~(1 << YELLOW_LED);
		}
		
		show_distance(distance);
		distance -= 1;
		_delay_ms(1000);
	}
	
	_delay_ms(5000);
	mode = WAITING;
}

void calc_distance() 
{
	gv_echo = BEGIN;	// Set gv_echo to BEGIN
	PORTD |= _BV(trigPin);	// Set trigPin to 1 -> send pulse
	_delay_us(12);			// Wait for pulse to complete
	PORTD &= ~(1<<trigPin);		// Clear PORTD (trigPin & LEDs)
	PORTD &= ~(1<<echoPin);
	_delay_ms(30);			// Wait to make sure the signal of the pulse has been returned to echo
	
	distance = calc_cm(gv_counter);
	show_distance(distance);
}


int main(void)
{
	init_ext_int();	 // Turn required external interrupts on
	init_timer();	 // Enable timer interrupts
	init_ports();    // Enable ports
	reset_display(); // Clear display
	init_adc();		 // Enable analog
	SCH_Init_T0();	 // Enable schedualr
	int t1 = SCH_Add_Task(check_light,0,100); // check light intensity
	sei();			 // Set interrupt flag
	_delay_ms(50);	// Make sure everything is initialized
	
	while(1)
	{
		SCH_Dispatch_Tasks();
		PORTD &= ~(1 << RED_LED);
		PORTD &= ~(1 << GREEN_LED);
		
		switch(mode)
		{
			case ROLLING_DOWN:
				roll_down();
				break;
			case ROLLING_UP:
				roll_up();
				break;
			case WAITING:
				calc_distance();
				break;
		}
		
		if (distance > distant_max)
		{
			PORTD |= (1<<GREEN_LED);		// Set green LED to 1
		}
		else if (distance < distant_min) 
		{
			PORTD |= (1<<RED_LED); // Set yellow LED to 1
		}	

		_delay_ms(500);
		
	}	
	SCH_Delete_Task(t1);
	cli();
}


ISR(INT1_vect)
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


