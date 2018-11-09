/*
 * arduino_rollende_luiken.c
 *
 * Created: 30-Oct-18 12:21:19
 * Author: Daniël Geerts && Florian Molenaars
 * This code is written for the arduino 328p, it's code for operating a distance sensor,
 * temperature sensor and a light sensor. It has a schedular to opatere these sensors at the right time.
 * This can communicate with python software developed by the 'rollende luiken'.
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
const int lightSensor = 0;  // light sensor  PA0
const int temperature_sensor = 1;  // Temperature_sensor  PA1

volatile uint16_t gv_counter; // 16 bit counter value
volatile uint8_t gv_echo; // a flag
volatile double light_sensitivity = -1; //value of light sensivity
volatile uint16_t distance; //distance of roller shutter
volatile double temperature;  //temperature in Celsius
uint8_t distant_max = 65;	//max distant roller shutter
uint8_t distant_min = 5;	//min distant roller shutter
uint8_t light_min = 15;		//min light intensity
uint16_t light_max = 65;	//max light intensity
uint16_t temperature_max = 30; // set max temperature
uint8_t temperature_min = 10;	// set minimum temperature

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


double calc_temperature(double adc_value)
{
	adc_value = adc_value * (5.0/1023);		// calculate value to volt
	adc_value = adc_value - 0.5;			// convert to celcius
	adc_value = adc_value * 100;			
	return adc_value;
}


double calc_ligth(double light)
{
	light = light / 1023;			// calculate to volt
	light = light * 100;			// to percentage
	return light;
}


void init_adc()
{
	// turn on channels
	ADMUX = (1<<REFS0);
	 //enable the ADC & prescale = 128
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}
	

uint16_t get_adc_value(uint8_t ADC_port)
{
	// Clear the previously read channel.
	ADC_port &= 0b0000111;
	ADMUX = (ADMUX & 0xF8) | ADC_port;
	ADCSRA |= (1<<ADSC); // start conversion
	while(ADCSRA & (1<<ADSC));
	//loop_until_bit_is_clear(ADCSRA, ADSC);
	return ADC; // 8-bit resolution, left adjusted
}


void check_temperature()
{	
	double temp = get_adc_value(temperature_sensor); // get value from adc port
	temperature = calc_temperature(temp);			 // calculate the temperature	
							 
	if (temperature >= temperature_max)				 // compare temperature to decide if
	{												 // the rolling shutter needs to roll down or roll up
		mode = ROLLING_DOWN;
	}
	else if (temperature <= temperature_min)
	{
		mode = ROLLING_UP;
	} 
	else 
	{
		mode = WAITING;
	}
	
	_delay_ms(100);
	return temperature;
}


void check_light()
{
	static int send_info = 0; // 0 - false, 1 - true
	uint8_t temp = light_sensitivity;
	light_sensitivity = get_adc_value(lightSensor);
	light_sensitivity = calc_ligth(light_sensitivity);

	_delay_ms(100);
	
	if (light_sensitivity > 5)
	{
		if (light_sensitivity >= light_max)
		{
			mode = ROLLING_DOWN;
		}
		else if (light_sensitivity <= light_min) 
		{
			mode = ROLLING_UP;	
		}
		 else 
		{
			mode = WAITING;
		}
		
	}
					
}


void roll_down(void)
{
	while(distance <= distant_max)
	{
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
	while(distance >= distant_min)
	{
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
	SCH_Init_T0();	 // Enable scheduler
	int t1 = SCH_Add_Task(check_light,0,300); // check light intensity	
	int t2 = SCH_Add_Task(check_temperature,0,400); // check temperature in celcius
	sei();			 // Set interrupt flag
	_delay_ms(50);	 // Make sure everything is initialized
	
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
	SCH_Delete_Task(t2);
	cli();
}


ISR(INT1_vect)
{	
	if (gv_echo == BEGIN) {
		TCNT1 = 0;
		TCCR1B = _BV(CS10);
		gv_echo = END;
	} 
	else
	{
		TCCR1B = 0;
		gv_counter = TCNT1;
	}
}


