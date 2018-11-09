/*
 * arduino_rollende_luiken.c
 *
 * Created: 30-Oct-18 12:21:19
 * Author: Daniël Geerts && Florian Molenaars
 * This code is written for the arduino 328p, it's code for operating a distance sensor,
 * temperature sensor and a light sensor. It has a schedular to opatere these sensors at the right time.
 * This can communicate with python software developed by the 'rollende luiken'.
 */ 

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "display.h"
#include "schedular.h"
#include "serial.h"

static int ID = 0;			// ID of device

// PD
const int trigPin = 2;		// Trigger		PD2
const int echoPin = 3;		// Echo			PD3
const int RED_LED = 4;		// Red LED		PD4
const int YELLOW_LED = 5;	// Yellow LED	PD5
const int GREEN_LED = 6;	// Green LED	PD6
const int lightSensor = 0;  // light sensor  PA0
const int temperature_sensor = 1;  // Temperature_sensor  PA1

volatile uint16_t gv_counter;		// 16 bit counter value
volatile uint8_t gv_echo;			// a flag
volatile double light_sensitivity = -1; //value of light sensivity
volatile uint16_t distance;			// distance of roller shutter
volatile double temperature;  //temperature in Celsius
// Default values of the maximal and minimal variables
uint8_t distant_max = 65;			//max distant roller shutter
uint8_t distant_min = 5;			//min distant roller shutter
uint8_t light_min = 15;		//min light intensity
uint16_t light_max = 65;	//max light intensity
uint16_t temperature_max = 30; // set max temperature
uint8_t temperature_min = 10;	// set minimum temperature

// setting up mode for arduino
typedef enum{ROLLING_UP = 0, ROLLING_DOWN = 1, WAITING = 2, STOP_ROLLING = 3} mode_t;
mode_t mode = WAITING;

// setting up modes for received requests from python
//	ASCII values: 35==# ; 36==$ ; 37==% ; 38==& ;
typedef enum{NONE = -1, ROLLER = 35, LIGHT = 36, TEMPERATURE = 37, COMMANDO = 38} data_mode;
data_mode d_modes = NONE;

void init_ports(void)
{
	// Set Trigger to OUTPUT, Echo to INPUT, Red LED to OUTPUT, Yellow LED to OUTPUT, Green LED to OUTPUT
	DDRD = (1<<trigPin) | (0<<echoPin) | (1<<RED_LED) | (1<<YELLOW_LED) | (1<<GREEN_LED);
	
	// Set Clock to OUTPUT, Strobe to OUTPUT, Data to OUTPUT
	DDRB = (1<<strobe) | (1<<clock) | (1<<data);
	
	sendCommand(0x89); // activate and set brightness to medium
}


void init_ext_int(void)
{
	// any change triggers ext interrupt 1
	EICRA = (1 << ISC10);
	EIMSK = (1 << INT1);
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

void check_received() {
	
	uint8_t r = uart_recieve();
	_delay_us(25);
	//uart_transmit_char(r);
	
	switch (r) {
		case ROLLER:
			d_modes = ROLLER;
		break;
		case LIGHT:
			d_modes = LIGHT;
		break;
		case TEMPERATURE:
			d_modes = TEMPERATURE;
		break;
		case COMMANDO:
			d_modes = COMMANDO;
		break;
		case NONE:
			d_modes = NONE;
		break;
		}
	
	uint8_t *data;
	if (d_modes != NONE) {
		switch (d_modes) {
			case ROLLER:
				data = insert_data_from_pyhton(ROLLER);
				distant_min = data[0];
				distant_max = data[1];
				free(data);
				d_modes = NONE;
			break;
			case LIGHT:	
				data = insert_data_from_pyhton(LIGHT);
				light_min = data[0];
				light_max = data[1];
				free(data);
				d_modes = NONE;
			break;
			case TEMPERATURE:
				ID = 7777;
				d_modes = NONE;
			break;
			case COMMANDO:
				_delay_us(25);
				
				uint8_t received = uart_recieve();
				_delay_us(50);
				//uart_transmit_char(received);
				if (received >= 48 && received <= 57) {	// ASCII 0-9
					ID = from_ascii_to_digit(received);
				} else if (received == 100) {		// ASCII d
					mode = ROLLING_DOWN;
				} else if (received == 117) {		// ASCII u
					mode = ROLLING_UP;
				} else if (received == 115) {		// ASCII s
					mode = STOP_ROLLING;
				}
								
				d_modes = NONE;
			break;
		}
	}	
	d_modes = NONE;
}

void send_info() {
	
	uart_transmit_char('#');
	uart_transmit_int(ID);
	uart_transmit_char('.');
	uart_transmit_int(distance);
	uart_transmit_char('#');
	
	uart_transmit_char('$');
	uart_transmit_int(ID);
	uart_transmit_char('.');
	uart_transmit_int(light_sensitivity);
	uart_transmit_char('$');
	
	uart_transmit_char('%');
	uart_transmit_int(ID);
	uart_transmit_char('.');
	uart_transmit_int(25);							// change to variable temperature
	uart_transmit_char('%');
}		

					


void roll_down(void)
{
	while(distance <= distant_max && mode != STOP_ROLLING)
	{
		PORTD ^= (1 << YELLOW_LED);
		if(distance >= distant_max || distance <= distant_min)
		{
			PORTD &= ~(1 << YELLOW_LED);
		}
		
		show_distance(distance);
		distance += 1;
		_delay_ms(2500);
	}
	mode = WAITING;
}	


void roll_up(void)
{
	while(distance >= distant_min && mode != STOP_ROLLING)
	{
		PORTD ^= (1 << YELLOW_LED);
		if(distance >= distant_max || distance <= distant_min)
		{
			PORTD &= ~(1 << YELLOW_LED);
		}
		
		show_distance(distance);
		distance -= 1;
		_delay_ms(2500);
	}
	mode = WAITING;
}


uint16_t calc_cm(uint16_t counter)
{
	// Min 2cm - Max 70cm
	uint16_t microSec = counter / 16;
	return (microSec/58.2);
}

void measure_distance() 
{
	gv_echo = BEGIN;			// Set gv_echo to BEGIN
	PORTD |= _BV(trigPin);		// Set trigPin to 1 -> send pulse
	_delay_us(12);				// Wait for pulse to complete
	PORTD &= ~(1<<trigPin);		// Clear PORTD (trigPin & LEDs)
	PORTD &= ~(1<<echoPin);
	_delay_ms(30);				// Wait to make sure the signal of the pulse has been returned to echo
	
	distance = calc_cm(gv_counter);
	show_distance(distance);
}

int main(void)
{	
	init_ext_int();		// Init external interrupts (INT1)
	SCH_Init_T0();		// Init schedular (Timer0)
	init_ports();		// Init ports
	init_adc();			// Init analog
	uart_init();		// Init uart (setup serial usb connection)
	
	int tasks[5];
	
	SCH_Init_T0();	 // Enable scheduler
	// check light intensity every 10ms * 100 = 1sec with zero delay
	tasks[0] = SCH_Add_Task(check_light,0,100);
	tasks[1] = SCH_Add_Task(check_temperature,0,200); // check temperature in celcius
	
	// check every 10ms * 2 = 20ms if python has updated some data
	// 1000ms/20ms = 50 pakketjes per seconde worden er verstuurd
	tasks[2] = SCH_Add_Task(check_received,0,5);				
	
	tasks[3] = SCH_Add_Task(send_info,0,100);	// 10ms * 100 = 1000ms = 1sec		
	
	sei();				// Set interrupt flag
	_delay_ms(50);	 // Make sure everything is initialized
	
	reset_display();	// Clear display
	
	while(1)
	{
		SCH_Dispatch_Tasks();
		PORTD &= ~(1 << GREEN_LED);			// Clear green LED (set to 0)
		PORTD &= ~(1 << RED_LED);			// Clear red LED (set to 0)
		
		switch(mode)
		{
			case ROLLING_DOWN:
				roll_down();
				break;
			case ROLLING_UP:
				roll_up();
				break;
			case STOP_ROLLING:
				show_distance(distance);
				PORTD &= ~(1 << YELLOW_LED);
				break;
			case WAITING:
				measure_distance();
				break;
		}
		
		if (distance >= distant_max)
		{
			PORTD |= (1<<GREEN_LED);		// Set green LED to 1
		}
		else if (distance <= distant_min) 
		{
			PORTD |= (1<<RED_LED);			// Set red LED to 1
		}

		_delay_ms(500);
		
	}
	
	for (int t = 0; t < tasks; t++)
	{
		SCH_Delete_Task(tasks[t]);
	}

	cli();
	
	return 0;
}


ISR(INT1_vect)
{	
	if (gv_echo == BEGIN)
	{
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


