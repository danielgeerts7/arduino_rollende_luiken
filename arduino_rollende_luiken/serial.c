/*
 * serial.c
 *
 * Created: 06-Nov-18 14:29:58
 *  Author: Daniel Geerts
 */

#include "serial.h"

void uart_init(void) 
{
	UBRR0H = 0;
	UBRR0L = UBBRVAL;
	UCSR0A = 0;
	UCSR0B = _BV(TXEN0) | _BV(RXEN0);
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}

// function to receive data
unsigned char uart_recieve (void)
{
	while(!(UCSR0A) & (1<<RXC0));                  // wait while data is being received
	return UDR0;                                   // return 8-bit data
}

// function to send data
void uart_transmit_char (unsigned char data)	// send as char, example: data=126 -> python receives '~'
{
	while (!( UCSR0A & (1<<UDRE0)));            // wait while register is free
	UDR0 = data;
}

void uart_transmit_int(unsigned int data)
{		// send as int, example: data=3 -> python receives '3'
	float honderdtal = data / 100;
	if (honderdtal >= 1) 
	{
		honderdtal += 48;
		uart_transmit_char(honderdtal);
	}
	
	float tiental = data % 100;
	tiental = tiental / 10;
	if (tiental >= 1) 
	{
		tiental += 48;
		uart_transmit_char(tiental);
	}
	
	int rest = data % 10;
	rest += 48;
	uart_transmit_char(rest);
}

int from_ascii_to_digit(char a)
{
	return a - 48;
}

int calc_to_the_power(int nr_scale, int exponent)
{
	int power = 1;

	for (int i = 0; i < exponent; ++i) 
	{
		power *= nr_scale;
	}

	return(power);
}

uint16_t sum_array_elements(uint8_t size, uint8_t list[])
{
	uint16_t result = 0;
	
	for (int i = 0; i < size; i++)
	{
		uint16_t temp = calc_to_the_power(10, size-i-1);
		result += list[i] * temp;
	}
	return result;
}

uint8_t* insert_data_from_pyhton(uint8_t from_sensor) 
{
	uint8_t running = 1;
	uint8_t isMin = 1;
	uint8_t mincount = 0;
	uint8_t maxcount = 0;
	uint8_t result = 0;
	uint8_t min[3];
	uint8_t max[3];
	uint8_t received = 0;
	
	while (running == 1 && from_sensor != -1) 
	{
		received = 0;
		received = uart_recieve();
		_delay_us(15);
		//uart_transmit_char(received);
		
		if (received != from_sensor) 
		{
			if (received >= 48 && received <= 57) // ASCII 0-9
			{			
				if (isMin == 1) 
				{
					result = from_ascii_to_digit(received);
					min[mincount] = result;
					mincount++;
				} 
				else 
				{
					result = from_ascii_to_digit(received);
					max[maxcount] = result;
					maxcount++;
				}
			}
			if (received == 46 && isMin == 1) // ASCII .
			{			
				isMin = 0;
			}
		}
		if (received == from_sensor && isMin == 0) 
		{
			running = 0;
		}
	}
	
	uint8_t minimal = sum_array_elements(mincount, min);
	uint8_t maximal = sum_array_elements(maxcount, max);
	
	uint8_t* t = malloc(2);
	t[0] = minimal;
	t[1] = maximal;
	
	return t;
}
