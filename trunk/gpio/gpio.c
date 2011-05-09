/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs

	Copyright (C) 2007--2011 Stephane Magnenat <stephane at magnenat dot net>,
	Philippe Retornaz <philippe dot retornaz at epfl dot ch>
	Mobots group (http://mobots.epfl.ch), Robotics system laboratory (http://lsro.epfl.ch)
	EPFL Ecole polytechnique federale de Lausanne (http://www.epfl.ch)

	See authors.txt for more details about other contributors.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, version 3 of the License.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include "gpio.h"
#include "../error/error.h"


/* (ds)PIC GPIO memory:
	TRISx
	PORTx
	LATx
*/

#define ODC_EXIST(p) ((defined _ODC## p ##0) || (defined _ODC## p ##1) || (defined _ODC## p ##2) || (defined _ODC## p ##3) || (defined _ODC## p ##4) || (defined _ODC## p ##5)  || (defined _ODC## p ##6) || (defined _ODC## p ##7) || (defined _ODC## p ##8) || (defined _ODC## p ##8) || (defined _ODC## p ##9) || (defined _ODC## p ##10) || (defined _ODC## p ##11) || (defined _ODC## p ##12) || (defined _ODC## p ##13) || (defined _ODC## p ##14) || (defined _ODC## p ##15))

/** 
	Set the OpenDrain functionality of a GPIO

	\param gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID()
	\param opendrain
			\ref true if must be configured as open-drain, false otherwise
*/

void gpio_set_opendrain(gpio gpio_id, int opendrain) {
	/* Since this is a configuration function, it's not time-critical so I can do it the "dumb" way
	 */
	int port, pin, mask;
	volatile unsigned int * ptr;
	pin = gpio_id & 0xF;
	port = gpio_id >> 4;
	
	if(!(gpio_id & 0xFFF0))	/* GPIO_NONE */
		return;
	
#if ODC_EXIST(A)
	if(port == (unsigned int) GPIO_PORTA) {
			ptr = (volatile unsigned int *) &ODCA;
	} else
#endif
#if ODC_EXIST(B)
	if(port == (unsigned int) GPIO_PORTB) {
			ptr = (volatile unsigned int *) &ODCB;
	} else 
#endif
#if ODC_EXIST(C)
	if(port == (unsigned int) GPIO_PORTC) {
			ptr = (volatile unsigned int *) &ODCC;
	} else 
#endif
#if ODC_EXIST(D)
	if(port == (unsigned int) GPIO_PORTD) {
			ptr = (volatile unsigned int *) &ODCD;
	} else 
#endif
#if ODC_EXIST(E)
	if(port == (unsigned int) GPIO_PORTE) {
			ptr = (volatile unsigned int *) &ODCE;	
	} else 
#endif
#if ODC_EXIST(F)
	if(port == (unsigned int) GPIO_PORTF) {
			ptr = (volatile unsigned int *) &ODCF;
	} else 
#endif
#if ODC_EXIST(G)
	if(port == (unsigned int) GPIO_PORTG) {
			ptr = (volatile unsigned int *) &ODCG;
	} else
#endif
	{
			ERROR(GPIO_INVALID_GPIO, &gpio_id);	
	}

	if(opendrain) {
		mask = 1 << pin;
		atomic_or(ptr, mask);
	} else {
		mask = ~(1 << pin);
		atomic_and(ptr, mask);
	}
}

/**
	Set the direction of a GPIO.
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID()
	\param	dir
			direction of the GPIO, either \ref GPIO_OUTPUT or \ref GPIO_INPUT .
*/
void gpio_set_dir(gpio gpio_id, int dir)
{
	int pin = gpio_id & 0xF;
	unsigned int mask;
	volatile unsigned int * ptr =  (volatile unsigned int *) (gpio_id >> 4);
	
/*
	I Cannot do the test since not every PIC has PORTA and PORTG 
	if(((ptr < GPIO_PORTA) || (ptr > GPIO_PORTG)) && ptr)
		ERROR(GPIO_INVALID_GPIO, &gpio_id);
*/	
	if(ptr == GPIO_NONE) 
		return;

	if(dir == GPIO_OUTPUT)
	{
		mask = ~(1 << pin);
		atomic_and(ptr, mask);
	}
	else if (dir == GPIO_INPUT)
	{
		mask = 1 << pin;
		atomic_or(ptr, mask);
	}
	else
		ERROR(GPIO_INVALID_DIR, &dir);
}

/**
	Set the value of a GPIO.
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID().
	\param	value
			\ref true for VCC, \ref false for GND.
*/
void gpio_write(gpio gpio_id, bool value)
{
	int pin = gpio_id & 0xF;
	unsigned int mask;
	volatile unsigned int * ptr = (volatile unsigned int *) (gpio_id >> 4);

/*
	I Cannot do the test since not every PIC has PORTA and PORTG 
	if(((ptr < GPIO_PORTA) || (ptr > GPIO_PORTG)) && ptr)
		ERROR(GPIO_INVALID_GPIO, &gpio_id);
*/
	if(ptr == GPIO_NONE) 
		return;

	ptr += 2;
	if(value == false)
	{
		mask = ~(1 << pin);
		atomic_and(ptr, mask);
	}
	else if(value == true)
	{
		mask = 1 << pin;
		atomic_or(ptr, mask);
	}
	else
		ERROR(GPIO_INVALID_VALUE, &value);
}

/**
	Get the value of a GPIO.
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID().
	\return
			\ref true for VCC, \ref false for GND.
*/
bool gpio_read(gpio gpio_id)
{
	int pin = gpio_id & 0xF;
	volatile unsigned int * ptr = (volatile unsigned int *) (gpio_id >> 4);

/*
	I Cannot do the test since not every PIC has PORTA and PORTG 
	if(((ptr < GPIO_PORTA) || (ptr > GPIO_PORTG)) && ptr)
		ERROR(GPIO_INVALID_GPIO, &gpio_id);
	
*/
	if(ptr == GPIO_NONE) 
		return false;
	
	ptr++;
	if(*ptr & (1 << pin))
		return true;
	else
		return false;
}


/**
	Write to a 16 bits GPIO port
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Take only in account the PORT and not the bit number.
	\param	value
			the value to write on the port
*/
void gpio_write_word(gpio gpio_id, unsigned int value) 
{
	volatile unsigned int * ptr = (volatile unsigned int *) (gpio_id >> 4);

	if(ptr == GPIO_NONE) 
		return;

	ptr += 2;

	*ptr = value;
}

/**
	Get the value of a GPIO port
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Take only in account the PORT and not the bit number.
	\return
			Return the value read on the port
*/
unsigned int gpio_read_word(gpio gpio_id) 
{
	volatile unsigned int * ptr = (volatile unsigned int *) (gpio_id >> 4);
	
	if(ptr == GPIO_NONE)
		return 0;
	ptr++;
	
	return *ptr;
}

/**
	Set the direction of a GPIO.
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Take only in account the PORT and not the bit number.
	\param	dir
			direction of the GPIO, either \ref GPIO_OUTPUT or \ref GPIO_INPUT .
*/
void gpio_set_dir_word(gpio gpio_id, int dir)
{
	volatile unsigned int * ptr =  (volatile unsigned int *) (gpio_id >> 4);
	
	if(ptr == GPIO_NONE) 
		return;

	if(dir == GPIO_OUTPUT)
	{
		*ptr = 0;
	}
	else if (dir == GPIO_INPUT)
	{
		*ptr = 0xFFFF;
	}
	else
		ERROR(GPIO_INVALID_DIR, &dir);

}

/**
	Set the OpenDrain functionality of a GPIO
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Use  \ref GPIO_LOW_BYTE or \ref  GPIO_HIGH_BYTE as pin number to select which byte is used.
	\param opendrain
			\ref true if must be configured as open-drain, false otherwise
*/

void gpio_set_opendrain_word(gpio gpio_id, int opendrain) {
	int i;
	/* Do it the dumb and non-atomic way */
	for(i = 0; i < 16; i++) 
		gpio_set_opendrain((gpio_id & 0xFFF0) | i, opendrain);
}

/**
	Write to a 8 bits GPIO port
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Use  \ref GPIO_LOW_BYTE or \ref  GPIO_HIGH_BYTE as pin number to select which byte is used.
	\param	value
			the value to write on the port
*/
void gpio_write_byte(gpio gpio_id, unsigned char value) 
{
	int pin = gpio_id & 0xF;
	volatile unsigned char * ptr = (volatile unsigned char *) (gpio_id >> 4);

	if(ptr == GPIO_NONE) 
		return;

	ptr += 4;

	if(pin > 7) 
		ptr++;	

	*ptr = value;
}

/**
	Get the value of a GPIO port
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Use  \ref GPIO_LOW_BYTE or \ref  GPIO_HIGH_BYTE as pin number to select which byte is used.
	\return
			Return the value read on the port
*/
unsigned char gpio_read_byte(gpio gpio_id) 
{
	int pin = gpio_id & 0xF;
	volatile unsigned char * ptr = (volatile unsigned char *) (gpio_id >> 4);
	
	if(ptr == GPIO_NONE)
		return 0;

	ptr += 2;

	if(pin > 7)
		ptr++;

	return *ptr;
}

/**
	Set the direction of a GPIO.
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Use  \ref GPIO_LOW_BYTE or \ref  GPIO_HIGH_BYTE as pin number to select which byte is used.
	\param	dir
			direction of the GPIO, either \ref GPIO_OUTPUT or \ref GPIO_INPUT .
*/
void gpio_set_dir_byte(gpio gpio_id, int dir)
{
	int pin = gpio_id & 0xF;
	volatile unsigned char * ptr =  (volatile unsigned char *) (gpio_id >> 4);
	
	if(ptr == GPIO_NONE) 
		return;

	if(pin > 7)
		ptr++;

	if(dir == GPIO_OUTPUT)
	{
		*ptr = 0;
	}
	else if (dir == GPIO_INPUT)
	{
		*ptr = 0xFF;
	}
	else
		ERROR(GPIO_INVALID_DIR, &dir);

}

/**
	Set the OpenDrain functionality of a GPIO
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Use  \ref GPIO_LOW_BYTE or \ref  GPIO_HIGH_BYTE as pin number to select which byte is used.
	\param opendrain
			\ref true if must be configured as open-drain, false otherwise
*/
void gpio_set_opendrain_byte(gpio gpio_id, int opendrain) {
	int i;
	int start = ((gpio_id & 0xF) > 7) ? 8 : 0;

	/* Do it the dumb and non-atomic way */

	for(i = start; i < start + 8; i++) 
		gpio_set_opendrain((gpio_id & 0xFFF0) | i, opendrain);
}

