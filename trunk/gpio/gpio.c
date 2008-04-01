/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	
	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>,
	Mobots group (http://mobots.epfl.ch), Robotics system laboratory (http://lsro.epfl.ch)
	EPFL Ecole polytechnique federale de Lausanne (http://www.epfl.ch)
	
	See AUTHORS for more details about other contributors.
	
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	
	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <p33fxxxx.h>

#include "gpio.h"
#include "../error/error.h"


/* (ds)PIC GPIO memory:
	TRISx
	PORTx
	LATx
*/

/** Atomic and operation to prevent race conditions inside interrupts: *x = (*x) & y */
#define atomic_and(x,y) do { __asm__ volatile ("and.w %[yy], [%[xx]], [%[xx]]": : [xx] "r" (x), [yy] "r"(y): "cc","memory"); } while(0)
/** Atomic or operation to prevent race conditions inside interrupts: *x = (*x) | y */
#define atomic_or(x,y) do { __asm__ volatile ("ior.w %[yy], [%[xx]], [%[xx]]" : : [xx] "r" (x), [yy] "r"(y): "cc","memory"); } while(0)

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
	Write to a 8 bits GPIO port
	
	\param	gpio
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Take only in account the PORT and if the bit number > 7 the high byte of the port is read else the low byte is used.
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
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Take only in account the PORT and if the bit number > 7 the high byte of the port is read else the low byte is used.
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
			identifier of the GPIO, must be created by GPIO_MAKE_ID(). Take only in account the PORT and if the bit number > 7 the high byte of the port is read else the low byte is used.
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



