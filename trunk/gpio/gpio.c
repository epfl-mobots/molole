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

/* *x = (*x) & y */
#define atomic_and(x,y) do { __asm__ volatile ("and.w %[yy], [%[xx]], [%[xx]]": : [xx] "r" (x), [yy] "r"(y): "cc","memory"); } while(0)
/* *x = (*x) | y */
#define atomic_or(x,y) do { __asm__ volatile ("ior.w %[yy], [%[xx]], [%[xx]]" : : [xx] "r" (x), [yy] "r"(y): "cc","memory"); } while(0)

void gpio_set_dir(gpio gpio_id, int dir) {
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

	if(dir == GPIO_OUTPUT) {
		mask = ~(1 << pin);
		atomic_and(ptr, mask);	
	} else if(dir == GPIO_INPUT) {
		mask = 1 << pin;
		atomic_or(ptr, mask);
	} else
		ERROR(GPIO_INVALID_DIR, &dir);
}

void gpio_write(gpio gpio_id, bool value) {
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
	if(value == false) {
		mask = ~(1 << pin);
		atomic_and(ptr, mask);
	} else if(value == true) {
		mask = 1 << pin;
		atomic_or(ptr, mask);
	} else
		ERROR(GPIO_INVALID_VALUE, &value);
}

bool gpio_read(gpio gpio_id) {
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
	

