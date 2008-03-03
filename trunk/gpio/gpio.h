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

#ifndef _MOLOLE_GPIO_H
#define _MOLOLE_GPIO_H

#include "../types/types.h"

/** \addtogroup gpio */
/*@{*/

/** \file
	GPIO abstraction
*/

// Defines

/** Errors GPIO can throw */
enum gpio_errors
{
	GPIO_ERROR_BASE = 0x0900,
	GPIO_INVALID_GPIO,				/**< The specified GPIO doesn't exist */
	GPIO_INVALID_DIR,				/**< The specified direction doesn't exist */
	GPIO_INVALID_VALUE,		  		/**< The specified value is not true of false */
};

/** TRIS configuration mode */
enum gpio_dir
{
	GPIO_OUTPUT = 0,		/**< TRIS configured as output */
	GPIO_INPUT  = 1,		/**< TRIS configured as input */
};

/** GPIO port number, to use with \ref GPIO_MAKE_ID */
#define GPIO_PORTA (&TRISA)
#define GPIO_PORTB (&TRISB)
#define GPIO_PORTC (&TRISC)
#define GPIO_PORTD (&TRISD)
#define GPIO_PORTE (&TRISE)
#define GPIO_PORTF (&TRISF)
#define GPIO_PORTG (&TRISG)
#define GPIO_NONE (0)

/** Compute the GPIO number from the port and pin number. */
#define GPIO_MAKE_ID(port, num) ((gpio) ((((unsigned int)port) << 4) | ((num) & 0xF))) 

/** GPIO identifier */
typedef unsigned int gpio;


// Functions, doc in the .c

void gpio_set_dir(gpio gpio_id, int dir);
void gpio_write(gpio gpio_id, bool value);
bool gpio_read(gpio gpio_id);

/*@}*/

#endif
