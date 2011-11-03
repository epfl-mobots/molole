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

#ifndef _MOLOLE_GPIO_H
#define _MOLOLE_GPIO_H

#include "../types/types.h"

/** \addtogroup gpio */
/*@{*/

/** \file
	\brief A wrapper around dsPIC33 GPIOs.
*/

// Defines
                                                                                                                                                                                                        
/** Errors GPIO can throw */
enum gpio_errors
{
	GPIO_ERROR_BASE = 0x0900,
	GPIO_INVALID_GPIO,			/**< The specified GPIO doesn't exist */
	GPIO_INVALID_DIR,			/**< The specified direction doesn't exist */
	GPIO_INVALID_VALUE,			/**< The specified value is not true of false */
};

/** TRIS configuration mode */
enum gpio_dir
{
	GPIO_OUTPUT = 0,			/**< TRIS configured as output */
	GPIO_INPUT  = 1,			/**< TRIS configured as input */
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

/** GPIO Pin number, to use with \ref GPIO_MAKE_ID 
 * The association pin number 0 == value 0 etc ... is alway guaranteed
 */
enum gpio_pin_number
{
	/*! */	GPIO_PIN_0 = 0,
	/*! */	GPIO_PIN_1,
	/*! */	GPIO_PIN_2,
	/*! */	GPIO_PIN_3,
	/*! */	GPIO_PIN_4,
	/*! */	GPIO_PIN_5,
	/*! */	GPIO_PIN_6,
	/*! */	GPIO_PIN_7,
	/*! */	GPIO_PIN_8,
	/*! */	GPIO_PIN_9,
	/*! */	GPIO_PIN_10,
	/*! */	GPIO_PIN_11,
	/*! */	GPIO_PIN_12,
	/*! */	GPIO_PIN_13,
	/*! */	GPIO_PIN_14,
	/*! */	GPIO_PIN_15,
};

/** GPIO Byte referenced, to use with \ref GPIO_MAKE_ID and the _byte version of the gpio access function */
/*! */	#define GPIO_LOW_BYTE 0
/*! */	#define GPIO_HIGH_BYTE 8

/** GPIO Word tag, to use with \ref GPIO_MAKE_ID and the _word version of the gpio access */
#define GPIO_WORD 0

/** Compute the GPIO number from the port and pin number. */
#define GPIO_MAKE_ID(port, num) ((gpio) ((((unsigned int)port) << 4) | ((num) & 0xF))) 

/** GPIO identifier */
typedef unsigned int gpio;


// Functions, doc in the .c

void gpio_set_opendrain(gpio gpio_id, int opendrain);
void gpio_set_dir(gpio gpio_id, int dir);
void gpio_write(gpio gpio_id, bool value);
bool gpio_read(gpio gpio_id);

void gpio_set_opendrain_byte(gpio gpio_id, int opendrain);
void gpio_set_dir_byte(gpio gpio_id, int dir);
void gpio_write_byte(gpio gpio_id, unsigned char value);
unsigned char gpio_read_byte(gpio gpio_id);

void gpio_set_opendrain_word(gpio gpio_id, int opendrain);
void gpio_set_dir_word(gpio gpio_id, int dir);
unsigned int gpio_read_word(gpio gpio_id);
void gpio_write_word(gpio gpio_id, unsigned int value);

/*@}*/

#endif
