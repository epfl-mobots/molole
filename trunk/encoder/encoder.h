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

#ifndef _MOLOLE_ENCODER_H
#define _MOLOLE_ENCODER_H

#include "../types/types.h"
#include "../gpio/gpio.h"

/** \addtogroup encoder */
/*@{*/

/** \file
	Encoders abstraction
*/

// Defines

/** Errors Input Capture can throw */
enum encoder_errors
{
	ENCODER_ERROR_BASE = 0x0800,
	ENCODER_INVALID_TYPE,				/**< The specified encoder type is invalid, must be one of \ref encoder_type */
	ENCODER_INVALID_MODE,				/**< The specified encoder speed is invalid, must be one of \ref encoder_mode */
};


/** Type of the encoder; might either be hard (one per dsPIC) or soft (one per 16 bits timer) */
enum encoder_type
{
	ENCODER_TIMER_1 = 0,
	ENCODER_TIMER_2,
	ENCODER_TIMER_3,
	ENCODER_TIMER_4,
	ENCODER_TIMER_5,
	ENCODER_TIMER_6,
	ENCODER_TIMER_7,
	ENCODER_TIMER_8,
	ENCODER_TIMER_9,
	ENCODER_TYPE_HARD,		/**< encoder uses Quadrature Encoder Interface */
};

/** Direction of the encoder; might either be normal, or reverse */
enum encoder_direction
{
	ENCODER_DIR_NORMAL = 0,
	ENCODER_DIR_REVERSE = 1,
};

/** Decoding mode; Either 4x mode or 2x */
enum encoder_mode
{
	ENCODER_MODE_X2 = 0,
	ENCODER_MODE_X4 = 1,
};

// Functions, doc in the .c

void encoder_init(int type, int encoder_ic, long* pos, int* speed, int direction, gpio gpio_dir, gpio gpio_speed, int decoding_mode, int priority);

void encoder_step(int type);

long encoder_get_position(int type);

void encoder_reset(int type);

/*@}*/

#endif
