/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	
	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>,
	Philippe Retornaz <philippe dot retornaz at epfl dot ch>
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

//--------------------
// Usage documentation
//--------------------

/**
	\defgroup encoder Encoders
	
	Wrapper around Quadrature Encoder Interface or software implementation using \ref TIMER_2 or \ref TIMER_3 .
*/
/*@{*/

/** \file
	Implementation of the Encoders abstraction.
*/


//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "encoder.h"
#include "../error/error.h"

//-----------------------
// Structures definitions
//-----------------------

/** Data for the Quadrature Encoder Interface */
static struct
{
	long* pos;			/**< absolute position */
	int* speed;			/**< difference of last two absolutes positions (i.e. speed) */
} QEI_Encoder_Data;

static struct
{
	long* pos;			/**< absolute position */
	int* speed;			/**< difference of last two absolutes positions (i.e. speed) */
	int ic;				/**< Input Capture to use, must be one of \ref ic_identifiers */
} Software_Encoder_Data[2];


//-------------------
// Exported functions
//-------------------

void encoder_init(int type, int encoder_ic, long* pos, int* speed, int priority)
{
	// TODO: implement me
}

void encoder_step(int type)
{
	// TODO: implement me
	if (type == ENCODER_TIMER_2)
	{
	
	}
	else if (type == ENCODER_TIMER_2)
	{
	
	}
	else if (type == ENCODER_TYPE_HARD)
	{
	
	}
	else
	{
		ERROR(ENCODER_INVALID_TYPE, &type);
	}
}

// TODO: implement callbacks

/*@}*/
