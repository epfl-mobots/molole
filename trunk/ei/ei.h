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

#ifndef _MOLOLE_EI_H
#define _MOLOLE_EI_H

/** \addtogroup ei */
/*@{*/

/** \file
	External Interrupt wrapper definitions
*/

/** Errors External Interrupt can throw */
enum ei_errors
{
	EI_ERROR_BASE = 0x0D00,
	EI_INVALID_ID,						/**< The desired External Interrupt does not exists, must be one of \ref ei_identifiers. */
	EI_INVALID_POLARITY,			/**< The specified polarity is invalid, must be one of \ref ei_polarity */
};


/** External interrupt callback on interrupt */
typedef void (*ei_callback)(int ei_id, void * user_data);

/** Available External Interrupt signal polarity */
enum ei_polarity {
	EI_POSITIVE_EDGE = 0,
	EI_NEGATIVE_EDGE,
};

/** Identifiers of available External Interrupt. */
enum ei_identifiers {
	EI_MIN = 0,
#ifdef _INT0EP
	EI_0 = EI_MIN,
#ifdef _INT1EP
	EI_1,
#ifdef _INT2EP
	EI_2,
#ifdef _INT3EP
	EI_3,
#ifdef _INT4EP
	EI_4,
#ifdef _INT5EP
	EI_5,
#ifdef _INT6EP
	EI_6,
#ifdef _INT7EP
	EI_7,
#else
	EI_MAX = EI_6,
#endif
#else
	EI_MAX = EI_5,
#endif
#else
	EI_MAX = EI_4,
#endif
#else
	EI_MAX = EI_3,
#endif
#else
	EI_MAX = EI_2,
#endif
#else
	EI_MAX = EI_1,
#endif
#else
	EI_MAX = EI_0,
#endif
#else
	#error "No external Interrupt available"
#endif
};

// Functions, doc in the .c

void ei_init(int ei_id, int polarity, int priority);

void ei_disable(int ei_id);

void ei_enable(int ei_id, ei_callback callback, void * user_data);


/*@}*/


#endif

