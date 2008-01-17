/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	
	Copyright (C) 2007 - 2008 Stephane Magnenat <stephane at magnenat dot net>,
	Mobots group (http://mobots.epfl.ch), Robotics system laboratory (http://lsro.epfl.ch)
	EPFL Ecole polytechnique federale de Lausanne (http://www.epfl.ch)
	
	Copyright (C) 2007 Florian Vaussard <Florian dot Vaussard at a3 dot epfl dot ch>
	
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

#ifndef _MOLOLE_TIMERS_H
#define _MOLOLE_TIMERS_H

#include "../types/types.h"

/** \addtogroup timer */
/*@{*/

/** \file
	\brief A wrapper around dsPIC33 Timers.
*/

// Defines

/** Errors timer can throw */
enum timer_errors
{
	TIMER_ERROR_BASE = 0x0100,
	TIMER_ERROR_SAMPLE_TIME_NOT_IN_RANGE,	/**< The required timing is not possible (see the Limits paragraph) above */
	TIMER_ERROR_ALREADY_IN_USE,				/**< The specified timer (one 16-bits or 2 16-bits timers for a 32-bits timer) is already in use ! */
	TIMER_ERROR_NOT_INITIALIZED,			/**< The specified timer is not initialized. Call timer_init() first. */
	TIMER_ERROR_INVALID_TIMER_ID,			/**< The specified timer is not one of timer_identifiers. */
	TIMER_ERROR_INVALID_UNIT,				/**< The specified unit parameter, passed to timer_init(), is not valid */
	TIMER_ERROR_INVALID_CLOCK_SOURCE,		/**< The specified clock source is not one of timer_clock_source. */
};

/** Source of clock for timers */
enum timer_clock_source
{
	TIMER_CLOCK_INTERNAL			= 0,	/**< Use the interal clock (Fcy). This is the default option. */
	TIMER_CLOCK_EXTERNAL			= 1,	/**< Use the external clock from pin T1CK (on the rising edge). */
};

/** Identifiers of available timers. Not all can be used concurrently */
enum timer_identifiers
{
	/* 16 bits timers */
	TIMER_1							= 0,	/**< 16-bits Timer 1 */
	TIMER_2							= 1,	/**< 16-bits Timer 2 */
	TIMER_3							= 2,	/**< 16-bits Timer 3 */
	TIMER_4							= 3,	/**< 16-bits Timer 4 */
	TIMER_5							= 4,	/**< 16-bits Timer 5 */
	TIMER_6							= 5,	/**< 16-bits Timer 6 */
	TIMER_7							= 6,	/**< 16-bits Timer 7 */
	TIMER_8							= 7,	/**< 16-bits Timer 8 */
	TIMER_9							= 8,	/**< 16-bits Timer 9 */

	/* 32 bits timers */
	TIMER_23						= 9,	/**< 32-bits timer formed of the Timer 2 + 3 */
	TIMER_45						= 10,	/**< 32-bits timer formed of the Timer 4 + 5 */
	TIMER_67						= 11,	/**< 32-bits timer formed of the Timer 6 + 7 */
	TIMER_89						= 12,	/**< 32-bits timer formed of the Timer 8 + 9 */
};

/** Timer callback on interrupt */
typedef void(*timer_callback)(int timer_id);


// Functions, doc in the .c

void timer_init(int id, unsigned long int sample_time, int unit);

void timer_release(int id);

void timer_set_period(int id, unsigned long int sample_time, int unit);

bool timer_is_free(int id);

void timer_enable(int id);

void timer_disable(int id);

void timer_set_enabled(int id, bool enabled);

void timer_set_value(int id, unsigned long value);

unsigned long timer_get_value(int id);

void timer_set_clock_source(int id, int clock_source);

void timer_use_gated_time_accumulation(int id, bool enable);

void timer_enable_interrupt(int id, timer_callback callback, int priority);

void timer_disable_interrupt(int id);

/*@}*/

#endif
