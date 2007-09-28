/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	Copyright (C) 2006 - 2007 Florian Vaussard <Florian dot Vaussard at a3 dot epfl dot ch>
	Copyright (C) 2007 Stephane Magnenat <stephane at magnenat dot net>
	
	Mobots group http://mobots.epfl.ch
	Robotics system laboratory http://lsro.epfl.ch
	EPFL Ecole polytechnique federale de Lausanne: http://www.epfl.ch
	
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

/** \file
	\brief A wrapper around dsPIC33 Timers.
*/

#ifndef _MOLOLE_TIMERS_H
#define _MOLOLE_TIMERS_H

#include "../types/types.h"

/* Timers library defines */

/** Function errors */
enum timer_return_values
{
	TIMER_NO_ERROR							= 0,	/**< The function has completed without any error */
	TIMER_ERROR_SAMPLE_TIME_NOT_IN_RANGE	= 1,	/**< The required timing is not possible (see the Limits paragraph) above */
	TIMER_ERROR_ALREADY_IN_USE				= 2,	/**< The desired timer (one 16-bits or 2 16-bits timers for a 32-bits timer) is already in use ! */
	TIMER_ERROR_NOT_INITIALIZED				= 3,	/**< The desired timer is not initialized. Call timer_init() first. */
	TIMER_ERROR_INVALIDE_TIMER_ID			= 4,	/**< The desired timer is not available. */
	TIMER_ERROR_INVALIDE_UNIT				= 5,	/**< The unit parameter, passed to timer_init(), is not valid */
};

/** Source of clock for timers */
enum timer_clock_source
{
	TIMER_CLOCK_INTERNAL					= 0,	/**< Use the interal clock (Fcy). This is the default option. */
	TIMER_CLOCK_EXTERNAL					= 1,	/**< Use the external clock from pin T1CK (on the rising edge). */
};

/** Identifiers of available timers. Not all can be used concurrently */
enum timer_identifiers
{
	/* 16 bits timers */
	TIMER1									= 0,	/**< 16-bits Timer 1 */
	TIMER2									= 1,	/**< 16-bits Timer 2 */
	TIMER3									= 2,	/**< 16-bits Timer 3 */
	TIMER4									= 3,	/**< 16-bits Timer 4 */
	TIMER5									= 4,	/**< 16-bits Timer 5 */
	TIMER6									= 5,	/**< 16-bits Timer 6 */
	TIMER7									= 6,	/**< 16-bits Timer 7 */
	TIMER8									= 7,	/**< 16-bits Timer 8 */
	TIMER9									= 8,	/**< 16-bits Timer 9 */

	/* 32 bits timers */
	TIMER23									= 9,	/**< 32-bits timer formed of the Timer 2 + 3 */
	TIMER45									= 10,	/**< 32-bits timer formed of the Timer 4 + 5 */
	TIMER67									= 11,	/**< 32-bits timer formed of the Timer 6 + 7 */
	TIMER89									= 12,	/**< 32-bits timer formed of the Timer 8 + 9 */
};

/** Timer callback on interrupt */
typedef void(*timer_callback)(void);


// Functions, doc in the .c

int timer_init(int id, unsigned long int sample_time, int unit);

int timer_release(int id);

int timer_set_period(int id, unsigned long int sample_time, int unit);

bool timer_is_free(int id);

int timer_set_enabled(int id, bool enabled);

int timer_reset(int id);

int timer_set_clock_source(int id, int clock_source);

int timer_use_gated_time_accumulation(int id, bool enable);

int timer_enable_interrupt(int id, int priority, timer_callback callback);

int timer_disable_interrupt(int id);

#endif
