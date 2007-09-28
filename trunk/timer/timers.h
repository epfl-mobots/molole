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
	
	<h2>Introduction</h2>
	
	This library allows the user to easily configure and use the 9 16-bits timers of the
	dsPIC33 microcontroller's family. The 16-bits timers can also be used 2 by 2
	(TIMER2 + TIMER3, TIMER4 + TIMER5, TIMER6 + TIMER7, TIMER8 + TIMER9), which allows up to
	4 32-bits timers. The management of the 16-bits / 32-bits timers is totally transparent,
	preventing the user to configure an already in-use timer.
	
	<h2>Limits</h2>
	
	With a cycle frequency of 40 MHz, the maximum reachable timings are as follow:
	<ul>
		<li>16-bits timers : 419 ms</li>
		<li>32-bits timers : 27'487 s</li>
	</ul>
	
	<h2>Timer set-up</h2>
	
	You can configure one of the 16-bits timer (TIMER1 -> TIMER9), or one of the 32-bits timer (TIMER23 -> TIMER89).
	The configuration process is: initializing the timer with the desired timing, optionally defining an interrupt routine,
	and finally launch the timer.
	
	\code
	if (timer_init(TIMER1, 400, 6)!=TIMER_NO_ERROR)		// 400 us
		return 1;		// error !!!
	
	timer_enable_interrupt(TIMER1, 1, int_timer1);		// int_timer1 is a void ...(void) function
	
	timer_set_enabled(TIMER1, true);					// start
	\endcode
	
	The return value of the timer_init() function determines if the initialization process is correctly completed.
		You can test if a timer is available, with the function timer_is_free().
	
	<h2>Known bugs on MPLAB SIM</h2>
	
	The following problems occurs when using the MPLAB SIM simulator (the library is working fine on the real microcontrollers):
	<ul>
		<li>TIMER6: timer_enable_interrupt() don't modify the IEC2.T6IE register ! So the interruption is never fired...</li>
		<li>TIMER9: everything is ok (register IEC3.T9IE = 1), the timer normally counts, but the flag IFS3bits.T9IF is never set !
			The interruption is never fired ! If we set the flag by hand, we enter in the interruption...</li>
		<li>TIMER67 and TIMER89: same problem as TIMER9</li>
	</ul>

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


#endif // _TIMERS_H
