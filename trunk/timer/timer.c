/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	Copyright (C) 2006 - 2007 Florian Vaussard <Florian dot Vaussard at a3 dot epfl dot ch>
	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>
	
	Copyright (C) 2004-2008 Mobots group http://mobots.epfl.ch
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

//--------------------
// Usage documentation
//--------------------

/**

\defgroup timer Timer

A wrapper around dsPIC33 Timers.

\section Introduction

This module allows the programmer configure and use the 9 16-bits timers of the
dsPIC33 microcontroller's family in a convenient way.
Some 16-bits timers can also be used 2 by 2
(TIMER2 + TIMER3, TIMER4 + TIMER5, TIMER6 + TIMER7, TIMER8 + TIMER9), which allows up to
4 32-bits timers. The management of the 16-bits / 32-bits timers is totally transparent,
preventing the user to configure an already in-use timer.

\section Limits

With a cycle frequency of 40 MHz, the maximum reachable timings are as follow:
- 16-bits timers : 419 ms
- 32-bits timers : 27'487 s

\section Usage

You can configure one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9), or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
The configuration process is: initializing the timer with the desired timing, optionally defining an interrupt routine,
and finally launch the timer.

\code
timer_init(TIMER1, 400, 6)						// 400 us

timer_enable_interrupt(TIMER1, 1, int_timer1);	// int_timer1 is a void ...(void) function

timer_set_enabled(TIMER1, true);				// start
\endcode

You can test if a timer is available with the function timer_is_free().

All functions cast errors using the \ref error "error reporting mechanism"

\section Bugs Known bugs

\subsection SIM MPLAB SIM

The following problems occurs when using the MPLAB SIM simulator (the library is working fine on the real microcontrollers):
- \ref TIMER6: timer_enable_interrupt() don't modify the IEC2.T6IE register ! So the interruption is never fired...
- \ref TIMER9: everything is ok (register IEC3.T9IE = 1), the timer normally counts, but the flag IFS3bits.T9IF is never set !
	The interruption is never fired ! If we set the flag by hand, we enter in the interruption...
- \ref TIMER67 and \ref TIMER89: same problem as TIMER9

*/
/*@{*/

/** \file
	Implementation of the wrapper around dsPIC33 Timers.
*/

//---------
// Includes
//---------

#include <p33fxxxx.h>

#include "timer.h"
#include "../clock/clock.h"
#include "../error/error.h"

//------------
// Definitions
//------------

/** m_set_32_bits_mode parameter; Enable 16-bits timer mode */
#define TIMER_16B_MODE						0
/** m_set_32_bits_mode parameter; Enable 32-bits timer mode */
#define TIMER_32B_MODE						1

/** Maximum reachable timing with a 16-bits timer; The 256 factor is the maximum prescaler of the timer. */
#define TIMER_16B_MAX_TIME_NS				((unsigned long long)clock_get_cycle_duration() * 0x0000ffff00ULL)
/** Maximum reachable timing with a 32-bits timer; The 256 factor is the maximum prescaler of the timer. */
#define TIMER_32B_MAX_TIME_NS				((unsigned long long)clock_get_cycle_duration() * 0xffffffff00ULL)

//-----------------------
// Structures definitions
//-----------------------

/** The structs for timer implementation data */
static struct
{
	timer_callback callback; /**< function to call upon interrupt, 0 if none */
	unsigned is_free:1; /**< true if timer is available for allocation */
	unsigned is_initialized:1; /**< true if timer can receive command */
	unsigned is_32bits:1; /**< true if timer is 32 bits */
} Timer_Data[9] =
{
	{ 0, 1, 0, 0},
	{ 0, 1, 0, 0},
	{ 0, 1, 0, 0},
	{ 0, 1, 0, 0},
	{ 0, 1, 0, 0},
	{ 0, 1, 0, 0},
	{ 0, 1, 0, 0},
	{ 0, 1, 0, 0},
	{ 0, 1, 0, 0}
};

/** Return the index of timers for a specific id, assumed to be in correct range */
int timer_id_to_index(int id)
{
	if (id <= TIMER9)
		return id;
	else
		return (id - TIMER23) * 2 + 1;
}

/** Return wether a timer is 32 bits for a specific id, assumed to be in correct range */
bool timer_id_to_32bits(int id)
{
	return id > TIMER9;
}


//-------------------------------
// Internal functions, prototypes
//-------------------------------

static void m_set_32bits_mode(int id, char mode);

static void m_set_prescaler(int id, unsigned int prescaler);

static void m_set_period_16b(int id, unsigned short period);

static void m_set_period_32b(int id, unsigned long period);


//-------------------
// Exported functions
//-------------------

/**
	Initialize a timer, prior to any use.
	
	This function initializes the timer, defines its period and reserves it, avoiding someone else to use it.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89)
	\param 	arg_sample_time
			The period of the timer, expressed in the unit defined by the \e unit parameter
	\param 	unit
			Time base of the \e sample_time parameter. Possible values are:
			- 0 : second
			- 3 : millisecond
			- 6 : microsecond
			- 9 : nanosecond
	
	\note	Use timer_release() when you don't need a timer anymore, so that someone else can use it !
*/
void timer_init(int id, unsigned long int arg_sample_time, int unit)
{
	int index;
	
	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id)
		
	// test that the timer is free
	if (!Timer_Data[timer_id_to_index(id)].is_free)
		ERROR(TIMER_ERROR_ALREADY_IN_USE, &id)
	
	// init ok !
	index = timer_id_to_index(id);
	Timer_Data[index].is_initialized = 1;
	Timer_Data[index].is_free = 0;
	if (timer_id_to_32bits(id))
	{
		// 32 bits timers consume two slots
		Timer_Data[index + 1].is_free = 0;
		Timer_Data[index].is_32bits = 1;
	}
	else
	{
		Timer_Data[index].is_32bits = 0;
	}
	
	// disable timer interrupt
	timer_disable_interrupt(id);

	// set the period
	timer_set_period(id, arg_sample_time, unit);

	// By default, set to internal clock source, without gated time accumulation
	timer_set_clock_source(id, TIMER_CLOCK_INTERNAL);
	timer_use_gated_time_accumulation(id, false);	
}


/**
	Set the period of a timer.
	
	The timer must already be initialized with timer_init() !
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89)
	\param 	sample_time
			The period of the timer, expressed in the unit defined by the \e unit parameter
	\param 	unit
			Time base of the \e sample_time parameter. Possible values are:
			- 0 : second
			- 3 : millisecond
			- 6 : microsecond
			- 9 : nanosecond
	
	\note	Automatically reset the timer's counter.
*/
void timer_set_period(int id, unsigned long int sample_time, int unit)
{
	unsigned long long real_sample_time = 0;
	unsigned long long period = 0;
	unsigned long prescaler_value[4] = {1, 8, 64, 256};
	
	unsigned int prescaler = 0;
	int i;

	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id)
		
	// is the timer initialized ?
	if (!Timer_Data[timer_id_to_index(id)].is_initialized)
		ERROR(TIMER_ERROR_NOT_INITIALIZED, &id)
		
	// only unit 0,3,6,9 ok
	if (!(unit == 0 || unit == 3 || unit == 6 || unit == 9))
		ERROR(TIMER_ERROR_INVALID_UNIT, &unit)
	
	// compute sample time in ns
	real_sample_time = sample_time;
	for (i=0; i < 9 - unit; i++)
		real_sample_time *= 10;
	
	// 16 bits timer
	if (id <= TIMER9)
	{
		// set the 16 bits mode
		m_set_32bits_mode(id, TIMER_16B_MODE);
		
		// control the range validity
		if ((real_sample_time < clock_get_cycle_duration()) || (real_sample_time > TIMER_16B_MAX_TIME_NS))
			ERROR(TIMER_ERROR_SAMPLE_TIME_NOT_IN_RANGE, &real_sample_time)
		
		// compute the prescaler
		for (prescaler=0; real_sample_time > ((unsigned long long)clock_get_cycle_duration() * 0x0000ffffULL * (unsigned long long)prescaler_value[prescaler]); prescaler++)
			;
		
		// set the prescaler
		m_set_prescaler(id, prescaler);
		
		// compute the period value
		period = real_sample_time / ((unsigned long long)clock_get_cycle_duration() * (unsigned long long)prescaler_value[prescaler]);
		
		// set the period value
		m_set_period_16b(id, (unsigned short)period);
	}
	// 32 bits timer
	else
	{
		// set the 32 bits mode
		m_set_32bits_mode(id, TIMER_32B_MODE);
		
		// control the range validity
		if ((real_sample_time < clock_get_cycle_duration()) || (real_sample_time > TIMER_32B_MAX_TIME_NS))
			ERROR(TIMER_ERROR_SAMPLE_TIME_NOT_IN_RANGE, &real_sample_time)
		
		// compute the prescaler
		for (prescaler=0; real_sample_time > ((unsigned long long)clock_get_cycle_duration() * 0x0000ffffULL * (unsigned long long)prescaler_value[prescaler]); prescaler++)
			;
		
		// set the prescaler
		m_set_prescaler(id, prescaler);
		
		// compute the period value
		period = real_sample_time / ((unsigned long long)clock_get_cycle_duration() * (unsigned long long)prescaler_value[prescaler]);
		
		// set the period value
		m_set_period_32b(id, (unsigned long)period);
	}

	// Reset the timer counter
	timer_set_value(id, 0);
}


/**
	Release the timer, so that someone else can use it.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
*/
void timer_release(int id)
{
	int index;
	
	// test the validity of the timer identifier
	if(id < TIMER1 || id > TIMER89)
		ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id)
	
	index = timer_id_to_index(id);
	Timer_Data[index].is_initialized = 0;
	Timer_Data[index].is_free = 1;
	if (timer_id_to_32bits(id))
	{
		// 32 bits timers consume two slots, free the other too
		Timer_Data[index + 1].is_free = 1;
	}
}


/**
	Return wether a timer is available for use

	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).

	\return
	true is timer is free, false if it is already used and not available
 */
bool timer_is_free(int id)
{
	return Timer_Data[timer_id_to_index(id)].is_free;
}

/**
	Enable a timer

	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
*/
void timer_enable(int id)
{
	timer_set_enabled(id, true);
}

/**
	Disable a timer
	
	The timer is not released, even if enabled is false.

	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
*/
void timer_disable(int id)
{
	timer_set_enabled(id, false);
}

/**
	Enable / disable a timer
	
	Timer is not released, even if enabled is false.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
	\param	enabled
			True to enable timer, false to disable it.
*/
void timer_set_enabled(int id, bool enabled)
{
	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id)
		
	// is the timer initialized ?
	if (!Timer_Data[timer_id_to_index(id)].is_initialized)
		ERROR(TIMER_ERROR_NOT_INITIALIZED, &id)
	
	switch (id)
	{
		case TIMER1:	T1CONbits.TON = enabled;
			break;
		case TIMER2:
		case TIMER23:	T2CONbits.TON = enabled;
			break;
		case TIMER3:	T3CONbits.TON = enabled;
			break;
		case TIMER4:
		case TIMER45:	T4CONbits.TON = enabled;
			break;
		case TIMER5:	T5CONbits.TON = enabled;
			break;
		case TIMER6:
		case TIMER67:	T6CONbits.TON = enabled;
			break;
		case TIMER7:	T7CONbits.TON = enabled;
			break;
		case TIMER8:
		case TIMER89:	T8CONbits.TON = enabled;
			break;
		case TIMER9:	T9CONbits.TON = enabled;
			break;
	}
}


/**
	Set the value of the counter of a timer.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
*/
void timer_set_value(int id, unsigned long value)
{
	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id)
		
	switch (id)
	{
		case TIMER1: TMR1 = value; break;
		case TIMER2: TMR2 = value; break;
		case TIMER3: TMR3 = value; break;
		case TIMER4: TMR4 = value; break;
		case TIMER5: TMR5 = value; break;
		case TIMER6: TMR6 = value; break;
		case TIMER7: TMR7 = value; break;
		case TIMER8: TMR8 = value; break;
		case TIMER9: TMR9 = value; break;
		case TIMER23: TMR3HLD = value >> 16; TMR2 = value & 0xffff; break;
		case TIMER45: TMR5HLD = value >> 16; TMR4 = value & 0xffff; break;
		case TIMER67: TMR7HLD = value >> 16; TMR6 = value & 0xffff; break;
		case TIMER89: TMR9HLD = value >> 16; TMR8 = value & 0xffff; break;
	}
}

/**
	Returns the value of the counter of a timer.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
	\return
			The value of the counter of the requested timer.
*/
unsigned long timer_get_value(int id)
{
	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		ERROR_RET_0(TIMER_ERROR_INVALID_TIMER_ID, &id)
	
	switch (id)
	{
		case TIMER1: return TMR1;
		case TIMER2: return TMR2;
		case TIMER3: return TMR3;
		case TIMER4: return TMR4;
		case TIMER5: return TMR5;
		case TIMER6: return TMR6;
		case TIMER7: return TMR7;
		case TIMER8: return TMR8;
		case TIMER9: return TMR9;
		case TIMER23: return (unsigned long)TMR2 | ((unsigned long)TMR3HLD << 16);
		case TIMER45: return (unsigned long)TMR4 | ((unsigned long)TMR5HLD << 16);
		case TIMER67: return (unsigned long)TMR6 | ((unsigned long)TMR7HLD << 16);
		case TIMER89: return (unsigned long)TMR8 | ((unsigned long)TMR9HLD << 16);
	}
	return 0;
}

/**
	Set the clock source for a timer
	
	This function is useful if you want to use an external clock source, on the T1CK pin (rising edge).
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
	\param	clock_source
			The parameter can be one of the two following constants:
			- \ref TIMER_CLOCK_INTERNAL
			- \ref TIMER_CLOCK_EXTERNAL
*/
void timer_set_clock_source(int id, int clock_source)
{
	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id)
	
	switch (id)
	{
		case TIMER1:	T1CONbits.TCS = clock_source;
			break;
		case TIMER2:
		case TIMER23:	T2CONbits.TCS = clock_source;
			break;
		case TIMER3:	T3CONbits.TCS = clock_source;
			break;
		case TIMER4:
		case TIMER45:	T4CONbits.TCS = clock_source;
			break;
		case TIMER5:	T5CONbits.TCS = clock_source;
			break;
		case TIMER6:
		case TIMER67:	T6CONbits.TCS = clock_source;
			break;
		case TIMER7:	T7CONbits.TCS = clock_source;
			break;
		case TIMER8:
		case TIMER89:	T8CONbits.TCS = clock_source;
			break;
		case TIMER9:	T9CONbits.TCS = clock_source;
			break;
	}
}


/**
	Enable / disable the gated time accumulation
	
	This option is meaningful only when using the internal oscillator !
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
	\param	enable
			True to enable gated time accumulation.
*/
void timer_use_gated_time_accumulation(int id, bool enable)
{
	// test the validity of the timer identifier
	if(id < TIMER1 || id > TIMER89)
		ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id)
		
	switch (id)
	{
		case TIMER1:	T1CONbits.TGATE = enable;
			break;
		case TIMER2:
		case TIMER23:	T2CONbits.TGATE = enable;
			break;
		case TIMER3:	T3CONbits.TGATE = enable;
			break;
		case TIMER4:
		case TIMER45:	T4CONbits.TGATE = enable;
			break;
		case TIMER5:	T5CONbits.TGATE = enable;
			break;
		case TIMER6:
		case TIMER67:	T6CONbits.TGATE = enable;
			break;
		case TIMER7:	T7CONbits.TGATE = enable;
			break;
		case TIMER8:
		case TIMER89:	T8CONbits.TGATE = enable;
			break;
		case TIMER9:	T9CONbits.TGATE = enable;
			break;
	}
}


/**
	Enable a the interrupt of a timer
	
	Continue timer operation in Idle mode (but discontinue it in Sleep mode).
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
	\param 	callback
			Pointer to a function that will be called upon interrupt
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 7 (highest priority)
*/
void timer_enable_interrupt(int id, timer_callback callback, int priority)
{
	// test the validity of the timer identifier
	if(id < TIMER1 || id > TIMER89)
		ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id)
	
	Timer_Data[timer_id_to_index(id)].callback = callback;
	
	// TxCONbits.TSIDL = 0;		Continue operation in IDLE mode
	// _TxIP					Interrupt priority
	// _TxIF					Clear interrupt flag
	// _TxIE					Enable interrupt
	switch(id)
	{
		case TIMER1:
			T1CONbits.TSIDL = 0;
			_T1IP = priority;
			_T1IF = 0;
			_T1IE = 1;
			break;
		case TIMER2:
			T2CONbits.TSIDL = 0;
			_T2IP = priority;
			_T2IF = 0;
			_T2IE = 1;
			break;
		case TIMER23:
		case TIMER3:
			T3CONbits.TSIDL = 0;
			_T3IP = priority;
			_T3IF = 0;
			_T3IE = 1;
			break;
		case TIMER4:
			T4CONbits.TSIDL = 0;
			_T4IP = priority;
			_T4IF = 0;
			_T4IE = 1;
			break;
		case TIMER45:
		case TIMER5:
			T5CONbits.TSIDL = 0;
			_T5IP = priority;
			_T5IF = 0;
			_T5IE = 1;
			break;
		case TIMER6:
			T6CONbits.TSIDL = 0;
			_T6IP = priority;
			_T6IF = 0;
			_T6IE = 1;
			break;
		case TIMER67:
		case TIMER7:
			T7CONbits.TSIDL = 0;
			_T7IP = priority;
			_T7IF = 0;
			_T7IE = 1;
			break;
		case TIMER8:
			T8CONbits.TSIDL = 0;
			_T8IP = priority;
			_T8IF = 0;
			_T8IE = 1;
			break;
		case TIMER89:
		case TIMER9:
			T9CONbits.TSIDL = 0;
			_T9IP = priority;
			_T9IF = 0;
			_T9IE = 1;
			break;
	}
}


/**
	Disable the interrupt of a timer.
	
	Discontinue timer operation in Idle mode and in Sleep mode.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).
*/
void timer_disable_interrupt(int id)
{
	// test the validity of the timer identifier
	if(id < TIMER1 || id > TIMER89)
		ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id)
	
	// _TxIE					Disable interrupt
	// _TxIF					Clear interrupt flag
	// TxCONbits.TSIDL = 1;		Discontinue operation in IDLE mode
	switch(id)
	{
		case TIMER1:
			_T1IE = 0;
			_T1IF = 0;
			T1CONbits.TSIDL = 1;
			break;
		case TIMER2:
			_T2IE = 0;
			_T2IF = 0;
			T2CONbits.TSIDL = 1;
			break;
		case TIMER23:
		case TIMER3:
			_T3IE = 0;
			_T3IF = 0;
			T3CONbits.TSIDL = 1;
			break;
		case TIMER4:
			_T4IE = 0;
			_T4IF = 0;
			T4CONbits.TSIDL = 1;
			break;  
		case TIMER45:
		case TIMER5:
			_T5IE = 0;
			_T5IF = 0;
			T5CONbits.TSIDL = 1;
			break;
		case TIMER6:
			_T6IE = 0;
			_T6IF = 0;
			T6CONbits.TSIDL = 1;
			break;
		case TIMER67:
		case TIMER7:
			_T7IE = 0;
			_T7IF = 0;
			T7CONbits.TSIDL = 1;
			break;
		case TIMER8:
			_T8IE = 0;
			_T8IF = 0;
			T8CONbits.TSIDL = 1;
			break;
		case TIMER89:
		case TIMER9:
			_T9IE = 0;
			_T9IF = 0;
			T9CONbits.TSIDL = 1;
			break;
	}
}



//-----------------------------------
// Internal functions, implementation
//-----------------------------------

/**
	Set a timer in 16-bits or 32-bits mode.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89).

	\param	mode
			The parameter can be one of the two following constants:
			- TIMER_16B_MODE
			- TIMER_32B_MODE

	\note For a timer in 32-bits mode, only the configuration register associated with
	the even timer (T2CON, T4CON, T6CON, T8CON) is taken into account.
*/
void m_set_32bits_mode(int id, char mode)
{
	switch(id)
	{
		case TIMER2:
		case TIMER3:
		case TIMER23:	T2CONbits.T32 = mode;
			break;
		case TIMER4:
		case TIMER5:
		case TIMER45:	T4CONbits.T32 = mode;
			break;
		case TIMER6:
		case TIMER7:
		case TIMER67:	T6CONbits.T32 = mode;
			break;
		case TIMER8:
		case TIMER9:
		case TIMER89:	T8CONbits.T32 = mode;
			break;
	}
}


/**
	Set the prescaler register of a timer.

	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89)

	\param	prescaler
			Prescaler value. Can be one of the following:
			- 0: 1:1
			- 1: 1:8
			- 2: 1:64
			- 3: 1:256
 
	\note For a timer in 32-bits mode, only the configuration register associated with
	the even timer (T2CON, T4CON, T6CON, T8CON) is taken into account.
*/
void m_set_prescaler(int id, unsigned int prescaler)
{
	switch(id)
	{
		case TIMER1:	T1CONbits.TCKPS = prescaler;
			break;
		case TIMER2:
		case TIMER23:	T2CONbits.TCKPS = prescaler;
			break;
		case TIMER3:	T3CONbits.TCKPS = prescaler;
			break;
		case TIMER4:
		case TIMER45:	T4CONbits.TCKPS = prescaler;
			break;
		case TIMER5:	T5CONbits.TCKPS = prescaler;
			break;
		case TIMER6:
		case TIMER67:	T6CONbits.TCKPS = prescaler;
			break;
		case TIMER7:	T7CONbits.TCKPS = prescaler;
			break;
		case TIMER8:
		case TIMER89:	T8CONbits.TCKPS = prescaler;
			break;
		case TIMER9:	T9CONbits.TCKPS = prescaler;
			break;
	}
}


/**
	Set the period register of a 16-bits timer.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89)

	\param	period
			Value of the period. Can be any number between 0x0000 and 0xFFFF.
*/
void m_set_period_16b(int id, unsigned short period)
{
	switch(id)
	{
		case TIMER1:	PR1 = period;
			break;
		case TIMER2:	PR2 = period;
			break;
		case TIMER3:	PR3 = period;
			break;
		case TIMER4:	PR4 = period;
			break;
		case TIMER5:	PR5 = period;
			break;
		case TIMER6:	PR6 = period;
			break;
		case TIMER7:	PR7 = period;
			break;
		case TIMER8:	PR8 = period;
			break;
		case TIMER9:	PR9 = period;
			break;
	}
}


/**
	Set the period register of a 32-bits timer.

	To do this, the period register of the 2 corresponding 16-bits timers has to be set.

	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER1 -> \ref TIMER9) or one of the 32-bits timer (\ref TIMER23 -> \ref TIMER89)

	\param	period
			Value of the period. Can be any number between 0x0000 and 0xFFFFFFFF.
*/
void m_set_period_32b(int id, unsigned long period)
{
	switch(id)
	{
		case TIMER23:	
			PR2 = period & 0xffff;	// LSB
			PR3 = (period >> 16);	// MSB
			break;
		case TIMER45:	
			PR4 = period & 0xffff;	// LSB
			PR5 = (period >> 16);	// MSB
			break;
		case TIMER67:	
			PR6 = period & 0xffff;	// LSB
			PR7 = (period >> 16);	// MSB
			break;
		case TIMER89:	
			PR8 = period & 0xffff;	// LSB
			PR9 = (period >> 16);	// MSB
			break;
	}
}


//--------------------------
// Interrupt service routine
//--------------------------

/**
	Timer 1 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T1Interrupt(void)
{
	// function must exist because this interrupt is enabled
	Timer_Data[0].callback();
	
	_T1IF = 0;
}

/**
	Timer 2 Interrupt Service Routine.

	Call the user-defined function.
*/
void _ISR _T2Interrupt(void)
{
	// function must exist because this interrupt is enabled
	Timer_Data[1].callback();
	
	_T2IF = 0;
}

/**
	Timer 3 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T3Interrupt(void)
{
	// function must exist because this interrupt is enabled
	Timer_Data[2].callback();
	
	_T3IF = 0;
}

/**
	Timer 4 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T4Interrupt(void)
{
	// function must exist because this interrupt is enabled
	Timer_Data[3].callback();
	
	_T4IF = 0;
}

/**
	Timer 5 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T5Interrupt(void)
{
	// function must exist because this interrupt is enabled
	Timer_Data[4].callback();
	
	_T5IF = 0;
}

/**
	Timer 6 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T6Interrupt(void)
{
	// function must exist because this interrupt is enabled
	Timer_Data[5].callback();
	
	_T6IF = 0;
}

/**
	Timer 7 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T7Interrupt(void)
{
	// function must exist because this interrupt is enabled
	Timer_Data[6].callback();
	
	_T7IF = 0;
}

/**
	Timer 8 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T8Interrupt(void)
{
	// function must exist because this interrupt is enabled
	Timer_Data[7].callback();
	
	_T8IF = 0;
}

/**
	Timer 9 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T9Interrupt(void)
{
	// function must exist because this interrupt is enabled
	Timer_Data[8].callback();
	
	_T9IF = 0;
}

/*@}*/
