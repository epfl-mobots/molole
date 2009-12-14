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
(\ref TIMER_2 + \ref TIMER_3, \ref TIMER_4 + \ref TIMER_5, \ref TIMER_6 + \ref TIMER_7, \ref TIMER_8 + \ref TIMER_9), which allows up to
4 32-bits timers (\ref TIMER_23, \ref TIMER_45, \ref TIMER_67, and \ref TIMER_89).
The management of the 16-bits / 32-bits timers is totally transparent,
preventing the user to configure an already in-use timer.

\section Limits

With a cycle frequency of 40 MHz, the maximum reachable timings are as follow:
- 16-bits timers : 419 ms
- 32-bits timers : 27'487 s

\section Usage

You can configure one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9), or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
The configuration process is: initializing the timer with the desired timing, optionally defining an interrupt routine,
and finally launch the timer.

\code
timer_init(TIMER_1, 400, 6)						// 400 us

timer_enable_interrupt(TIMER_1, int_timer1, 1);	// int_timer1 is a function of type timer_callback

timer_set_enabled(TIMER_1, true);				// start
\endcode

You can test if a timer is available with the function timer_is_free().

All functions cast errors using the \ref error "error reporting mechanism"

\section Bugs Known bugs

\subsection SIM MPLAB SIM

The following problems occurs when using the MPLAB SIM simulator (the library is working fine on the real microcontrollers):
- \ref TIMER_6 : timer_enable_interrupt() don't modify the IEC2.T6IE register ! So the interruption is never fired...
- \ref TIMER_9 : everything is ok (register IEC3.T9IE = 1), the timer normally counts, but the flag IFS3bits.T9IF is never set !
	The interruption is never fired ! If we set the flag by hand, we enter in the interruption...
- \ref TIMER_67 and \ref TIMER_89 : same problem as TIMER_9

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

// TODO: we could probably optimize this by looking directly into the registers and removing redundant fields.

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
	if (id <= TIMER_9)
		return id;
	else
		return (id - TIMER_23) * 2 + 1;
}

/** Return wether a timer is 32 bits for a specific id, assumed to be in correct range */
bool timer_id_to_32bits(int id)
{
	return id > TIMER_9;
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
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89)
	\param 	arg_sample_time
			The period of the timer, expressed in the unit defined by the \e unit parameter
	\param 	unit
			Time base of the \e sample_time parameter. Possible values are:
			- -4 : cpu clock/256
			- -3 : cpu clock/64
			- -2 : cpu clock/8
			- -1 : cpu clock
			-  0 : second
			-  3 : millisecond
			-  6 : microsecond
			-  9 : nanosecond
	
	\note	Use timer_release() when you don't need a timer anymore, so that someone else can use it !
*/
void timer_init(int id, unsigned long int arg_sample_time, int unit)
{
	int index;
	
	// test the validity of the timer identifier
	ERROR_CHECK_RANGE(id, TIMER_1, TIMER_89, TIMER_ERROR_INVALID_TIMER_ID);
		
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
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89)
	\param 	sample_time
			The period of the timer, expressed in the unit defined by the \e unit parameter
	\param 	unit
			Time base of the \e sample_time parameter. Possible values are:
			- -4 : cpu clock/256
			- -3 : cpu clock/64
			- -2 : cpu clock/8
			- -1 : cpu clock
			-  0 : second
			-  3 : millisecond
			-  6 : microsecond
			-  9 : nanosecond
	
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
	ERROR_CHECK_RANGE(id, TIMER_1, TIMER_89, TIMER_ERROR_INVALID_TIMER_ID);
	
	// is the timer initialized ?
	if (!Timer_Data[timer_id_to_index(id)].is_initialized)
		ERROR(TIMER_ERROR_NOT_INITIALIZED, &id)
	
	// only unit 0,3,6,9, -1, -2, -3, -4 ok
	if (!(unit == 0 || unit == 3 || unit == 6 || unit == 9 || unit == -1 
		  || unit == -2 || unit == -3 || unit == -4))
		ERROR(TIMER_ERROR_INVALID_UNIT, &unit)
	
	// compute sample time in ns
	if(unit >= 0) {
		real_sample_time = sample_time;
		for (i=0; i < 9 - unit; i++)
			real_sample_time *= 10;
	}
	
	// 16 bits timer
	if (id <= TIMER_9)
	{
		// set the 16 bits mode
		m_set_32bits_mode(id, TIMER_16B_MODE);
		
		if (unit >= 0) {
			// control the range validity
			if ((real_sample_time < clock_get_cycle_duration()) || (real_sample_time > TIMER_16B_MAX_TIME_NS))
				ERROR(TIMER_ERROR_SAMPLE_TIME_NOT_IN_RANGE, &real_sample_time)
		
			// compute the prescaler
			for (prescaler=0; real_sample_time > ((unsigned long long)clock_get_cycle_duration() * 0x0000ffffULL * (unsigned long long)prescaler_value[prescaler]); prescaler++)
				;
			// compute the period value
			period = real_sample_time / ((unsigned long long)clock_get_cycle_duration() * (unsigned long long)prescaler_value[prescaler]);

		} else {
			prescaler = -unit - 1;
			ERROR_CHECK_RANGE(sample_time, 0, 0xFFFFU, TIMER_ERROR_SAMPLE_TIME_NOT_IN_RANGE);
			period = sample_time;
		}
		// set the prescaler
		m_set_prescaler(id, prescaler);
		
		
		// set the period value
		m_set_period_16b(id, (unsigned short)period);
	}
	// 32 bits timer
	else
	{
		// set the 32 bits mode
		m_set_32bits_mode(id, TIMER_32B_MODE);
		
		if(unit >= 0) {
			// control the range validity
			if ((real_sample_time < clock_get_cycle_duration()) || (real_sample_time > TIMER_32B_MAX_TIME_NS))
				ERROR(TIMER_ERROR_SAMPLE_TIME_NOT_IN_RANGE, &real_sample_time)
		
			// compute the prescaler
			for (prescaler=0; real_sample_time / (clock_get_cycle_duration()*prescaler_value[prescaler]) > 0xFFFFFFFFULL; prescaler++)
				;
			// compute the period value
			period = (real_sample_time / clock_get_cycle_duration()) / prescaler_value[prescaler];
		} else {
			prescaler = -unit - 1;
			ERROR_CHECK_RANGE(sample_time, 0, 0xFFFFFFFFUL, TIMER_ERROR_SAMPLE_TIME_NOT_IN_RANGE);
			period = sample_time;
		}

		// set the prescaler
		m_set_prescaler(id, prescaler);
		
		// set the period value
		m_set_period_32b(id, (unsigned long)period);
	}

	// Reset the timer counter
	timer_set_value(id, 0);
}


/**
	Release the timer, so that someone else can use it.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
*/
void timer_release(int id)
{
	int index;
	
	// test the validity of the timer identifier
	ERROR_CHECK_RANGE(id, TIMER_1, TIMER_89, TIMER_ERROR_INVALID_TIMER_ID);
	
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
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).

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
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
*/
void timer_enable(int id)
{
	timer_set_enabled(id, true);
}

/**
	Disable a timer
	
	The timer is not released, even if enabled is false.

	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
*/
void timer_disable(int id)
{
	timer_set_enabled(id, false);
}

/**
	Enable / disable a timer
	
	Timer is not released, even if enabled is false.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
	\param	enabled
			True to enable timer, false to disable it.
*/
void timer_set_enabled(int id, bool enabled)
{
	// test the validity of the timer identifier
	ERROR_CHECK_RANGE(id, TIMER_1, TIMER_89, TIMER_ERROR_INVALID_TIMER_ID);
	
	// is the timer initialized ?
	if (!Timer_Data[timer_id_to_index(id)].is_initialized)
		ERROR(TIMER_ERROR_NOT_INITIALIZED, &id)
	
	switch (id)
	{
		case TIMER_1:	T1CONbits.TON = enabled;
			break;
		case TIMER_2:
		case TIMER_23:	T2CONbits.TON = enabled;
			break;
		case TIMER_3:	T3CONbits.TON = enabled;
			break;
		case TIMER_4:
		case TIMER_45:	T4CONbits.TON = enabled;
			break;
		case TIMER_5:	T5CONbits.TON = enabled;
			break;
#ifdef _T6IF
		case TIMER_6:
		case TIMER_67:	T6CONbits.TON = enabled;
			break;
#endif
#ifdef _T7IF
		case TIMER_7:	T7CONbits.TON = enabled;
			break;
#endif
#ifdef _T8IF
		case TIMER_8:
		case TIMER_89:	T8CONbits.TON = enabled;
			break;
#endif
#ifdef _T9IF
		case TIMER_9:	T9CONbits.TON = enabled;
			break;
#endif
		default:
			ERROR( TIMER_ERROR_INVALID_TIMER_ID, &id)
	}
}


/**
	Set the value of the counter of a timer.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
	\param	value
			value to set
*/
void timer_set_value(int id, unsigned long value)
{
	switch (id)
	{
		case TIMER_1: TMR1 = value; break;
		case TIMER_2: TMR2 = value; break;
		case TIMER_23: TMR3HLD = value >> 16; TMR2 = value & 0xffff; break;
		case TIMER_3: TMR3 = value; break;
		case TIMER_4: TMR4 = value; break;
		case TIMER_45: TMR5HLD = value >> 16; TMR4 = value & 0xffff; break;
		case TIMER_5: TMR5 = value; break;
#ifdef _T6IF
		case TIMER_6: TMR6 = value; break;
		case TIMER_67: TMR7HLD = value >> 16; TMR6 = value & 0xffff; break;
#endif
#ifdef _T7IF
		case TIMER_7: TMR7 = value; break;
#endif
#ifdef _T8IF
		case TIMER_8: TMR8 = value; break;
		case TIMER_89: TMR9HLD = value >> 16; TMR8 = value & 0xffff; break;
#endif
#ifdef _T9IF
		case TIMER_9: TMR9 = value; break;
#endif
		
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
}

/**
	Returns the value of the counter of a timer.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
	\return
			The value of the counter of the requested timer.
*/
unsigned long timer_get_value(int id)
{
	switch (id)
	{
		case TIMER_1: return TMR1;
		case TIMER_2: return TMR2;
		case TIMER_23: return (unsigned long)TMR2 | ((unsigned long)TMR3HLD << 16);
		case TIMER_3: return TMR3;
		case TIMER_4: return TMR4;
		case TIMER_45: return (unsigned long)TMR4 | ((unsigned long)TMR5HLD << 16);
		case TIMER_5: return TMR5;
#ifdef _T6IF
		case TIMER_6: return TMR6;
		case TIMER_67: return (unsigned long)TMR6 | ((unsigned long)TMR7HLD << 16);
#endif
#ifdef _T7IF
		case TIMER_7: return TMR7;
#endif
#ifdef _T8IF
		case TIMER_8: return TMR8;
		case TIMER_89: return (unsigned long)TMR8 | ((unsigned long)TMR9HLD << 16);
#endif
#ifdef _T9IF
		case TIMER_9: return TMR9;
#endif
		
		default:
			ERROR_RET_0(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
	return 0;
}

/**
	Set the clock source for a timer
	
	This function is useful if you want to use an external clock source, on the T1CK pin (rising edge).
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
	\param	clock_source
			One of the \ref timer_clock_source, that are:
			- \ref TIMER_CLOCK_INTERNAL
			- \ref TIMER_CLOCK_EXTERNAL
*/
void timer_set_clock_source(int id, int clock_source)
{
	ERROR_CHECK_RANGE(clock_source, 0, 1, TIMER_ERROR_INVALID_CLOCK_SOURCE);
	
	switch (id)
	{
		case TIMER_1:	T1CONbits.TCS = clock_source;
						T1CONbits.TSIDL = 0;
			break;
		case TIMER_23:
						T3CONbits.TSIDL = 0;
		case TIMER_2:
						T2CONbits.TCS = clock_source;
						T2CONbits.TSIDL = 0;
			break;
		case TIMER_3:	
						T3CONbits.TCS = clock_source;
						T3CONbits.TSIDL = 0;
			break;
		case TIMER_45:
						T5CONbits.TSIDL = 0;
		case TIMER_4:
						T4CONbits.TCS = clock_source;
						T4CONbits.TSIDL = 0;
			break;
		case TIMER_5:	
						T5CONbits.TCS = clock_source;
						T5CONbits.TSIDL = 0;
			break;
#ifdef _T6IF
		case TIMER_67:
						T7CONbits.TSIDL = 0;
		case TIMER_6:
						T6CONbits.TCS = clock_source;
						T6CONbits.TSIDL = 0;
			break;
#endif
#ifdef _T7IF
		case TIMER_7:	
						T7CONbits.TCS = clock_source;
						T7CONbits.TSIDL = 0;
			break;
#endif
#ifdef _T8IF
		case TIMER_89:
						T9CONbits.TSIDL = 0;
		case TIMER_8:
						T8CONbits.TCS = clock_source;
						T8CONbits.TSIDL = 0;
			break;
#endif
#ifdef _T9IF
		case TIMER_9:	
						T9CONbits.TCS = clock_source;
						T9CONbits.TSIDL = 0;
			break;
#endif
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
}


/**
	Enable / disable the gated time accumulation
	
	This option is meaningful only when using the internal oscillator !
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
	\param	enable
			True to enable gated time accumulation.
*/
void timer_use_gated_time_accumulation(int id, bool enable)
{
	switch (id)
	{
		case TIMER_1:	T1CONbits.TGATE = enable;
			break;
		case TIMER_2:
		case TIMER_23:	T2CONbits.TGATE = enable;
			break;
		case TIMER_3:	T3CONbits.TGATE = enable;
			break;
		case TIMER_4:
		case TIMER_45:	T4CONbits.TGATE = enable;
			break;
		case TIMER_5:	T5CONbits.TGATE = enable;
			break;
#ifdef _T6IF
		case TIMER_6:
		case TIMER_67:	T6CONbits.TGATE = enable;
			break;
#endif
#ifdef _T7IF
		case TIMER_7:	T7CONbits.TGATE = enable;
			break;
#endif
#ifdef _T8IF
		case TIMER_8:
		case TIMER_89:	T8CONbits.TGATE = enable;
			break;
#endif
#ifdef _T9IF
		case TIMER_9:	T9CONbits.TGATE = enable;
			break;
#endif
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
}


/**
	Enable a the interrupt of a timer
	
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
	\param 	callback
			Pointer to a function that will be called upon interrupt
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 6 (highest normal priority)
*/
void timer_enable_interrupt(int id, timer_callback callback, int priority)
{
	// test the validity of the timer identifier
	ERROR_CHECK_RANGE(id, TIMER_1, TIMER_89, TIMER_ERROR_INVALID_TIMER_ID);
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	
	Timer_Data[timer_id_to_index(id)].callback = callback;
	
	// TxCONbits.TSIDL = 0;		Continue operation in IDLE mode
	// _TxIP					Interrupt priority
	// _TxIF					Clear interrupt flag
	// _TxIE					Enable interrupt
	switch(id)
	{
		case TIMER_1:
			_T1IP = priority;
			_T1IF = 0;
			_T1IE = 1;
			break;
		case TIMER_2:
			_T2IP = priority;
			_T2IF = 0;
			_T2IE = 1;
			break;
		case TIMER_23:
		case TIMER_3:
			_T3IP = priority;
			_T3IF = 0;
			_T3IE = 1;
			break;
		case TIMER_4:
			_T4IP = priority;
			_T4IF = 0;
			_T4IE = 1;
			break;
		case TIMER_45:
		case TIMER_5:
			_T5IP = priority;
			_T5IF = 0;
			_T5IE = 1;
			break;
#ifdef _T6IF
		case TIMER_6:
			_T6IP = priority;
			_T6IF = 0;
			_T6IE = 1;
			break;
#endif
#ifdef _T7IF
		case TIMER_67:
		case TIMER_7:
			_T7IP = priority;
			_T7IF = 0;
			_T7IE = 1;
			break;
#endif
#ifdef _T8IF
		case TIMER_8:
			_T8IP = priority;
			_T8IF = 0;
			_T8IE = 1;
			break;
#endif
#ifdef _T9IF
		case TIMER_89:
		case TIMER_9:
			_T9IP = priority;
			_T9IF = 0;
			_T9IE = 1;
			break;
#endif
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
}

/**
	Force the interrupt of a timer.
	
	Force the timer interrupt to be called. Even if the timer is disabled.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).

	\return 
			\true if the timer interrupt was perviously enabled, false otherwise
*/

bool timer_force_interrupt(int id)
{
	// _TxIE					Disable interrupt
	// _TxIF					Clear interrupt flag
	bool ret;
	switch(id)
	{
		case TIMER_1:
			ret = _T1IE;
			_T1IE = 1;
			_T1IF = 1;
			break;
		case TIMER_2:
			ret = _T2IE;
			_T2IE = 1;
			_T2IF = 1;
			break;
		case TIMER_23:
		case TIMER_3:
			ret = _T3IE;
			_T3IE = 1;
			_T3IF = 1;
			break;
		case TIMER_4:
			ret = _T4IE;
			_T4IE = 1;
			_T4IF = 1;
			break;  
		case TIMER_45:
		case TIMER_5:
			ret = _T5IE;
			_T5IE = 1;
			_T5IF = 1;
			break;
#ifdef _T6IF
		case TIMER_6:
			ret = _T6IE;
			_T6IE = 1;
			_T6IF = 1;
			break;
#endif
#ifdef _T7IF
		case TIMER_67:
		case TIMER_7:
			ret = _T7IE;
			_T7IE = 1;
			_T7IF = 1;
			break;
#endif
#ifdef _T8IF
		case TIMER_8:
			ret = _T8IE;
			_T8IE = 1;
			_T8IF = 1;
			break;
#endif
#ifdef _T9IF
		case TIMER_89:
		case TIMER_9:
			ret = _T9IE;
			_T9IE = 1;
			_T9IF = 1;
			break;
#endif
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
	return ret;
}

/**
	Get the interrupt flag status of the timer
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).

	\return 
			\true if the timer interrupt flag was active, false otherwise
*/
bool timer_get_if(int id)
{
	switch(id)
	{
		case TIMER_1:
			return _T1IF;
		case TIMER_2:
			return _T2IF;
		case TIMER_23:
		case TIMER_3:
			return _T3IF;
		case TIMER_4:
			return _T4IF;
		case TIMER_45:
		case TIMER_5:
			return _T5IF;
#ifdef _T6IF
		case TIMER_6:
			return _T6IF;
#endif
#ifdef _T7IF
		case TIMER_67:
		case TIMER_7:
			return _T7IF;
#endif
#ifdef _T8IF
		case TIMER_8:
			return _T8IF;
#endif
#ifdef _T9IF
		case TIMER_89:
		case TIMER_9:
			return _T9IF;
#endif
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
}


/**
	Disable the interrupt of a timer.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).
*/
void timer_disable_interrupt(int id)
{
	// _TxIE					Disable interrupt
	// _TxIF					Clear interrupt flag
	// TxCONbits.TSIDL = 1;		Discontinue operation in IDLE mode
	switch(id)
	{
		case TIMER_1:
			_T1IE = 0;
			_T1IF = 0;
			break;
		case TIMER_2:
			_T2IE = 0;
			_T2IF = 0;
			break;
		case TIMER_23:
		case TIMER_3:
			_T3IE = 0;
			_T3IF = 0;
			break;
		case TIMER_4:
			_T4IE = 0;
			_T4IF = 0;
			break;  
		case TIMER_45:
		case TIMER_5:
			_T5IE = 0;
			_T5IF = 0;
			break;
#ifdef _T6IF
		case TIMER_6:
			_T6IE = 0;
			_T6IF = 0;
			break;
#endif
#ifdef _T7IF
		case TIMER_67:
		case TIMER_7:
			_T7IE = 0;
			_T7IF = 0;
			break;
#endif
#ifdef _T8IF
		case TIMER_8:
			_T8IE = 0;
			_T8IF = 0;
			break;
#endif
#ifdef _T9IF
		case TIMER_89:
		case TIMER_9:
			_T9IE = 0;
			_T9IF = 0;
			break;
#endif
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
}



//-----------------------------------
// Internal functions, implementation
//-----------------------------------

/**
	Set a timer in 16-bits or 32-bits mode.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89).

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
		case TIMER_2:
		case TIMER_3:
		case TIMER_23:	T2CONbits.T32 = mode;
			break;
		case TIMER_4:
		case TIMER_5:
		case TIMER_45:	T4CONbits.T32 = mode;
			break;
#ifdef _T6IF
		case TIMER_6:
		case TIMER_7:
		case TIMER_67:	T6CONbits.T32 = mode;
			break;
#endif
#ifdef _T8IF
		case TIMER_8:
		case TIMER_9:
		case TIMER_89:	T8CONbits.T32 = mode;
			break;
#endif
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
	}
}


/**
	Set the prescaler register of a timer.

	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89)

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
		case TIMER_1:	T1CONbits.TCKPS = prescaler;
			break;
		case TIMER_2:
		case TIMER_23:	T2CONbits.TCKPS = prescaler;
			break;
		case TIMER_3:	T3CONbits.TCKPS = prescaler;
			break;
		case TIMER_4:
		case TIMER_45:	T4CONbits.TCKPS = prescaler;
			break;
		case TIMER_5:	T5CONbits.TCKPS = prescaler;
			break;
#ifdef _T6IF
		case TIMER_6:
		case TIMER_67:	T6CONbits.TCKPS = prescaler;
			break;
#endif
#ifdef _T7IF
		case TIMER_7:	T7CONbits.TCKPS = prescaler;
			break;
#endif
#ifdef _T8IF
		case TIMER_8:
		case TIMER_89:	T8CONbits.TCKPS = prescaler;
			break;
#endif
#ifdef _T9IF
		case TIMER_9:	T9CONbits.TCKPS = prescaler;
			break;
#endif
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
}


/**
	Set the period register of a 16-bits timer.
	
	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89)

	\param	period
			Value of the period. Can be any number between 0x0000 and 0xFFFF.
*/
void m_set_period_16b(int id, unsigned short period)
{
	switch(id)
	{
		case TIMER_1:	PR1 = period;
			break;
		case TIMER_2:	PR2 = period;
			break;
		case TIMER_3:	PR3 = period;
			break;
		case TIMER_4:	PR4 = period;
			break;
		case TIMER_5:	PR5 = period;
			break;
#ifdef _T6IF
		case TIMER_6:	PR6 = period;
			break;
#endif
#ifdef _T7IF
		case TIMER_7:	PR7 = period;
			break;
#endif
#ifdef _T8IF
		case TIMER_8:	PR8 = period;
			break;
#endif
#ifdef _T9IF
		case TIMER_9:	PR9 = period;
			break;
#endif
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
}


/**
	Set the period register of a 32-bits timer.

	To do this, the period register of the 2 corresponding 16-bits timers has to be set.

	\param	id
			The timer can be one of the 16-bits timer (\ref TIMER_1 -> \ref TIMER_9) or one of the 32-bits timer (\ref TIMER_23 -> \ref TIMER_89)

	\param	period
			Value of the period. Can be any number between 0x0000 and 0xFFFFFFFF.
*/
void m_set_period_32b(int id, unsigned long period)
{
	switch(id)
	{
		case TIMER_23:	
			PR2 = period & 0xffff;	// LSB
			PR3 = (period >> 16);	// MSB
			break;
		case TIMER_45:	
			PR4 = period & 0xffff;	// LSB
			PR5 = (period >> 16);	// MSB
			break;
#ifdef _T6IF
		case TIMER_67:	
			PR6 = period & 0xffff;	// LSB
			PR7 = (period >> 16);	// MSB
			break;
#endif
#ifdef _T8IF
		case TIMER_89:	
			PR8 = period & 0xffff;	// LSB
			PR9 = (period >> 16);	// MSB
			break;
#endif
		default:
			ERROR(TIMER_ERROR_INVALID_TIMER_ID, &id);
			break;
	}
}


//--------------------------
// Interrupt service routine
//--------------------------

#ifndef TIMER_1_CUSTOM_INTERRUPT_HANDLER

/**
	Timer 1 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T1Interrupt(void)
{
	// function must exist because this interrupt is enabled
	Timer_Data[0].callback(TIMER_1);
	
	_T1IF = 0;
}

#endif

#ifndef TIMER_2_CUSTOM_INTERRUPT_HANDLER

/**
	Timer 2 Interrupt Service Routine.

	Call the user-defined function.
*/
void _ISR _T2Interrupt(void)
{
	_T2IF = 0;
	// function must exist because this interrupt is enabled
	Timer_Data[1].callback(TIMER_2);
}

#endif

#ifndef TIMER_3_CUSTOM_INTERRUPT_HANDLER

/**
	Timer 3 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T3Interrupt(void)
{
		
	_T3IF = 0;
	// function must exist because this interrupt is enabled
	if (Timer_Data[1].is_initialized && Timer_Data[1].is_32bits)
		Timer_Data[1].callback(TIMER_23);
	else
		Timer_Data[2].callback(TIMER_3);
}

#endif

#ifndef TIMER_4_CUSTOM_INTERRUPT_HANDLER

/**
	Timer 4 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T4Interrupt(void)
{
	_T4IF = 0;
	
	// function must exist because this interrupt is enabled
	Timer_Data[3].callback(TIMER_4);
}

#endif

#ifndef TIMER_5_CUSTOM_INTERRUPT_HANDLER

/**
	Timer 5 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T5Interrupt(void)
{
	_T5IF = 0;

	// function must exist because this interrupt is enabled
	if (Timer_Data[3].is_initialized && Timer_Data[3].is_32bits)
		Timer_Data[3].callback(TIMER_45);
	else
		Timer_Data[4].callback(TIMER_5);
	
}

#endif

#ifndef TIMER_6_CUSTOM_INTERRUPT_HANDLER
#ifdef _T6IF
/**
	Timer 6 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T6Interrupt(void)
{
	_T6IF = 0;
	// function must exist because this interrupt is enabled
	Timer_Data[5].callback(TIMER_6);
	
}
#endif
#endif

#ifndef TIMER_7_CUSTOM_INTERRUPT_HANDLER
#ifdef _T7IF
/**
	Timer 7 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T7Interrupt(void)
{
	_T7IF = 0;

	// function must exist because this interrupt is enabled
	if (Timer_Data[5].is_initialized && Timer_Data[5].is_32bits)
		Timer_Data[5].callback(TIMER_67);
	else
		Timer_Data[6].callback(TIMER_7);

}
#endif
#endif

#ifndef TIMER_8_CUSTOM_INTERRUPT_HANDLER
#ifdef _T8IF
/**
	Timer 8 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T8Interrupt(void)
{
	_T8IF = 0;	
	
	// function must exist because this interrupt is enabled
	Timer_Data[7].callback(TIMER_8);
	
}

#endif
#endif

#ifndef TIMER_9_CUSTOM_INTERRUPT_HANDLER
#ifdef _T9IF
/**
	Timer 9 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _T9Interrupt(void)
{
	_T9IF = 0;

	// function must exist because this interrupt is enabled
	if (Timer_Data[7].is_initialized && Timer_Data[7].is_32bits)
		Timer_Data[7].callback(TIMER_89);
	else
		Timer_Data[8].callback(TIMER_9);
}
#endif
#endif

/*@}*/
