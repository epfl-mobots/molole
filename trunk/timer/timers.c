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
 *  \brief
 *  dsPIC33 Timers library implementation.
 */

#include <p33fxxxx.h>

#include "timers.h"
#include "timing.h"

//------------
// Definitions
//------------

/** m_set_32_bits_mode parameter. Enable 16-bits timer mode */
#define TIMER_16B_MODE						0
/** m_set_32_bits_mode parameter. Enable 32-bits timer mode */
#define TIMER_32B_MODE						1

/** Maximum reachable timing with a 16-bits timer. The 256 factor is the maximum prescaler of the timer. */
#define TIMER_16B_MAX_TIME_NS				(TCY_PIC*256*0x0000ffff)
/** Maximum reachable timing with a 32-bits timer. The 256 factor is the maximum prescaler of the timer. */
#define TIMER_32B_MAX_TIME_NS				(TCY_PIC*256*0xffffffff)

//-----------------------
// Structures definitions
//-----------------------

/** The struct for timer implementation data */
typedef struct
{
	timer_callback callback; /**< function to call upon interrupt, 0 if none */
	unsigned is_free:1; /**< true if timer is available for allocation */
	unsigned is_initialized:1; /**< true if timer can receive command */
	unsigned is_32bits:1; /**< true if timer is 32 bits */
} Timer_Data;

/** Timer implementation data */
static Timer_Data timers[9] =
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

/** Return if timer is 32 bits for a specific id, assumed to be in correct range */
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
	\brief 	Initialize a timer, before you can use it.
	
	This function initializes the timer, defines its period and reserves it, avoiding someone else to use it.
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)
	\param 	sample_time
			The period of the timer, expressed in the unit defined by the <i>unit</i> parameter
	\param 	unit
			Time base of the <i>sample_time</i> parameter. Possible values are:
			<ul>
				<li>0 : second</li>
				<li>3 : millisecond</li>
				<li>6 : microsecond</li>
				<li>9 : nanosecond</li>
			</ul>
	
	\return
	TIMER_NO_ERROR on success, or a value from timer_return_values describing the error.
	
	\note	Use timer_release() when you don't need a timer anymore, so that someone else can use it !
*/
int timer_init(int id, unsigned long int arg_sample_time, int unit)
{
	int index;
	
	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		return TIMER_ERROR_INVALIDE_TIMER_ID;
		
	// test that the timer is free
	if (!timers[timer_id_to_index(id)].is_free)
		return TIMER_ERROR_ALREADY_IN_USE;
	
	// disable timer interrupt
	timer_disable_interrupt(id);

	// set the period
	timer_set_period(id, arg_sample_time, unit);

	// By default, set to internal clock source, without gated time accumulation
	timer_set_clock_source(id, TIMER_CLOCK_INTERNAL);
	timer_use_gated_time_accumulation(id, false);
	
	// init ok !
	index = timer_id_to_index(id);
	timers[index].is_initialized = 1;
	timers[index].is_free = 0;
	if (timer_id_to_32bits(id))
	{
		// 32 bits timers consume two slots
		timers[index + 1].is_free = 0;
		timers[index].is_32bits = 1;
	}
	else
	{
		timers[index].is_32bits = 0;
	}
	
	return TIMER_NO_ERROR;
}


/**
	\brief	Set the period of a timer.
	
	The timer must already be initialized with timer_init() !
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)
	\param 	sample_time
			The period of the timer, expressed in the unit defined by the <i>unit</i> parameter
	\param 	unit
			Time base of the <i>sample_time</i> parameter. Possible values are:
			<ul>
				<li>0 : second</li>
				<li>3 : millisecond</li>
				<li>6 : microsecond</li>
				<li>9 : nanosecond</li>
			</ul>
	
	\return
	TIMER_NO_ERROR on success, or a value from timer_return_values describing the error.
	
	\note	Automatically reset the timer's counter.
*/
int timer_set_period(int id, unsigned long int sample_time, int unit)
{
	unsigned long long real_sample_time = 0;
	unsigned long long period = 0;
	unsigned long prescaler_value[4] = {1, 8, 64, 256};
	
	unsigned int prescaler = 0;
	int i;

	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		return TIMER_ERROR_INVALIDE_TIMER_ID;
		
	// is the timer initialized ?
	if (!timers[timer_id_to_index(id)].is_initialized)
		return TIMER_ERROR_NOT_INITIALIZED;
		
	// only unit 0,3,6,9 ok
	if (!(unit == 0 || unit == 3 || unit == 6 || unit == 9))
		return TIMER_ERROR_INVALIDE_UNIT;
	
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
		if ((real_sample_time < TCY_PIC) || (real_sample_time > TIMER_16B_MAX_TIME_NS))
			return TIMER_ERROR_SAMPLE_TIME_NOT_IN_RANGE;
		
		// compute the prescaler
		for (prescaler=0; real_sample_time > ((unsigned long long)(TCY_PIC) * 0x0000ffff * prescaler_value[prescaler]); prescaler++)
			;
		
		// set the prescaler
		m_set_prescaler(id, prescaler);
		
		// compute the period value
		period = real_sample_time / ((unsigned long long)((TCY_PIC) * prescaler_value[prescaler]));
		
		// set the period value
		m_set_period_16b(id, (unsigned short)period);
	}
	// 32 bits timer
	else
	{
		// set the 32 bits mode
		m_set_32bits_mode(id, TIMER_32B_MODE);
		
		// control the range validity
		if ((real_sample_time < TCY_PIC) || (real_sample_time > TIMER_32B_MAX_TIME_NS))
			return TIMER_ERROR_SAMPLE_TIME_NOT_IN_RANGE;
		
		// compute the prescaler
		for (prescaler=0; real_sample_time > ((unsigned long long)(TCY_PIC) * 0xffffffff * prescaler_value[prescaler]); prescaler++)
			;
		
		// set the prescaler
		m_set_prescaler(id, prescaler);
		
		// compute the period value
		period = real_sample_time / ((unsigned long long)((TCY_PIC) * prescaler_value[prescaler]));
		
		// set the period value
		m_set_period_32b(id, (unsigned long)period);
	}

	// Reset the timer counter
	timer_reset(id);
	
	return TIMER_NO_ERROR;
}


/**
	\brief	Release the timer, so that someone else can use it.
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)
	
	\return	
	TIMER_NO_ERROR on success, or a value from timer_return_values describing the error
*/
int timer_release(int id)
{
	int index;
	
	// test the validity of the timer identifier
	if(id < TIMER1 || id > TIMER89)
		return TIMER_ERROR_INVALIDE_TIMER_ID;
	
	index = timer_id_to_index(id);
	timers[index].is_initialized = 0;
	timers[index].is_free = 1;
	if (timer_id_to_32bits(id))
	{
		// 32 bits timers consume two slots, free the other too
		timers[index + 1].is_free = 1;
	}
	
	return TIMER_NO_ERROR;
}


/**
	\brief	Returns if a timer is available for use

	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)

	\return
	true is timer is free, false if it is already used and not available
 */
bool timer_is_free(int id)
{
	return timers[timer_id_to_index(id)].is_free;
}


/**
	\brief	Enable / disable the timer
	
	Timer is not released, even if enabled is false.
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)
	\param	mode
			True to enable timer, false to disable it
	
	\return	
	TIMER_NO_ERROR on success, or a value from timer_return_values describing the error
*/
int timer_set_enabled(int id, bool enabled)
{
	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		return TIMER_ERROR_INVALIDE_TIMER_ID;
		
	// is the timer initialized ?
	if (!timers[timer_id_to_index(id)].is_initialized)
		return TIMER_ERROR_NOT_INITIALIZED;
	
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
	
	return TIMER_NO_ERROR;
}


/**
	\brief 	Reset the counter of the timer.
	
	 This is useful if you want to restart the timer.
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)
	
	\return	
	TIMER_NO_ERROR on success, or a value from timer_return_values describing the error
*/
int timer_reset(int id)
{
	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		return TIMER_ERROR_INVALIDE_TIMER_ID;
		
	switch (id)
	{
		case TIMER1:	TMR1 = 0;	
			break;
		case TIMER2:
		case TIMER23:	TMR2 = 0;
			break;
		case TIMER3:	TMR3 = 0;
			break;
		case TIMER4:
		case TIMER45:	TMR4 = 0;
			break;
		case TIMER5:	TMR5 = 0;
			break;
		case TIMER6:
		case TIMER67:	TMR6 = 0;
			break;
		case TIMER7:	TMR7 = 0;
			break;
		case TIMER8:
		case TIMER89:	TMR8 = 0;
			break;
		case TIMER9:	TMR9 = 0;
			break;
	}
	
	return TIMER_NO_ERROR;
}


/**
	\brief	Set the clock source for the timer
	
	This function is useful if you want to use an external clock source, on the T1CK pin (rising edge).
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)
	\param	clock_source
			The parameter can be one of the two following constants:
			<ul>
				<li>TIMER_INTERNAL_CLOCK</li>
				<li>TIMER_EXTERNAL_CLOCK</li>
			</ul>
	
	\return	
	TIMER_NO_ERROR on success, or a value from timer_return_values describing the error
*/
int timer_set_clock_source(int id, int clock_source)
{
	// test the validity of the timer identifier
	if (id < TIMER1 || id > TIMER89)
		return TIMER_ERROR_INVALIDE_TIMER_ID;
	
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
	
	return TIMER_NO_ERROR;
}


/**
	\brief Enable / disable the gated time accumulation
	
	Enable / disable the gated time accumulation. This option is meaningful only when using the internal oscillator !
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)
	\param	enable
			True to enable gated time accumulation
	
	\return	
	TIMER_NO_ERROR on success, or a value from timer_return_values describing the error
*/
int timer_set_gate_time_accumulation(int id, char mode)
{
	// test the validity of the timer identifier
	if(id < TIMER1 || id > TIMER89)
		return TIMER_ERROR_INVALIDE_TIMER_ID;
		
	switch (id)
	{
		case TIMER1:	T1CONbits.TGATE = mode;
			break;
		case TIMER2:
		case TIMER23:	T2CONbits.TGATE = mode;
			break;
		case TIMER3:	T3CONbits.TGATE = mode;
			break;
		case TIMER4:
		case TIMER45:	T4CONbits.TGATE = mode;
			break;
		case TIMER5:	T5CONbits.TGATE = mode;
			break;
		case TIMER6:
		case TIMER67:	T6CONbits.TGATE = mode;
			break;
		case TIMER7:	T7CONbits.TGATE = mode;
			break;
		case TIMER8:
		case TIMER89:	T8CONbits.TGATE = mode;
			break;
		case TIMER9:	T9CONbits.TGATE = mode;
			break;
	}
	
	return TIMER_NO_ERROR;
}


/**
	\brief	Enable the timer's interrupt.
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 7 (highest priority)
	\param 	callback
			Pointer to a function that will be called upon interrupt
	
	\return	
	TIMER_NO_ERROR on success, or a value from timer_return_values describing the error
*/
int timer_enable_interrupt(int id, int priority, timer_callback callback)
{
	// test the validity of the timer identifier
	if(id < TIMER1 || id > TIMER89)
		return TIMER_ERROR_INVALIDE_TIMER_ID;
	
	timers[timer_id_to_index(id)].callback = callback;
	
	switch(id)
	{
		case TIMER1:
			_T1IP = priority;
			_T1IF = 0;
			_T1IE = 1;
			break;
		case TIMER2:
			_T2IP = priority;
			_T2IF = 0;
			_T2IE = 1;
			break;
		case TIMER23:
		case TIMER3:
			_T3IP = priority;
			_T3IF = 0;
			_T3IE = 1;
			break;
		case TIMER4:
			_T4IP = priority;
			_T4IF = 0;
			_T4IE = 1;
			break;
		case TIMER45:
		case TIMER5:
			_T5IP = priority;
			_T5IF = 0;
			_T5IE = 1;
			break;
		case TIMER6:
			_T6IP = priority;
			_T6IF = 0;
			_T6IE = 1;
			break;
		case TIMER67:
		case TIMER7:
			_T7IP = priority;
			_T7IF = 0;
			_T7IE = 1;
			break;
		case TIMER8:
			_T8IP = priority;
			_T8IF = 0;
			_T8IE = 1;
			break;
		case TIMER89:
		case TIMER9:
			_T9IP = priority;
			_T9IF = 0;
			_T9IE = 1;
			break;
	}
	
	return TIMER_NO_ERROR;
}


/**
	\brief	Disable the timer's interrupt.
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)
	
	\return	
	TIMER_NO_ERROR on success, or a value from timer_return_values describing the error
*/
int timer_disable_interrupt(int id)
{
	// test the validity of the timer identifier
	if(id < TIMER1 || id > TIMER89)
		return TIMER_ERROR_INVALIDE_TIMER_ID;
	
	switch(id)
	{
		case TIMER1:	_T1IE = 0;		_T1IF = 0;
			break;
		case TIMER2:	_T2IE = 0;		_T2IF = 0;
			break;
		case TIMER23:
		case TIMER3:	_T3IE = 0;		_T3IF = 0;
			break;
		case TIMER4:	_T4IE = 0;		_T4IF = 0;
			break;
		case TIMER45:
		case TIMER5:	_T5IE = 0;		_T5IF = 0;
			break;
		case TIMER6:	_T6IE = 0;		_T6IF = 0;
			break;
		case TIMER67:
		case TIMER7:	_T7IE = 0;		_T7IF = 0;
			break;
		case TIMER8:	_T8IE = 0;		_T8IF = 0;
			break;
		case TIMER89:
		case TIMER9:	_T9IE = 0;		_T9IF = 0;
			break;
	}
	
	return TIMER_NO_ERROR;
}



//-----------------------------------
// Internal functions, implementation
//-----------------------------------

/**
	\brief	Set the timer in 16-bits or 32-bits mode.
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)

	\param	mode
			The parameter can be one of the two following constants:
			<ul>
				<li>TIMER_16B_MODE</li>
				<li>TIMER_32B_MODE</li>
			</ul>

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
	\brief	Set the prescaler register of the timer.

	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)

	\param	prescaler
			Prescaler value. Can be one of the following:
			<ul>
				<li>0: 1:1</li>
				<li>1: 1:8</li>
				<li>2: 1:64</li>
				<li>3: 1:256</li>
			</ul>
 
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
	\brief	Set the period register of a 16-bits timer.
	
	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)

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
	\brief	Set the period register of a 32-bits timer.

	To do this, the period register of the 2 corresponding 16-bits timers has to be set.

	\param	id
			The timer can be one of the 16-bits timer (TIMER1 -> TIMER9) or one of the 32-bits timer (TIMER23 -> TIMER89)

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
	\brief	Timer 1 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void __attribute__((__interrupt__)) _ISR _T1Interrupt(void)
{
	// function must exist because this interrupt is enabled
	timers[0].callback();
	
	_T1IF=0;
}

/**
	\brief	Timer 2 Interrupt Service Routine.

	Call the user-defined function.
*/
void __attribute__((__interrupt__)) _ISR _T2Interrupt(void)
{
	// function must exist because this interrupt is enabled
	timers[1].callback();
	
	_T2IF=0;
}

/**
	\brief	Timer 3 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void __attribute__((__interrupt__)) _ISR _T3Interrupt(void)
{
	// function must exist because this interrupt is enabled
	timers[2].callback();
	
	_T3IF=0;
}

/**
	\brief	Timer 4 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void __attribute__((__interrupt__)) _ISR _T4Interrupt(void)
{
	// function must exist because this interrupt is enabled
	timers[3].callback();
	
	_T4IF=0;
}

/**
	\brief	Timer 5 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void __attribute__((__interrupt__)) _ISR _T5Interrupt(void)
{
	// function must exist because this interrupt is enabled
	timers[4].callback();
	
	_T5IF=0;
}

/**
	\brief	Timer 6 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void __attribute__((__interrupt__)) _ISR _T6Interrupt(void)
{
	// function must exist because this interrupt is enabled
	timers[5].callback();
	
	_T6IF=0;
}

/**
	\brief	Timer 7 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void __attribute__((__interrupt__)) _ISR _T7Interrupt(void)
{
	// function must exist because this interrupt is enabled
	timers[6].callback();
	
	_T7IF=0;
}

/**
	\brief	Timer 8 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void __attribute__((__interrupt__)) _ISR _T8Interrupt(void)
{
	// function must exist because this interrupt is enabled
	timers[7].callback();
	
	_T8IF=0;
}

/**
	\brief	Timer 9 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void __attribute__((__interrupt__)) _ISR _T9Interrupt(void)
{
	// function must exist because this interrupt is enabled
	timers[8].callback();
	
	_T9IF=0;
}
