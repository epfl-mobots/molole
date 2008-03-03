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
#include "../timer/timer.h"
#include "../ic/ic.h"
#include "../gpio/gpio.h"

//-----------------------
// Structures definitions
//-----------------------

/** Data for the Quadrature Encoder Interface */
static struct
{	
	int high_word;		/**< High word of the 32 bits position */
	long* pos;			/**< absolute position */
	int* speed;			/**< difference of last two absolutes positions (i.e. speed) */
} QEI_Encoder_Data;

/** Data for the Software (emulated) Encoder Interface */
static struct
{
	long tpos;			/**< 32bits temporary position */
	int sens;			/**< Upward/backward counting */
	int up; 			/**< State of the pin when counting upward */
	long* pos;			/**< absolute position */
	int* speed;			/**< difference of last two absolutes positions (i.e. speed) */
	int ic;				/**< Input Capture to use, must be one of \ref ic_identifiers */
	gpio mode;			/**< GPIO used for mode selection */
	gpio g_sens;		/**< GPIO used for direction read */
} Software_Encoder_Data[2];



//------------------
// Private functions
//------------------

static void ic_tmr2_cb(int __attribute__((unused)) foo, unsigned int value, void * __attribute__((unused)) bar);

static void ic_tmr3_cb(int __attribute__((unused)) foo, unsigned int value, void * __attribute__((unused)) bar);

static void tmr_2_cb(int __attribute__((unused)) foo);

static void tmr_3_cb(int __attribute__((unused)) foo);

static void init_qei1_module(int ipl, bool reverse, int x2x4)
{
	QEI1CONbits.QEIM = 0;				// disable QEI
	IEC3bits.QEIIE = 0;					// disable interrupt
	QEI1CONbits.CNTERR = 0;				// No position count error has occured
	QEI1CONbits.QEISIDL = 0;			// Continue Module Operation in idle mode 
	QEI1CONbits.SWPAB = reverse;		// Phase A and Phase B not swapped 
	QEI1CONbits.PCDOUT = 0;				// Direction status Pin NOT controlled by the QEI Logic 
	QEI1CONbits.TQGATE = 0;				// Do not use timer function
	DFLT1CONbits.QEOUT = 1;				// Digital Filter QEA, QEB enabled 
	DFLT1CONbits.QECK = 0;				// clock divide QEA, QEB: 0=1,1=2,2=4,3=16,4=32,5=64,6=128,7=256
	QEI1CONbits.POSRES = 0;				// Index pulse does not reset position counter	
	DFLT1CONbits.CEID = 1;				// Interrups due to position count errors disabled
	MAX1CNT = 0xFFFF;					
	POS1CNT = 0;							// Set Counter to 0	
	
	if(x2x4 == ENCODER_MODE_X4)
		QEI1CONbits.QEIM = 7;			// x4 mode position counter reset by match MAXCNT
	else if(x2x4 == ENCODER_MODE_X2)
		QEI1CONbits.QEIM = 5;			// x2 mode position counter reset by match MAXCNT
	else
		ERROR(ENCODER_INVALID_TYPE, &x2x4);
	IFS3bits.QEIIF = 0;					// Clear interrupt flag
	IPC14bits.QEIIP = ipl;
	IEC3bits.QEIIE = 1;					// Enable Interrupt
}

static void init_timer2_encoder(int ipl)
{
	timer_init(TIMER_2, 0xFFFF,-1);
	timer_set_clock_source(TIMER_2, TIMER_CLOCK_EXTERNAL);
	timer_enable_interrupt(TIMER_2, tmr_2_cb, ipl);
	timer_set_enabled(TIMER_2, true);
}

static void init_timer3_encoder(int ipl)
{
	timer_init(TIMER_3, 0xFFFF,-1);
	timer_set_clock_source(TIMER_3, TIMER_CLOCK_EXTERNAL);
	timer_enable_interrupt(TIMER_3, tmr_3_cb, ipl);
	timer_set_enabled(TIMER_3, true);
}

//-------------------
// Exported functions
//-------------------

/**
	Init an encoder.
	
	\param	type
			Type of encoder, either hardware or software, one of \ref encoder_type.
	\param	encoder_ic
			Identifier of Input Capture for direction change from the external decoder, one of \ref ic_identifiers ; only used when type is \ref ENCODER_TIMER_2 or \ref ENCODER_TIMER_3 ; ignored when type is \ref ENCODER_TYPE_HARD
	\param	pos
			Pointer to where encoder_step() must update the position.
	\param	speed
			Pointer to where encoder_step() must update the speed (difference of positions between two calls to encoder_step()).
	\param	diretion
			Direction of counting, either \ref ENCODER_DIR_NORMAL or \ref ENCODER_DIR_REVERSE .
	\param	gpio_dir
			GPIO on which encoder_ic is routed ; only used when type is \ref ENCODER_TIMER_2 or \ref ENCODER_TIMER_3 ; ignored when type is \ref ENCODER_TYPE_HARD
	\param	gpio_speed
			GPIO used to control the decoding_mode if the external decoder ; only used when type is \ref ENCODER_TIMER_2 or \ref ENCODER_TIMER_3 ; ignored when type is \ref ENCODER_TYPE_HARD
	\param	decoding_mode
			speed of counting, either \ref ENCODER_MODE_X4 or \ref ENCODER_MODE_X2 .
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 6 (highest normal priority)
*/
void encoder_init(int type, int encoder_ic, long* pos, int* speed, int direction, gpio gpio_dir, gpio gpio_speed, int decoding_mode, int priority)
{
	switch(type)
	{
	case ENCODER_TYPE_HARD:
		QEI_Encoder_Data.pos = pos;
		QEI_Encoder_Data.speed = speed;
		init_qei1_module(priority, direction, decoding_mode);
		return;

	case ENCODER_TIMER_2:

		Software_Encoder_Data[0].pos = pos;
		Software_Encoder_Data[0].speed = speed;
		Software_Encoder_Data[0].sens = gpio_read(gpio_dir);
		Software_Encoder_Data[0].up = !direction;  
		Software_Encoder_Data[0].mode = gpio_speed;
		Software_Encoder_Data[0].g_sens = gpio_dir;

		ic_enable(encoder_ic, IC_TIMER2, IC_EDGE_CAPTURE, ic_tmr2_cb, priority, 0);

		gpio_set_dir(gpio_dir, GPIO_INPUT);

		if(decoding_mode == ENCODER_MODE_X4) 
			gpio_set_dir(gpio_speed, GPIO_INPUT);
		else if(decoding_mode == ENCODER_MODE_X2) {
			gpio_write(gpio_speed, true);
			gpio_set_dir(gpio_speed, GPIO_OUTPUT);
		} else
			ERROR(ENCODER_INVALID_TYPE, &decoding_mode)

		init_timer2_encoder(priority);
		return;

	case ENCODER_TIMER_3:
		Software_Encoder_Data[1].pos = pos;
		Software_Encoder_Data[1].speed = speed;
		Software_Encoder_Data[1].sens = gpio_read(gpio_dir);
		Software_Encoder_Data[1].up = !direction;
		Software_Encoder_Data[1].mode = gpio_speed;
		Software_Encoder_Data[1].g_sens = gpio_dir;
		ic_enable(encoder_ic, IC_TIMER3, IC_EDGE_CAPTURE, ic_tmr3_cb, priority, 0);
		
		gpio_set_dir(gpio_dir, GPIO_INPUT);

		if(decoding_mode == ENCODER_MODE_X4) 
			gpio_set_dir(gpio_speed, GPIO_INPUT);
		else if(decoding_mode == ENCODER_MODE_X2) {
			gpio_write(gpio_speed, true);
			gpio_set_dir(gpio_speed, GPIO_OUTPUT);
		} else
			ERROR(ENCODER_INVALID_TYPE, &decoding_mode)

		init_timer3_encoder(priority);
		return;
		
	default:
		ERROR(ENCODER_INVALID_TYPE, &type)
	}

}

/**
	Update the position and speed variables of an encoder.
	
	\param	type
			Type of encoder, either hardware or software, one of \ref encoder_type.
*/
void encoder_step(int type)
{
	long old_pos;
	unsigned int temp1;
	unsigned int temp2;
	long temp3;
	int flags;

	if (type == ENCODER_TIMER_2)
	{
		old_pos = *(Software_Encoder_Data[0].pos);

		IRQ_DISABLE(flags);
		temp3 = Software_Encoder_Data[0].tpos;
		temp1 = TMR2;
		temp2 = Software_Encoder_Data[0].sens;
		IRQ_ENABLE(flags);

		if(temp2 == Software_Encoder_Data[0].up) 
			temp3 += temp1;
		else
			temp3 -= temp1;

		*(Software_Encoder_Data[0].pos) = temp3;
		*(Software_Encoder_Data[0].speed) = *(Software_Encoder_Data[0].pos) - old_pos;
	}
	else if (type == ENCODER_TIMER_3)
	{
		old_pos = *(Software_Encoder_Data[1].pos);

		IRQ_DISABLE(flags);
		temp3 = Software_Encoder_Data[1].tpos;
		temp1 = TMR3;
		temp2 = Software_Encoder_Data[1].sens;
		IRQ_ENABLE(flags);

		if(temp2 == Software_Encoder_Data[1].up) 
			temp3 += temp1;
		else
			temp3 -= temp1;

		*(Software_Encoder_Data[1].pos) = temp3;
		*(Software_Encoder_Data[1].speed) = *(Software_Encoder_Data[1].pos) - old_pos;
	}
	else if (type == ENCODER_TYPE_HARD)
	{
		old_pos = *QEI_Encoder_Data.pos;

		IRQ_DISABLE(flags);
		temp1 = POS1CNT;
		temp2 = QEI_Encoder_Data.high_word;
		IRQ_ENABLE(flags);

		*QEI_Encoder_Data.pos = ((long) temp2) << 16 | temp1;
		*QEI_Encoder_Data.speed = *QEI_Encoder_Data.pos - old_pos;
	}
	else
	{
		ERROR(ENCODER_INVALID_TYPE, &type);
	}
}

//! Callback for Input Capture 
static void ic_tmr2_cb(int __attribute__((unused)) foo, unsigned int value, void * __attribute__((unused)) bar)
{
	unsigned int tmr;

	if(Software_Encoder_Data[0].sens == Software_Encoder_Data[0].up) 
		Software_Encoder_Data[0].tpos += value;
	else
		Software_Encoder_Data[0].tpos -= value;

	/* Now update the timer with the correct value */
	/* Yeah, it's a bit racy, but we are a _lot_ more faster than the external clock
	 * So the race window is small */
	tmr = TMR2;
	TMR2 = 0;

	if (tmr < value)
	{
		/* WTF ?!? the timer has done an overflow while the motor has changed direction ...*/
		/* what can I do ? ... mmm .... */
		/* we must account the number of imp. done since the IC trigger */
		/* Clear the timer interrupt flag so it don't trigger and account false results */
		IFS0bits.T2IF = 0;
		if(Software_Encoder_Data[0].sens == Software_Encoder_Data[0].up) {
			Software_Encoder_Data[0].tpos -= (((unsigned int) 0xFFFF) - value) + tmr;
		} else {
			Software_Encoder_Data[0].tpos += (((unsigned int) 0xFFFF) - value) + tmr;
		}
	} else {
		/* tmr - icval is the number of imp. done since we have changed direction */
		if(Software_Encoder_Data[0].sens == Software_Encoder_Data[0].up) 
			Software_Encoder_Data[0].tpos -= ((long) tmr) - ((long) value);
		else
			Software_Encoder_Data[0].tpos += ((long) tmr) - ((long) value);
	}
	
	Software_Encoder_Data[0].sens = gpio_read(Software_Encoder_Data[0].g_sens);
}

//! Callback for Input Capture 
static void ic_tmr3_cb(int __attribute__((unused)) foo, unsigned int value, void * __attribute__((unused)) bar)
{
	unsigned int tmr;

	if(Software_Encoder_Data[1].sens == Software_Encoder_Data[1].up) 
		Software_Encoder_Data[1].tpos += value;
	else
		Software_Encoder_Data[1].tpos -= value;

	/* Now update the timer with the correct value */
	/* Yeah, it's a bit racy, but we are a _lot_ more faster than the external clock
	 * So the race window is small */
	tmr = TMR3;
	TMR3 = 0;

	if (tmr < value)
	{
		/* WTF ?!? the timer has done an overflow while the motor has changed direction ...*/
		/* what can I do ? ... mmm .... */
		/* we must account the number of imp. done since the IC trigger */
		/* Clear the timer interrupt flag so it don't trigger and account false results */
		IFS0bits.T3IF = 0;
		if(Software_Encoder_Data[1].sens == Software_Encoder_Data[1].up) {
			Software_Encoder_Data[1].tpos -= (((unsigned int) 0xFFFF) - value) + tmr;
		} else {
			Software_Encoder_Data[1].tpos += (((unsigned int) 0xFFFF) - value) + tmr;
		}
	} else {
		/* tmr - icval is the number of imp. done since we have changed direction */
		if(Software_Encoder_Data[1].sens == Software_Encoder_Data[1].up) 
			Software_Encoder_Data[1].tpos -= ((long) tmr) - ((long) value);
		else
			Software_Encoder_Data[1].tpos += ((long) tmr) - ((long) value);
	}
	
	Software_Encoder_Data[1].sens = gpio_read(Software_Encoder_Data[1].g_sens);
}

//! Callback for Timer
static void tmr_2_cb(int __attribute__((unused)) foo)
{
	/* Ok, so we have an overflow. Let's update the internal count */
	if(Software_Encoder_Data[0].sens != gpio_read(Software_Encoder_Data[0].g_sens))
	{
		/* Wow, we changed direction and IC interrupt has still not fired ... 
		 * Do not do anything */
		return ;
	}
	if (Software_Encoder_Data[0].sens == Software_Encoder_Data[0].up) 
		Software_Encoder_Data[0].tpos += 0x00010000;
	else
		Software_Encoder_Data[0].tpos -= 0x00010000;

}

//! Callback for Timer
static void tmr_3_cb(int __attribute__((unused)) foo)
{
	/* Ok, so we have an overflow. Let's update the internal count */
	if(Software_Encoder_Data[1].sens != gpio_read(Software_Encoder_Data[1].g_sens))
	{
		/* Wow, we changed direction and IC interrupt has still not fired ... 
		 * Do not do anything */
		return ;
	}
	if (Software_Encoder_Data[1].sens == Software_Encoder_Data[1].up)
		Software_Encoder_Data[1].tpos += 0x00010000;
	else
		Software_Encoder_Data[1].tpos -= 0x00010000;
}

//--------------------------
// Interrupt service routine
//--------------------------

/**
	QEI Interrupt Service Routine.
 
	Manage lowest significant int over/under-flow to increment highest significant int.
*/
void _ISR _QEIInterrupt(void)
{
	if (QEI1CONbits.UPDN)
		QEI_Encoder_Data.high_word++;		// Forward
	else									// Backwards
		QEI_Encoder_Data.high_word--;	

	IFS3bits.QEIIF = 0;						// Clear interrupt flag
}

/*@}*/
