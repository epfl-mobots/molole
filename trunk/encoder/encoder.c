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
	unsigned int ipl;	/**< IPL of qei interrupt */
	unsigned int poscnt_b15; /**< errata 31 "QEI Interrupt Generation" */
	unsigned int got_irq; /**< Set to 1 by each interrupt */
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
	unsigned int ipl;	/**< IPL of the timer/IC interrupt */
	unsigned int got_irq; /**< Set to 1 by each interrupt */
} Software_Encoder_Data[9];



//------------------
// Private functions
//------------------

static void ic_tmr2_cb(int __attribute__((unused)) foo, unsigned int value, void * __attribute__((unused)) bar);

static void ic_tmr3_cb(int __attribute__((unused)) foo, unsigned int value, void * __attribute__((unused)) bar);

static void tmr_cb(int tmr);

static void tmr2_3_cb(int tmr);

static void ic_cb(int __attribute__((unused)) foo, unsigned int __attribute((unused)) bar, void * data);

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
	MAX1CNT = 0x7FFF;					// Errata 31 "QEI Interrupt Generation"
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

static void init_timer_encoder(unsigned timer, int ipl)
{
	timer_init(timer, 0xFFFF,-1);
	timer_set_clock_source(timer, TIMER_CLOCK_EXTERNAL);
	if(timer == TIMER_2 || timer == TIMER_3)
		timer_enable_interrupt(timer, tmr2_3_cb, ipl);
	else
		timer_enable_interrupt(timer, tmr_cb, ipl);
	timer_set_enabled(timer, true);
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
		QEI_Encoder_Data.ipl = priority;
		QEI_Encoder_Data.pos = pos;
		QEI_Encoder_Data.speed = speed;
		init_qei1_module(priority, direction, decoding_mode);
		return;
	case ENCODER_TIMER_2:
		Software_Encoder_Data[1].ipl = priority;
		Software_Encoder_Data[1].pos = pos;
		Software_Encoder_Data[1].speed = speed;
		Software_Encoder_Data[1].sens = gpio_read(gpio_dir);
		Software_Encoder_Data[1].up = !direction;  
		Software_Encoder_Data[1].mode = gpio_speed;
		Software_Encoder_Data[1].g_sens = gpio_dir;

		ic_enable(encoder_ic, IC_TIMER2, IC_EDGE_CAPTURE, ic_tmr2_cb, priority, 0);

		gpio_set_dir(gpio_dir, GPIO_INPUT);

		if(decoding_mode == ENCODER_MODE_X4) 
			gpio_set_dir(gpio_speed, GPIO_INPUT);
		else if(decoding_mode == ENCODER_MODE_X2) {
			gpio_write(gpio_speed, true);
			gpio_set_dir(gpio_speed, GPIO_OUTPUT);
		} else
			ERROR(ENCODER_INVALID_TYPE, &decoding_mode)

		init_timer_encoder(TIMER_2, priority);
		return;

	case ENCODER_TIMER_3:
		Software_Encoder_Data[2].ipl = priority;
		Software_Encoder_Data[2].pos = pos;
		Software_Encoder_Data[2].speed = speed;
		Software_Encoder_Data[2].sens = gpio_read(gpio_dir);
		Software_Encoder_Data[2].up = !direction;
		Software_Encoder_Data[2].mode = gpio_speed;
		Software_Encoder_Data[2].g_sens = gpio_dir;
		ic_enable(encoder_ic, IC_TIMER3, IC_EDGE_CAPTURE, ic_tmr3_cb, priority, 0);
		
		gpio_set_dir(gpio_dir, GPIO_INPUT);

		if(decoding_mode == ENCODER_MODE_X4) 
			gpio_set_dir(gpio_speed, GPIO_INPUT);
		else if(decoding_mode == ENCODER_MODE_X2) {
			gpio_write(gpio_speed, true);
			gpio_set_dir(gpio_speed, GPIO_OUTPUT);
		} else
			ERROR(ENCODER_INVALID_TYPE, &decoding_mode)

		init_timer_encoder(TIMER_3, priority);
		return;
		
	case ENCODER_TIMER_1:
	case ENCODER_TIMER_4:
	case ENCODER_TIMER_5:
	case ENCODER_TIMER_6:
	case ENCODER_TIMER_7:
	case ENCODER_TIMER_8:
	case ENCODER_TIMER_9:
		Software_Encoder_Data[type].ipl = priority;
		Software_Encoder_Data[type].pos = pos;
		Software_Encoder_Data[type].speed = speed;
		Software_Encoder_Data[type].sens = gpio_read(gpio_dir);
		Software_Encoder_Data[type].up = !direction;
		Software_Encoder_Data[type].mode = gpio_speed;
		Software_Encoder_Data[type].g_sens = gpio_dir;
		ic_enable(encoder_ic, 0, IC_EDGE_CAPTURE, ic_cb, priority, (void *) type);
		
		gpio_set_dir(gpio_dir, GPIO_INPUT);

		if(decoding_mode == ENCODER_MODE_X4) 
			gpio_set_dir(gpio_speed, GPIO_INPUT);
		else if(decoding_mode == ENCODER_MODE_X2) {
			gpio_write(gpio_speed, true);
			gpio_set_dir(gpio_speed, GPIO_OUTPUT);
		} else
			ERROR(ENCODER_INVALID_TYPE, &decoding_mode)

		init_timer_encoder(type, priority);
		return;
	
	
	default:
		ERROR(ENCODER_INVALID_TYPE, &type)
	}

}

/** 
	Get the position of the encoder without interfering with the speed mesurment
	
	\param 	type
			Type of encoder, either hardware of software, one of \ref encoder_type.
*/
long encoder_get_position(int type) {
	unsigned int temp1;
	unsigned int temp2;
	unsigned int b15;
	long temp3;
	switch(type) {
		case ENCODER_TIMER_1 ... ENCODER_TIMER_9:
			do {
				Software_Encoder_Data[type].got_irq = 0;
				barrier();
				
				temp3 = Software_Encoder_Data[type].tpos;
				temp1 = timer_get_value(type);
				temp2 = Software_Encoder_Data[type].sens;
					
				barrier();
			} while(Software_Encoder_Data[type].got_irq);
				
			if(temp2 == Software_Encoder_Data[type].up) 
				temp3 += temp1;
			else
				temp3 -= temp1;
			return temp3;
			
		case ENCODER_TYPE_HARD:	
			do {
				QEI_Encoder_Data.got_irq = 0;
				barrier();
				
				temp1 = POS1CNT;
				temp2 = QEI_Encoder_Data.high_word;
				b15 = QEI_Encoder_Data.poscnt_b15;
				barrier();
			} while(QEI_Encoder_Data.got_irq);
	
			return ((long) temp2) << 16 | (temp1 + b15);
		default:
			ERROR(ENCODER_INVALID_TYPE, &type);
	}
}
			

	

/**
	Update the position and speed variables of an encoder.
	
	\param	type
			Type of encoder, either hardware or software, one of \ref encoder_type.
*/
void encoder_step(int type)
{
	long pos = encoder_get_position(type);

	switch(type) {
		case ENCODER_TIMER_1 ... ENCODER_TIMER_9:
		
			*(Software_Encoder_Data[type].speed) = pos - *(Software_Encoder_Data[type].pos);
			*(Software_Encoder_Data[type].pos) = pos;

			break;
		case ENCODER_TYPE_HARD:
	
			*QEI_Encoder_Data.speed = pos - *QEI_Encoder_Data.pos;
			*QEI_Encoder_Data.pos = pos;
			
			break;
		default:
			ERROR(ENCODER_INVALID_TYPE, &type);
	}
}

/**
	Reset the encoder position and speed.
	
	\param	type
			Type of encoder, either hardware or software, one of \ref encoder_type.
*/
void encoder_reset(int type)
{
	int flags;
	switch(type){
		case ENCODER_TIMER_1 ... ENCODER_TIMER_9:
			// FIXME: disable only the encoder interrupts (timer/IC)
			RAISE_IPL(flags, Software_Encoder_Data[type].ipl);
			// Reset internal state
			timer_set_value(type, 0);
			Software_Encoder_Data[type].tpos = 0;
			
			// Reset external position
			*(Software_Encoder_Data[type].speed) = 0;
			*(Software_Encoder_Data[type].pos) = 0;
			
			IRQ_ENABLE(flags);		
			break;
		case ENCODER_TYPE_HARD:
			// FIXME: disable only the encoder interrupt
			RAISE_IPL(flags, QEI_Encoder_Data.ipl);
			
			QEI_Encoder_Data.high_word = 0;
			POS1CNT = 0;
			QEI_Encoder_Data.poscnt_b15 = 0;
			
			*QEI_Encoder_Data.speed = 0;
			*QEI_Encoder_Data.pos = 0;
			
			IRQ_ENABLE(flags);	
			break;
		default:
			ERROR(ENCODER_INVALID_TYPE, &type);
	}	
}

//! Callback for Input Capture 
static void ic_tmr2_cb(int __attribute__((unused)) foo, unsigned int value, void * __attribute__((unused)) bar)
{
	unsigned int tmr;

	if(Software_Encoder_Data[1].sens == Software_Encoder_Data[1].up) 
		Software_Encoder_Data[1].tpos += value;
	else
		Software_Encoder_Data[1].tpos -= value;

	/* Now update the timer with the correct value */
	/* Yeah, it's a bit racy, but we are a _lot_ more faster than the external clock
	 * So the race window is small */
	 
	 // Race window could be closed by doing a "TMR2 -= tmr" atomic instruction and then checking that 
	 // TMR2 is really at 0 but it's too much pain for a such small race.
	tmr = TMR2;
	TMR2 = 0;

	if (tmr < value)
	{
		/* WTF ?!? the timer has done an overflow while the motor has changed direction ...*/
		/* what can I do ? ... mmm .... */
		/* we must account the number of imp. done since the IC trigger */
		/* Clear the timer interrupt flag so it don't trigger and account false results */
		IFS0bits.T2IF = 0;
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
	
	Software_Encoder_Data[1].got_irq = 1;
	Software_Encoder_Data[1].sens = gpio_read(Software_Encoder_Data[1].g_sens);
}

//! Callback for Input Capture 
static void ic_tmr3_cb(int __attribute__((unused)) foo, unsigned int value, void * __attribute__((unused)) bar)
{
	unsigned int tmr;

	if(Software_Encoder_Data[2].sens == Software_Encoder_Data[2].up) 
		Software_Encoder_Data[2].tpos += value;
	else
		Software_Encoder_Data[2].tpos -= value;

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
		if(Software_Encoder_Data[2].sens == Software_Encoder_Data[2].up) {
			Software_Encoder_Data[2].tpos -= (((unsigned int) 0xFFFF) - value) + tmr;
		} else {
			Software_Encoder_Data[2].tpos += (((unsigned int) 0xFFFF) - value) + tmr;
		}
	} else {
		/* tmr - icval is the number of imp. done since we have changed direction */
		if(Software_Encoder_Data[2].sens == Software_Encoder_Data[2].up) 
			Software_Encoder_Data[2].tpos -= ((long) tmr) - ((long) value);
		else
			Software_Encoder_Data[2].tpos += ((long) tmr) - ((long) value);
	}
	
	Software_Encoder_Data[2].sens = gpio_read(Software_Encoder_Data[2].g_sens);
	
	Software_Encoder_Data[2].got_irq = 1;
}

static void tmr2_3_cb(int tmr) {
	
	if(Software_Encoder_Data[tmr].sens != gpio_read(Software_Encoder_Data[tmr].g_sens))
	{
		/* Wow, we changed direction and IC interrupt has still not fired ...
		 * Do not do anything */
		return ;
	}

	
	if (Software_Encoder_Data[tmr].sens == Software_Encoder_Data[tmr].up) 
		Software_Encoder_Data[tmr].tpos += 0x00010000L;
	else
		Software_Encoder_Data[tmr].tpos -= 0x00010000L;
		
	Software_Encoder_Data[tmr].got_irq = 1;
}

static void tmr_cb(int tmr) {
	if (Software_Encoder_Data[tmr].sens == Software_Encoder_Data[tmr].up) 
		Software_Encoder_Data[tmr].tpos += 0x00010000L;
	else
		Software_Encoder_Data[tmr].tpos -= 0x00010000L;


	Software_Encoder_Data[tmr].got_irq = 1;
}

static void ic_cb(int __attribute__((unused)) foo, unsigned int __attribute((unused)) bar, void * data) {
	unsigned int value = 0;
	unsigned int tmr = (unsigned int) data;
	
	// I don't use "molole" timer access function because I want to be _FAST_ to avoid as much
	// as possible a race between timer read and timer clear
	switch(tmr) {
		case TIMER_1:
			value = TMR1;
			TMR1 = 0;
			break;
		case TIMER_4:
			value = TMR4;
			TMR4 = 0;
			break;
		case TIMER_5:
			value = TMR5;
			TMR5 = 0;
			break;
		case TIMER_6:
			value = TMR6;
			TMR6 = 0;
			break;
		case TIMER_7:
			value = TMR7;
			TMR7 = 0;
			break;
		case TIMER_8:
			value = TMR8;
			TMR8 = 0;
			break;
		case TIMER_9:
			value = TMR9;
			TMR9 = 0;
			break;
	}
	
	if(Software_Encoder_Data[tmr].sens == Software_Encoder_Data[tmr].up) 
		Software_Encoder_Data[tmr].tpos += value;
	else
		Software_Encoder_Data[tmr].tpos -= value;
	
	Software_Encoder_Data[tmr].sens = gpio_read(Software_Encoder_Data[tmr].g_sens);
	
	Software_Encoder_Data[tmr].got_irq = 1;
	
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
	IFS3bits.QEIIF = 0;						// Clear interrupt flag
	
	QEI_Encoder_Data.got_irq = 1;
	
	QEI_Encoder_Data.poscnt_b15 ^=0x8000;
	
	// If we have done an overflow while b15 was set, then update high_word
	if((!QEI_Encoder_Data.poscnt_b15) && (POS1CNT < 0x3FFF)) 
		if (QEI1CONbits.UPDN)
			QEI_Encoder_Data.high_word++;		// Forward
		
	// If we have done an underflow while b15 was not set, then update high_word
	if((QEI_Encoder_Data.poscnt_b15) && (POS1CNT > 0x3FFF)) 
		if (!QEI1CONbits.UPDN)
			QEI_Encoder_Data.high_word--;

	
}

/*@}*/
