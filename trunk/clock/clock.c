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
	\defgroup clock Clock
	
	Clock configuration.
	
	You must call either clock_init_internal_rc_40(), clock_init_internal_rc_30(), or
	clock_init_internal_rc_from_n1_m_n2() with valid values for n1, m, and n2 prior
	to use any peripheral.
	
	\section Usage
	
	Please refer to dsPIC33F Family Reference Manual Section 7 for more details.
*/
/*@{*/

/** \file
	\brief Implementation of clock configuration.
*/

//------------
// Definitions
//------------

// Clock constants



#include <p33fxxxx.h>

#include "clock.h"


//-----------------------
// Structures definitions
//-----------------------

/** Pre-computed clock constants */
static struct 
{
	unsigned long fcy; /**< instruction cycle frequency */
	unsigned target_bogomips;	/**< a vague and optimistic estimation of the MIPS this processor provides */
} Clock_Data;


//-------------------
// Exported functions
//-------------------

/**
	Initialize PLL for operations on internal RC with specified values.
	
	\param	n1
			PLL prescaler
	\param	m
			PLL multiplier
	\param	n2
			PLL postscaler
*/
void clock_init_internal_rc_from_n1_m_n2(unsigned n1, unsigned m, unsigned n2)
{
	// The dsPIC33 internal oscillator is rated at 7.37 MHz.
	const unsigned long fin = 7370000;
	
	_PLLPRE = n1 - 2;
	_PLLDIV = m - 2;
	switch (n2)
	{
		case 2: _PLLPOST = 0; break;
		case 4: _PLLPOST = 1; break;
		case 8: _PLLPOST = 3; break;
		default: _PLLPOST = 0;
	}
	
	// New oscillator selection bits
	asm volatile ("MOV #%0, w0 \n MOV #OSCCONH, w1 \n MOV #0x78, w2 \n MOV #0x9A, w3 \n MOV.b w2, [w1] \n MOV.b w3, [w1] \n MOV.b w0, [w1]" 
			: /* no outputs */ 
			: "i"(CLOCK_FRCPLL));

	// Request switch of clock to new oscillator source
	asm volatile ("MOV #0x01, w0 \n MOV #OSCCONL, w1 \n MOV #0x46, w2 \n MOV #0x57, w3 \n MOV.b w2, [w1] \n MOV.b w3, [w1] \n MOV.b w0, [w1]");

	// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1)
		{};
	
	// Compute cycle frequency
	unsigned long fosc = (fin * (unsigned long)m) / ((unsigned long)n1 * (unsigned long)n2);
	Clock_Data.fcy = fosc / 2;
	Clock_Data.target_bogomips = (Clock_Data.fcy + 500000) / 1000000;
	
	// Lower the priority of all non-interrupt code.
	SRbits.IPL = 0;
}

/**
	Initialize PLL for operations on internal RC for 30 MHz
*/
void clock_init_internal_rc_30()
{
	clock_init_internal_rc_from_n1_m_n2(8, 130, 2);
	Clock_Data.target_bogomips = 30;
}


/**
	Initialize PLL for operations on internal RC for 40 MHz
*/
void clock_init_internal_rc_40()
{
	clock_init_internal_rc_from_n1_m_n2(6, 130, 2);
	Clock_Data.target_bogomips = 40;
}

/**
	Return the duration of one CPU cycle, in ns
*/
unsigned long clock_get_cycle_duration()
{
	return 1000000000 / Clock_Data.fcy;
}

/**
	Return the frequency of CPU cycles, in Hz
*/
unsigned long clock_get_cycle_frequency()
{
	return Clock_Data.fcy;
}

/**
	Return the a vague and optimistic estimation of the MIPS this processor provides.
	
	Concretly, if the user called clock_init_internal_rc_30(), it will return 30, and
	if the user called clock_init_internal_rc_40(), it will return 40.
	
	If the user called clock_init_internal_rc_from_n1_m_n2() herself, this returns
	(Fcy + 500000) / 1000000.
*/
unsigned clock_get_target_bogomips()
{
	return Clock_Data.target_bogomips;
}

static bool clock_idle_enabled = true;

/**
	Disable use of idle mode, if using buggy DMA, see Errata 38.
*/
void clock_disable_idle()
{
	clock_idle_enabled = false;
}

/**
	Put the processor in Idle mode, or, if it was disactivated by clock_disable_idle(), just return.
*/
void clock_idle()
{
	if (clock_idle_enabled)
		__asm__ volatile ("pwrsav #1");
}

/**
	Delay the instruction flow of us microsecond. Expect a delay which is about 10-50% larger.
	\warning
		It does not take in account the timer lost in interrupt handler if preempted by an interrupt.
	\warning
		If the processor MIPS is lower than 4, this function does not work. And for low ( < 10 ) MIPS value the wait time is a lot larger than expected.
	\param us
		The number of microsecond to busy-wait
*/

void clock_delay_us(unsigned int us) 
{

	__asm__ volatile("inc %[us], %[us]\n\t"
					 "lsr %[mips], #2, w1\n\t"
					 "0:\n\t" 
					 "dec %[us], %[us]\n\t"
					 "bra z, 2f\n\t"
					 "mov 2, WREG\n\t" /* /!\ 2 is here for w1 address. since the assembler does not accept a mov w1, WREG to read w1 by accessing the file register so the flags are modified I have hardcoded the address into the assembler mnemonic */
					 "1:\n\t"
					 "bra z, 0b\n\t"
					 "dec w0,w0\n\t"
					 "bra 1b\n\t"
					 "2:\n\t" 
					: /* No output*/ 
					:[us] "r" (us), [mips] "r" (Clock_Data.target_bogomips)  /* Input */
					: "w0", "w1" /* We modify w0 and w1 */, "cc" /* alter flags */); 
}

/*@}*/
