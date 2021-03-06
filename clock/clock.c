/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs

	Copyright (C) 2007--2011 Stephane Magnenat <stephane at magnenat dot net>,
	Philippe Retornaz <philippe dot retornaz at epfl dot ch>
	Mobots group (http://mobots.epfl.ch), Robotics system laboratory (http://lsro.epfl.ch)
	EPFL Ecole polytechnique federale de Lausanne (http://www.epfl.ch)

	See authors.txt for more details about other contributors.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, version 3 of the License.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
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


#include "clock.h"
#include "../types/types.h"


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
// Private functions
//-------------------
#ifdef _PLLPRE
static void setup_pll(unsigned n1, unsigned m, unsigned n2, unsigned long fin, unsigned osc) {

	// Make sure we are on a "safe" oscillator (internal RC w/o pll)
	__builtin_write_OSCCONH(CLOCK_FRC); 
	__builtin_write_OSCCONL(OSCCONL | 0x1); // Initiate the switch
	
	// Wait for the switch to complete
	while(OSCCONbits.OSWEN)
		barrier();
	
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
	__builtin_write_OSCCONH(osc); 
	__builtin_write_OSCCONL(OSCCONL | 0x1); // Initiate the switch

	// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1)
		barrier();
		
	// Wait for the switch to complete
	while(OSCCONbits.OSWEN)
		barrier();
	
	// Compute cycle frequency
	unsigned long fosc = (fin * (unsigned long)m) / ((unsigned long)n1 * (unsigned long)n2);
	Clock_Data.fcy = fosc / 2;
	Clock_Data.target_bogomips = (Clock_Data.fcy + 500000) / 1000000;
	
	// Lower the priority of all non-interrupt code.
	SRbits.IPL = 0;
}

//-------------------
// Exported functions
//-------------------


/**
	Initialize PLL for operation on external clock with specified value.
	\param	n1
			PLL prescaler
	\param 	m
			PLL multiplier
	\param 	n2
			PLL postscaler
	\param 	source_freq
			External source frequency in Hz
*/

void clock_init_external_clock_from_n1_m_n2(unsigned n1, unsigned m, unsigned n2, unsigned long source_freq)
{
	setup_pll(n1, m, n2, source_freq, CLOCK_PRIMPLL);
	
}

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
	setup_pll(n1, m, n2, 7370000UL, CLOCK_FRCPLL);
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
#endif

/**
	Force the clock speed, this does not configure any hardware clock.
*/

void clock_set_speed(unsigned long hz, unsigned int mips) {
	Clock_Data.fcy = hz;
	Clock_Data.target_bogomips = mips;
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

static int clock_idle_disabled = 1000; // Non-zero so we can detect underflow

/**
	Disable use of idle mode, if using buggy DMA, see Errata 38.
*/
void clock_disable_idle()
{
	// Ok, it's not atomic, but it's not a problem since we have a huge margin
	if(clock_idle_disabled <= 2000 && clock_idle_disabled >= 1000) { // too many nesting or underflow
		atomic_add(&clock_idle_disabled, 1);
	}
}

void clock_enable_idle()
{
	if(clock_idle_disabled <= 2000 && clock_idle_disabled >= 1000) {
		atomic_add(&clock_idle_disabled, -1);
	}
}
		

/**
	Put the processor in Idle mode, or, if it was disactivated by clock_disable_idle(), just return.
*/
void clock_idle()
{
	if (clock_idle_disabled == 1000)
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
