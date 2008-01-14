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
	
	// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1)
		{};
	
	// Compute cycle frequency
	unsigned long fosc = (fin * (unsigned long)m) / ((unsigned long)n1 * (unsigned long)n2);
	Clock_Data.fcy = fosc / 2;
}

/**
	Initialize PLL for operations on internal RC for 30 MHz
*/
void clock_init_internal_rc_30()
{
	// TODO: add correct values.
	clock_init_internal_rc_from_n1_m_n2(6, 130, 2);
}


/**
	Initialize PLL for operations on internal RC for 40 MHz
*/
void clock_init_internal_rc_40()
{
	clock_init_internal_rc_from_n1_m_n2(6, 130, 2);
}

/**
	Returns the duration of one CPU cycle, in ns
/*/
unsigned long clock_get_cycle_duration()
{
	return 1000000000 / Clock_Data.fcy;
}

/**
	Returns the frequency of CPU cycles, in Hz
*/
unsigned long clock_get_cycle_frequency()
{
	return Clock_Data.fcy;
}

/*@}*/
