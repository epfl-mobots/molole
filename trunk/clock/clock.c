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

//--------------------
// Usage documentation
//--------------------

/**
	\defgroup clock Clock
	
	Clock configuration and constants.
*/
/*@{*/

/** \file
	\brief Implementation of clock configuration.
*/

//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "clock.h"

// TODO: module documentation
/*  Defines all the timing related constants.
*
*	This file defines the following timing related constants:
*  <ul>
*		<li> Internal oscillator frequency </li>
*		<li> PLL parameters, in order to get an oscillator frequency of 80 MHz (cylce frequency of 40 MHz) </li>
*		<li> Duration constants (ms, us, ns) </li>
*	</ul>
*  
*	This file is mainly used by the timers library (timers.h), and the PLL initialization routine.
*/

//-------------------
// Exported functions
//-------------------

/**
	\brief Initialize PLL from constants for operations on internal RC (constants are for 40 MHz)
*/
void clock_init_internal_rc(void)
{
	// Setup PLL constants from defines
	_PLLPRE = CLOCK_N1 - 2;
	_PLLDIV = CLOCK_M - 2;
	switch (CLOCK_N2)
	{
		case 2: _PLLPOST = 0; break;
		case 4: _PLLPOST = 1; break;
		case 8: _PLLPOST = 3; break;
		default: _PLLPOST = 0;
	}
	
	// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1)
		{};
}

/*@}*/
