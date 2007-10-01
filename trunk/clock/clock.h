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
	\brief Molole clock configuration and constants.
*/

#ifndef _MOLOLE_CLOCK_H
#define _MOLOLE_CLOCK_H

// Clock constants

/** The dsPIC33 internal oscillator is rated at 7.37 MHz. */
#define CLOCK_FIN			7.37e6
/** PLL prescaler N1 */
#define CLOCK_N1			6
/** PLL multiplication factor */
#define CLOCK_M				130
/**	PLL postscaler N2 */
#define CLOCK_N2			2
/**	PLL output frequency.  fosc = fin*M/(N1*N2). */
#define CLOCK_FOSC 			((float)(CLOCK_M / (float)(CLOCK_N1 * CLOCK_N2)) * CLOCK_FIN)
/** The instruction cycle frequency is half of the PLL frequency	fcy = fosc/2. */
#define CLOCK_FCY			(CLOCK_FOSC / 2.)
/**	Number of CPU cycles in a millisecond */
#define CLOCK_MILLISEC  	(CLOCK_FCY / 1.0e3)
/**	Number of CPU cycles in a microsecond */
#define CLOCK_MICROSEC  	(CLOCK_FCY / 1.0e6)
/**	Number of CPU cycles in a nanosecond */
#define CLOCK_NANOSEC   	(CLOCK_FCY / 1.0e9)
/**	Duration of 1 CPU cycle, in [ns] */
#define	CLOCK_TCY_PIC		(1e9 / CLOCK_FCY)

// Functions, doc in the .c

void clock_init_internal_rc(void);

#endif
