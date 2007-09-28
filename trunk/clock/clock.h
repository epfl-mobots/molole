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
#define MOLOLE_CLOCK_FIN		7.37e6
/** PLL prescaler N1 */
#define MOLOLE_CLOCK_N1			2.
/** PLL multiplication factor */
#define MOLOLE_CLOCK_M			130.
/**	PLL postscaler N2 */
#define MOLOLE_CLOCK_N2			6.
/**	PLL output frequency.  fosc = fin*M/(N1*N2). */
#define MOLOLE_CLOCK_FOSC 		((MOLOLE_CLOCK_M/(MOLOLE_CLOCK_N1*MOLOLE_CLOCK_N2))*MOLOLE_CLOCK_FIN)
/** The instruction cycle frequency is half of the PLL frequency	fcy = fosc/2. */
#define MOLOLE_CLOCK_FCY		(MOLOLE_CLOCK_FOSC / 2.)
/**	Number of CPU cycles in a millisecond */
#define MOLOLE_CLOCK_MILLISEC  	(MOLOLE_CLOCK_FCY/1.0e3)
/**	Number of CPU cycles in a microsecond */
#define MOLOLE_CLOCK_MICROSEC  	(MOLOLE_CLOCK_FCY/1.0e6)
/**	Number of CPU cycles in a nanosecond */
#define MOLOLE_CLOCK_NANOSEC   	(MOLOLE_CLOCK_FCY/1.0e9)
/**	Duration of 1 CPU cycle, in [ns] */
#define	MOLOLE_CLOCK_TCY_PIC	(1e9/MOLOLE_CLOCK_FCY)

// Functions, doc in the .c

void clock_init_pll(void);

#endif
