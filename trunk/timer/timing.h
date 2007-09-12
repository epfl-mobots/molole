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
 *  Defines all the timing related constants.
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

#ifndef _MOLOLE_TIMING_H
#define _MOLOLE_TIMING_H

/**
 *	Oscillator frequency. The dsPIC33 internal oscillator is rated at 7.37 MHz.
 */
#define FIN			7.37e6
/**
 * PLL prescaler N1
 */
#define N1			2
/**
 * PLL multiplication factor
 */
#define M			43
/**
 *	PLL postscaler N2
 */
#define N2			2
/**
 * 	PLL output frequency. According to Microchip's documentation, the value is equal to: fosc = fin*M/(N1*N2). 
 */
#define FOSC   		(((float)M)/((float)(N1*N2)))*FIN

/**
 *	The instruction cycle frequency is half of the PLL frequency, so: fcy = fosc/2.
 */
#define FCY     	(FOSC / 2.)
/**
 *	Number of CPU cycles in a millisecond
 */
#define MILLISEC  	(FCY/1.0e3)
/**
 *	Number of CPU cycles in a microsecond
 */
#define MICROSEC  	(FCY/1.0e6)
/**
 *	Number of CPU cycles in a nanosecond
 */
#define NANOSEC   	(FCY/1.0e9)
/**
 *	Duration of 1 CPU cycle, in [ns]
 */
#define	TCY_PIC		(1e9/FCY)

#endif // _TIMING_H
