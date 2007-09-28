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
	\brief Implementation of the wrapper around dsPIC33 ADC.
*/

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
	\brief Initialize PLL for 40 MHz operations on internal RC
*/
void clock_init_pll(void)
{
	// TODO
	/*
	pll_pllfbd_conf:
	; PLLDIV = M = 130
	.word 0x0080
	
	pll_clkdiv_conf:
	; ROI = 0, interrupts have no effect on the DOZEN bit
	; DOZE = 0, processor clock reduction ratio = 1
	; DOZEN = 0 no processor clock reduction
	; FRCDIV = 0 , FRC divided by 1
	; PLLPOST = 0, N2 = 2
	; PLLPRE = 4, N1 = 6
	.word 0x0004
	*/
}
