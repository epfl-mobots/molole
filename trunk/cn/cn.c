/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	
	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>,
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
	\defgroup cn Change Notification
	
	A wrapper around dsPIC change notification feature which generates an interrupt on status change of some pins.
*/
/*@{*/

/** \file
	The implementation of the change notification wrapper
*/


//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "cn.h"

static cn_callback CN_Callback;

/**
	Initialize Change Notification.
	
	All pins corresponding to bits at one on interrupt_mask must be configured as digital inputs by setting the associated bit in the TRISx register.
	
	\param 	interrupt_mask
			mask of pins that have to generate interrupts on state change
	\param	pull_up_mask
			mask of pins that should have weak pull-up
	\param	callback
			user-define function to call an change notification interrupt
*/
void cn_init(unsigned long interrupt_mask, unsigned long pull_up_mask, cn_callback callback)
{
	CNEN1 = (unsigned short)(interrupt_mask);
	CNEN2 = (unsigned short)(interrupt_mask >> 16);
	CNPU1 = (unsigned short)(interrupt_mask);
	CNPU2 = (unsigned short)(interrupt_mask >> 16);
	
	CN_Callback = callback;
	
	_CNIF = 0;					// Reset CN interrupt
	_CNIE = 1; 					// Enable CN interrupts
}


//--------------------------
// Interrupt service routine
//--------------------------

/**
	Change Notification Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _CNInterrupt(void)
{
	// Call user-defined function
	CN_Callback();
	
	// Clear CN interrupt flag
	_CNIF = 0;
}


/*@}*/
