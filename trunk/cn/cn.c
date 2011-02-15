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

#include "../types/uc.h"

#include "cn.h"
#include "../error/error.h"
#include "../types/types.h"

#if defined __dsPIC33F__
	#define CN_NUMBER		32
#elif defined __PIC24F__
	#define CN_NUMBER		82
#endif

static cn_callback CN_callback;

/**
	Initialize Change Notification.
	
	All pins corresponding to bits at one on interrupt_mask must be configured as digital inputs by setting the associated bit in the TRISx register. Only CN0 to CN31 can be configured this way. On PIC24F, you should use cn_add_notification if you want to configure CN above 31, or the optionnal internal pull-down.

	Bear in mind there is only one interrupt for all the CN pins. They will thus share the same callback. And there is no hardware edge discrimination. Try using another peripheral if possible...
	
	Finally, the pull-up is usually pulling to only VDD - 0.7V, so you'd better use an external pull-up depending on your application.

	\param 	interrupt_mask
			Mask of pins that have to generate interrupts on state change (CN0 to CN31)
	\param	pull_up_mask
			Mask of pins that should have weak pull-up
	\param	callback
			User-define function to call an change notification interrupt
	\param  priority
			Interrupt priority
*/

void cn_init(unsigned long interrupt_mask, unsigned long pull_up_mask, cn_callback callback, int priority)
{
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);

	CNEN1 = (unsigned short)(interrupt_mask);
	CNEN2 = (unsigned short)(interrupt_mask >> 16);
	CNPU1 = (unsigned short)(pull_up_mask);
	CNPU2 = (unsigned short)(pull_up_mask >> 16);
	
	CN_callback = callback;
	
	_CNIP = priority;
	_CNIF = 0;					// Reset CN interrupt
	_CNIE = 1; 					// Enable CN interrupts
}

/**
	Add a Change Notification.
	
	cn_init must be called first, at least to enable the CN global interrupt. cn_init(NULL,NULL,priority) is OK.

	Bear in mind there is only one interrupt for all the CN pins. They will thus share the same callback (defined in cn_init). And there is no hardware edge discrimination. Try using another peripheral if possible...
	
	Finally, the pull-up is usually pulling to only VDD - 0.7V, so you'd better use an external pull-up depending on your application.

	\param 	channel
			Channel CNx on which you have to generate interrupt on state change (CN0 to CN_NUMBER-1)
	\param	pullup
			Enable the weak pull-up if true
	\param	pulldown
			Enable the weak pull-down if true
*/

void cn_add_notification(unsigned int channel, bool pullup, bool pulldown)
{
	_CNIE = 0;

	ERROR_CHECK_RANGE(channel, 0, CN_NUMBER - 1, CN_ERROR_INVALID_CHANNEL);

	if (pullup && pulldown)
	{
		ERROR(CN_ERROR_PU_AND_PD, NULL);
		return;
	}

	// Set the CNENx register
	volatile unsigned int* ptr = &CNEN1;
	unsigned int index = channel >> 4;
	unsigned int bit = channel & 0x000F;
	atomic_or(&ptr[index], (1 << bit));

	// Set the CNPUx register
	if (pullup) {
		ptr = &CNPU1;
		atomic_or(&ptr[index], (1 << bit));
	}
#if defined __PIC24F__
	// Set the CNPDx register
	if (pulldown) {
		ptr = &CNPD1;
		atomic_or(&ptr[index], (1 << bit));
	}
#endif

	_CNIF = 0;
	_CNIE = 1;
}

/**
	Remove a Change Notification.

	Inverse of cn_add_notification().
	
	\param 	channel
			Channel CNx to remove (CN0 to CN_NUMBER-1)
*/

void cn_remove_notification(unsigned int channel)
{
	_CNIE = 0;

	ERROR_CHECK_RANGE(channel, 0, CN_NUMBER - 1, CN_ERROR_INVALID_CHANNEL);

	// Unset the CNENx register
	volatile unsigned int * ptr = &CNEN1;
	unsigned int index = channel >> 4;
	unsigned int bit = channel & 0x000F;
	atomic_and(&ptr[index], ~(1 << bit));

	// Unset the CNPUx register
	ptr = &CNPU1;
	atomic_and(&ptr[index], ~(1 << bit));
#if defined __PIC24F__
	// Unset the CNPDx register
	ptr = &CNPD1;
	atomic_and(&ptr[index], ~(1 << bit));
#endif

	_CNIF = 0;
	_CNIE = 1;
}

/**
	Re-Enable the global Change Notification interrupt (all channels affected)
*/

void cn_enable_interrupt(void) {
	_CNIF = 0;
	_CNIE = 1;
}

/**
	Disable the global Change Notification interrupt (all channels affected)
*/

void cn_disable_interrupt(void) {
	_CNIE = 0;
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
	// Clear CN interrupt flag
	_CNIF = 0;
	
	// Call user-defined function
	if(CN_callback)
		CN_callback();
}

/*@}*/
