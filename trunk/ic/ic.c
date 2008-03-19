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
	\defgroup ic Input Capture
	
	Wrapper around Input Capture, with a callback oriented interface.
*/
/*@{*/

/** \file
	Implementation of the wrapper around Input Capture.
*/


//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "ic.h"
#include "../error/error.h"

//-----------------------
// Structures definitions
//-----------------------

/** Input Capture wrapper data */
static struct
{
	ic_callback callback;	/**< callback to user-defined function */
	void* user_data;		/**< pointer to user-specified data to be passed in interrupt, may be 0 */
} IC_Data[8];


//-------------------
// Exported functions
//-------------------

/**
	Enable an Input Capture.
	
	This function does not change the state of the choosen timer.
	
	\param	ic_id
			Identifier of the Input Capture, from \ref IC_1 to \ref IC_8.
	\param	source
			Timer providing clock to the Input Capture. Must be \ref IC_TIMER2 or \ref IC_TIMER3.
	\param	mode
			Mode of this Input Capture. Must be one of \ref ic_modes but not \ref IC_DISABLED.
	\param	callback
			User-specified function to call when an input is captured.
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 6 (highest normal priority)
	\param user_data
			User data passed as callback argument
			
*/
void ic_enable(int ic_id, int source, int mode, ic_callback callback, int priority, void* user_data)
{
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	ERROR_CHECK_RANGE(source, 0, 1, IC_ERROR_INVALID_TIMER_SOURCE);
	ERROR_CHECK_RANGE(mode, IC_DISABLED + 1, 7, IC_ERROR_INVALID_MODE);
	if (mode == 6)
		ERROR(IC_ERROR_INVALID_MODE, &mode);
	
	switch (ic_id)
	{
		case IC_1:
		IC1CONbits.ICM = IC_DISABLED;
		IC1CONbits.ICTMR = source;
		IC1CONbits.ICM = mode;
		IC_Data[0].callback = callback;
		IC_Data[0].user_data = user_data;
		_IC1IP = priority;
		_IC1IF = 0;
		_IC1IE = 1;
		break;
		
		case IC_2:
		IC2CONbits.ICM = IC_DISABLED;
		IC2CONbits.ICTMR = source;
		IC2CONbits.ICM = mode;
		IC_Data[1].callback = callback;
		IC_Data[1].user_data = user_data;
		_IC2IP = priority;
		_IC2IF = 0;
		_IC2IE = 1;
		break;
		
		case IC_3:
		IC3CONbits.ICM = IC_DISABLED;
		IC3CONbits.ICTMR = source;
		IC3CONbits.ICM = mode;
		IC_Data[2].callback = callback;
		IC_Data[2].user_data = user_data;
		_IC3IP = priority;
		_IC3IF = 0;
		_IC3IE = 1;
		break;
		
		case IC_4:
		IC4CONbits.ICM = IC_DISABLED;
		IC4CONbits.ICTMR = source;
		IC4CONbits.ICM = mode;
		IC_Data[3].callback = callback;
		IC_Data[3].user_data = user_data;
		_IC4IP = priority;
		_IC4IF = 0;
		_IC4IE = 1;
		break;
		
		case IC_5:
		IC5CONbits.ICM = IC_DISABLED;
		IC5CONbits.ICTMR = source;
		IC5CONbits.ICM = mode;
		IC_Data[4].callback = callback;
		IC_Data[4].user_data = user_data;
		_IC5IP = priority;
		_IC5IF = 0;
		_IC5IE = 1;
		break;
		
		case IC_6:
		IC6CONbits.ICM = IC_DISABLED;
		IC6CONbits.ICTMR = source;
		IC6CONbits.ICM = mode;
		IC_Data[5].callback = callback;
		IC_Data[5].user_data = user_data;
		_IC6IP = priority;
		_IC6IF = 0;
		_IC6IE = 1;
		break;
		
		case IC_7:
		IC7CONbits.ICM = IC_DISABLED;
		IC7CONbits.ICTMR = source;
		IC7CONbits.ICM = mode;
		IC_Data[6].callback = callback;
		IC_Data[6].user_data = user_data;
		_IC7IP = priority;
		_IC7IF = 0;
		_IC7IE = 1;
		break;
		
		case IC_8:
		IC8CONbits.ICM = IC_DISABLED;
		IC8CONbits.ICTMR = source;
		IC8CONbits.ICM = mode;
		IC_Data[7].callback = callback;
		IC_Data[7].user_data = user_data;
		_IC8IP = priority;
		_IC8IF = 0;
		_IC8IE = 1;
		break;
		
		default: ERROR(IC_ERROR_INVALID_IC_ID, &ic_id);
	}
}

/**
	Disable an Input Capture.
	
	\param	ic_id
			Identifier of the Input Capture, from \ref IC_1 to \ref IC_8.
*/
void ic_disable(int ic_id)
{
	switch (ic_id)
	{
		case IC_1: IC1CONbits.ICM = IC_DISABLED; break;
		case IC_2: IC2CONbits.ICM = IC_DISABLED; break;
		case IC_3: IC3CONbits.ICM = IC_DISABLED; break;
		case IC_4: IC4CONbits.ICM = IC_DISABLED; break;
		case IC_5: IC5CONbits.ICM = IC_DISABLED; break;
		case IC_6: IC6CONbits.ICM = IC_DISABLED; break;
		case IC_7: IC7CONbits.ICM = IC_DISABLED; break;
		case IC_8: IC8CONbits.ICM = IC_DISABLED; break;
		default: ERROR(IC_ERROR_INVALID_IC_ID, &ic_id);
	}
}

//--------------------------
// Interrupt service routine
//--------------------------

/**
	Input Capture 1 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _IC1Interrupt(void)
{
	IC_Data[0].callback(IC_1, IC1BUF, IC_Data[0].user_data);
	
	// Clear interrupt flag
	_IC1IF = 0;
}

/**
	Input Capture 2 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _IC2Interrupt(void)
{
	IC_Data[1].callback(IC_2, IC2BUF, IC_Data[1].user_data);
	
	// Clear interrupt flag
	_IC2IF = 0;
}

/**
	Input Capture 3 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _IC3Interrupt(void)
{
	IC_Data[2].callback(IC_3, IC3BUF, IC_Data[2].user_data);
	
	// Clear interrupt flag
	_IC3IF = 0;
}

/**
	Input Capture 4 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _IC4Interrupt(void)
{
	IC_Data[3].callback(IC_4, IC4BUF, IC_Data[3].user_data);
	
	// Clear interrupt flag
	_IC4IF = 0;
}

/**
	Input Capture 5 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _IC5Interrupt(void)
{
	IC_Data[4].callback(IC_5, IC5BUF, IC_Data[4].user_data);
	
	// Clear interrupt flag
	_IC5IF = 0;
}

/**
	Input Capture 6 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _IC6Interrupt(void)
{
	IC_Data[5].callback(IC_6, IC6BUF, IC_Data[5].user_data);
	
	// Clear interrupt flag
	_IC6IF = 0;
}

/**
	Input Capture 7 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _IC7Interrupt(void)
{
	IC_Data[6].callback(IC_7, IC7BUF, IC_Data[6].user_data);
	
	// Clear interrupt flag
	_IC7IF = 0;
}

/**
	Input Capture 8 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _IC8Interrupt(void)
{
	IC_Data[7].callback(IC_8, IC8BUF, IC_Data[7].user_data);
	
	// Clear interrupt flag
	_IC8IF = 0;
}

/*@}*/
