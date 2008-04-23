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
	\defgroup oc Output Compare
	
	Wrapper around Output Compare, with a callback oriented interface.
	
	The Output Compare interrupts are not supported, but the timers one are, through the \ref timer library.
*/
/*@{*/

/** \file
	Implementation of the wrapper around Output Compare.
*/


//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "oc.h"
#include "../error/error.h"
#include "../timer/timer.h"

// TODO if necessary: interrupt

// if timer enabled, disable it

/**
	Enable an Output Compare.
	
	If timer source is enabled, this function disables it.
	
	\param	oc_id
			Identifier of the Output Compare, from \ref OC_1 to \ref OC_8.
	\param	timer
			Timer providing clock to the Output Compare. Must be \ref TIMER_2 or \ref TIMER_3.
	\param	mode
			Mode of this Output Compare. Must be one of \ref oc_modes but not \ref OC_DISABLED.
*/
void oc_enable(int oc_id, int timer, int mode)
{
	int source;
	ERROR_CHECK_RANGE(mode, OC_DISABLED + 1, 7, OC_ERROR_INVALID_MODE);
	
	oc_disable(oc_id);
	
	if (timer == TIMER_2) {
		timer_set_enabled(TIMER_2, false);
		source = 0;
	} else if(timer == TIMER_3) {
		timer_set_enabled(TIMER_3, false);
		source = 1;
	} else 
		ERROR(OC_ERROR_INVALID_TIMER_SOURCE, &timer);
	
	switch (oc_id)
	{
		case OC_1: OC1CONbits.OCM = mode; OC1CONbits.OCTSEL = source; break;
		case OC_2: OC2CONbits.OCM = mode; OC2CONbits.OCTSEL = source; break;
		case OC_3: OC3CONbits.OCM = mode; OC3CONbits.OCTSEL = source; break;
		case OC_4: OC4CONbits.OCM = mode; OC4CONbits.OCTSEL = source; break;
		case OC_5: OC5CONbits.OCM = mode; OC5CONbits.OCTSEL = source; break;
		case OC_6: OC6CONbits.OCM = mode; OC6CONbits.OCTSEL = source; break;
		case OC_7: OC7CONbits.OCM = mode; OC7CONbits.OCTSEL = source; break;
		case OC_8: OC8CONbits.OCM = mode; OC8CONbits.OCTSEL = source; break;
		default: ERROR(OC_ERROR_INVALID_OC_ID, &oc_id);
	}
}

/**
	Disable an Output Compare.
	
	\param	oc_id
			Identifier of the Output Compare, from \ref OC_1 to \ref OC_8.
*/
void oc_disable(int oc_id)
{
	switch (oc_id)
	{
		case OC_1: OC1CONbits.OCM = OC_DISABLED; break;
		case OC_2: OC2CONbits.OCM = OC_DISABLED; break;
		case OC_3: OC3CONbits.OCM = OC_DISABLED; break;
		case OC_4: OC4CONbits.OCM = OC_DISABLED; break;
		case OC_5: OC5CONbits.OCM = OC_DISABLED; break;
		case OC_6: OC6CONbits.OCM = OC_DISABLED; break;
		case OC_7: OC7CONbits.OCM = OC_DISABLED; break;
		case OC_8: OC8CONbits.OCM = OC_DISABLED; break;
		default: ERROR(OC_ERROR_INVALID_OC_ID, &oc_id);
	}
}

/**
	Set the register values of an Output Compare.

	\param	oc_id
			Identifier of the Output Compare, from \ref OC_1 to \ref OC_8.
	\param	primary
			Output Compare register, named OCxR in the documentation.
	\param	secondary
			Secondary Output Compare register, named OCxRS in the documentation.
*/
void oc_set_value(int oc_id, unsigned primary, unsigned secondary)
{
	switch (oc_id)
	{
		case OC_1: OC1R = primary; OC1RS = secondary; break;
		case OC_2: OC2R = primary; OC2RS = secondary; break;
		case OC_3: OC3R = primary; OC3RS = secondary; break;
		case OC_4: OC4R = primary; OC4RS = secondary; break;
		case OC_5: OC5R = primary; OC5RS = secondary; break;
		case OC_6: OC6R = primary; OC6RS = secondary; break;
		case OC_7: OC7R = primary; OC7RS = secondary; break;
		case OC_8: OC8R = primary; OC8RS = secondary; break;
		default: ERROR(OC_ERROR_INVALID_OC_ID, &oc_id);
	}
}

/**
	Set the register values of an Output Compare for PWM use.

	\param	oc_id
			Identifier of the Output Compare, from \ref OC_1 to \ref OC_8.
	\param	duty
			Set the Secondary Output Compare register, named OCxRS in the documentation.
*/
void oc_set_value_pwm(int oc_id, unsigned duty) {
	switch (oc_id)
	{
		case OC_1: OC1RS = duty; break;
		case OC_2: OC2RS = duty; break;
		case OC_3: OC3RS = duty; break;
		case OC_4: OC4RS = duty; break;
		case OC_5: OC5RS = duty; break;
		case OC_6: OC6RS = duty; break;
		case OC_7: OC7RS = duty; break;
		case OC_8: OC8RS = duty; break;
		default: ERROR(OC_ERROR_INVALID_OC_ID, &oc_id);
	}

}

/*@}*/
