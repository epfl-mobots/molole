/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>
	
	Copyright (C) 2004-2008 Mobots group http://mobots.epfl.ch
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
	\defgroup oc Output Compare
	
	Wrapper around Output Compare, with a callback oriented interface.
	
	The Output Compare interrupts are not supported, but the timers one are, through the \ref timers library.
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
			Identifier of the Output Compare, from 0 to 7.
	\param	source
			Timer providing clock to the output compare. Must be \ref OC_TIMER2 or \ref OC_TIMER3.
	\param	mode
			Mode of this Output Compare. Must be one of \ref oc_modes but not \ref OC_DISABLED.
*/
void oc_enable(int oc_id, int source, int mode)
{
	if ((source < 0) || (source > 1))
		ERROR(OC_ERROR_INVALIDE_SOURCE, &source);
	
	if ((mode == OC_DISABLED) || (mode > 7))
		ERROR(OC_ERROR_INVALIDE_MODE, &mode);
	
	oc_disable(oc_id);
	
	if (source == OC_TIMER2)
		timer_set_enabled(TIMER2, false);
	else
		timer_set_enabled(TIMER3, false);
	
	switch (oc_id)
	{
		case 0: OC1CONbits.OCM = mode; OC1CONbits.OCTSEL = source; break;
		case 1: OC2CONbits.OCM = mode; OC2CONbits.OCTSEL = source; break;
		case 2: OC3CONbits.OCM = mode; OC3CONbits.OCTSEL = source; break;
		case 3: OC4CONbits.OCM = mode; OC4CONbits.OCTSEL = source; break;
		case 4: OC5CONbits.OCM = mode; OC5CONbits.OCTSEL = source; break;
		case 5: OC6CONbits.OCM = mode; OC6CONbits.OCTSEL = source; break;
		case 6: OC7CONbits.OCM = mode; OC7CONbits.OCTSEL = source; break;
		case 7: OC8CONbits.OCM = mode; OC8CONbits.OCTSEL = source; break;
		default: ERROR(OC_ERROR_INVALIDE_OC_ID, &oc_id);
	}
}

/**
	Disable an Output Compare.
	
	\param	oc_id
			Identifier of the Output Compare, from 0 to 7.
*/
void oc_disable(int oc_id)
{
	switch (oc_id)
	{
		case 0: OC1CONbits.OCM = OC_DISABLED; break;
		case 1: OC2CONbits.OCM = OC_DISABLED; break;
		case 2: OC3CONbits.OCM = OC_DISABLED; break;
		case 3: OC4CONbits.OCM = OC_DISABLED; break;
		case 4: OC5CONbits.OCM = OC_DISABLED; break;
		case 5: OC6CONbits.OCM = OC_DISABLED; break;
		case 6: OC7CONbits.OCM = OC_DISABLED; break;
		case 7: OC8CONbits.OCM = OC_DISABLED; break;
		default: ERROR(OC_ERROR_INVALIDE_OC_ID, &oc_id);
	}
}

/**
	Set the register values of an Output Compare.

	\param	oc_id
			Identifier of the Output Compare, from 0 to 7.
	\param	primary
			Output Compare register, named OCxR in the documentation.
	\param	secondary
			Secondary Output Compare register, named OCxRS in the documentation.
*/
void oc_set_value(int oc_id, unsigned primary, unsigned secondary)
{
	switch (oc_id)
	{
		case 0: OC1R = primary; OC1RS = secondary; break;
		case 1: OC2R = primary; OC2RS = secondary; break;
		case 2: OC3R = primary; OC3RS = secondary; break;
		case 3: OC4R = primary; OC4RS = secondary; break;
		case 4: OC5R = primary; OC5RS = secondary; break;
		case 5: OC6R = primary; OC6RS = secondary; break;
		case 6: OC7R = primary; OC7RS = secondary; break;
		case 7: OC8R = primary; OC8RS = secondary; break;
		default: ERROR(OC_ERROR_INVALIDE_OC_ID, &oc_id);
	}
}

/*@}*/
