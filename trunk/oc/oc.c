/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs

	Copyright (C) 2007--2011 Stephane Magnenat <stephane at magnenat dot net>,
	Philippe Retornaz <philippe dot retornaz at epfl dot ch>
	Mobots group (http://mobots.epfl.ch), Robotics system laboratory (http://lsro.epfl.ch)
	EPFL Ecole polytechnique federale de Lausanne (http://www.epfl.ch)

	See authors.txt for more details about other contributors.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, version 3 of the License.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
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

static oc_irq_cb irq_cb[8];


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
#if OC_EXIST(1)
		case OC_1: OC1CONbits.OCM = mode; OC1CONbits.OCTSEL = source; OC1CONbits.OCSIDL = 0; break;
#endif
#if OC_EXIST(2)
		case OC_2: OC2CONbits.OCM = mode; OC2CONbits.OCTSEL = source; OC2CONbits.OCSIDL = 0; break;
#endif
#if OC_EXIST(3)
		case OC_3: OC3CONbits.OCM = mode; OC3CONbits.OCTSEL = source; OC3CONbits.OCSIDL = 0; break;
#endif
#if OC_EXIST(4)
		case OC_4: OC4CONbits.OCM = mode; OC4CONbits.OCTSEL = source; OC4CONbits.OCSIDL = 0; break;
#endif
#if OC_EXIST(5)
		case OC_5: OC5CONbits.OCM = mode; OC5CONbits.OCTSEL = source; OC5CONbits.OCSIDL = 0; break;
#endif
#if OC_EXIST(6)
		case OC_6: OC6CONbits.OCM = mode; OC6CONbits.OCTSEL = source; OC6CONbits.OCSIDL = 0; break;
#endif
#if OC_EXIST(7)
		case OC_7: OC7CONbits.OCM = mode; OC7CONbits.OCTSEL = source; OC7CONbits.OCSIDL = 0; break;
#endif
#if OC_EXIST(8)
		case OC_8: OC8CONbits.OCM = mode; OC8CONbits.OCTSEL = source; OC8CONbits.OCSIDL = 0; break;
#endif
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
#if OC_EXIST(1)
		case OC_1: OC1CONbits.OCM = OC_DISABLED; break;
#endif
#if OC_EXIST(2)
		case OC_2: OC2CONbits.OCM = OC_DISABLED; break;
#endif
#if OC_EXIST(3)
		case OC_3: OC3CONbits.OCM = OC_DISABLED; break;
#endif
#if OC_EXIST(4)
		case OC_4: OC4CONbits.OCM = OC_DISABLED; break;
#endif
#if OC_EXIST(5)
		case OC_5: OC5CONbits.OCM = OC_DISABLED; break;
#endif
#if OC_EXIST(6)
		case OC_6: OC6CONbits.OCM = OC_DISABLED; break;
#endif
#if OC_EXIST(7)
		case OC_7: OC7CONbits.OCM = OC_DISABLED; break;
#endif
#if OC_EXIST(8)
		case OC_8: OC8CONbits.OCM = OC_DISABLED; break;
#endif
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
#if OC_EXIST(1)
		case OC_1: OC1R = primary; OC1RS = secondary; break;
#endif
#if OC_EXIST(2)
		case OC_2: OC2R = primary; OC2RS = secondary; break;
#endif
#if OC_EXIST(3)
		case OC_3: OC3R = primary; OC3RS = secondary; break;
#endif
#if OC_EXIST(4)
		case OC_4: OC4R = primary; OC4RS = secondary; break;
#endif
#if OC_EXIST(5)
		case OC_5: OC5R = primary; OC5RS = secondary; break;
#endif
#if OC_EXIST(6)
		case OC_6: OC6R = primary; OC6RS = secondary; break;
#endif
#if OC_EXIST(7)
		case OC_7: OC7R = primary; OC7RS = secondary; break;
#endif
#if OC_EXIST(8)
		case OC_8: OC8R = primary; OC8RS = secondary; break;
#endif
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
#if OC_EXIST(1)
		case OC_1: OC1RS = duty; break;
#endif
#if OC_EXIST(2)
		case OC_2: OC2RS = duty; break;
#endif
#if OC_EXIST(3)
		case OC_3: OC3RS = duty; break;
#endif
#if OC_EXIST(4)
		case OC_4: OC4RS = duty; break;
#endif
#if OC_EXIST(5)
		case OC_5: OC5RS = duty; break;
#endif
#if OC_EXIST(6)
		case OC_6: OC6RS = duty; break;
#endif
#if OC_EXIST(7)
		case OC_7: OC7RS = duty; break;
#endif
#if OC_EXIST(8)
		case OC_8: OC8RS = duty; break;
#endif
		default: ERROR(OC_ERROR_INVALID_OC_ID, &oc_id);
	}

}

/**
	Enable the Output Compare interrupt.

	\param	oc_id
			Identifier of the Output Compare, from \ref OC_1 to \ref OC_8.
	\param	cb
			The callback function pointer
	\param 	priority
			Interrupt priority
*/
void oc_enable_interrupt(int oc_id, oc_irq_cb cb, int priority) {
		ERROR_CHECK_RANGE(oc_id, OC_1, OC_8, OC_ERROR_INVALID_OC_ID);
		ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
		
		irq_cb[oc_id] = cb;
		switch(oc_id) {
#if OC_EXIST(1)
			case OC_1: 
				_OC1IP = priority;
				_OC1IF = 0;
				_OC1IE = 1;
				break;
#endif
#if OC_EXIST(2)
			case OC_2: 
				_OC2IP = priority;
				_OC2IF = 0;
				_OC2IE = 1;
				break;
#endif
#if OC_EXIST(3)
			case OC_3: 
				_OC3IP = priority;
				_OC3IF = 0;
				_OC3IE = 1;
				break;
#endif
#if OC_EXIST(4)
			case OC_4: 
				_OC4IP = priority;
				_OC4IF = 0;
				_OC4IE = 1;
				break;
#endif
#if OC_EXIST(5)
			case OC_5: 
				_OC5IP = priority;
				_OC5IF = 0;
				_OC5IE = 1;
				break;
#endif
#if OC_EXIST(6)
			case OC_6: 
				_OC6IP = priority;
				_OC6IF = 0;
				_OC6IE = 1;
				break;
#endif
#if OC_EXIST(7)
			case OC_7: 
				_OC7IP = priority;
				_OC7IF = 0;
				_OC7IE = 1;
				break;
#endif
#if OC_EXIST(8)
			case OC_8: 
				_OC8IP = priority;
				_OC8IF = 0;
				_OC8IE = 1;
				break;
#endif
		}
}


/**
	Disable the Output Compare interrupt.

	\param	oc_id
			Identifier of the Output Compare, from \ref OC_1 to \ref OC_8.
*/
void oc_disable_interrupt(int oc_id) {
	ERROR_CHECK_RANGE(oc_id, OC_1, OC_8, OC_ERROR_INVALID_OC_ID);
	switch(oc_id) {
#if OC_EXIST(1)
			case OC_1: 
				_OC1IE = 0;
				break;
#endif
#if OC_EXIST(2)
			case OC_2: 
				_OC2IE = 0;
				break;
#endif
#if OC_EXIST(3)
			case OC_3: 
				_OC3IE = 0;
				break;
#endif
#if OC_EXIST(4)
			case OC_4: 
				_OC4IE = 0;
				break;
#endif
#if OC_EXIST(5)
			case OC_5: 
				_OC5IE = 0;
				break;
#endif
#if OC_EXIST(6)
			case OC_6: 
				_OC6IE = 0;
				break;
#endif
#if OC_EXIST(7)
			case OC_7: 
				_OC7IE = 0;
				break;
#endif
#if OC_EXIST(8)
			case OC_8: 
				_OC8IE = 0;
				break;
#endif
		}
}

/**
	Reenable a previously enabled  Output Compare interrupt.

	\param	oc_id
			Identifier of the Output Compare, from \ref OC_1 to \ref OC_8.
*/
void oc_reenable_interrupt(int oc_id) {
	ERROR_CHECK_RANGE(oc_id, OC_1, OC_8, OC_ERROR_INVALID_OC_ID);
	switch(oc_id) {
#if OC_EXIST(1)
			case OC_1: 
				_OC1IF = 0;
				_OC1IE = 1;
				break;
#endif
#if OC_EXIST(2)
			case OC_2: 
				_OC2IF = 0;
				_OC2IE = 1;
				break;
#endif
#if OC_EXIST(3)
			case OC_3: 
				_OC3IF = 0;
				_OC3IE = 1;
				break;
#endif
#if OC_EXIST(4)
			case OC_4: 
				_OC4IF = 0;
				_OC4IE = 1;
				break;
#endif
#if OC_EXIST(5)
			case OC_5: 
				_OC5IF = 0;
				_OC5IE = 1;
				break;
#endif
#if OC_EXIST(6)
			case OC_6: 
				_OC6IF = 0;
				_OC6IE = 1;
				break;
#endif
#if OC_EXIST(7)
			case OC_7: 
				_OC7IF = 0;
				_OC7IE = 1;
				break;
#endif
#if OC_EXIST(8)
			case OC_8: 
				_OC8IF = 0;
				_OC8IE = 1;
				break;
#endif
	}
}

#if OC_EXIST(1)
void _ISR _OC1Interrupt(void) {
	_OC1IF = 0;
	
	irq_cb[0](OC_1);	
}
#endif

#if OC_EXIST(2)
void _ISR _OC2Interrupt(void) {
	_OC2IF = 0;
	
	irq_cb[1](OC_2);	
}
#endif

#if OC_EXIST(3)
void _ISR _OC3Interrupt(void) {
	_OC3IF = 0;
	
	irq_cb[2](OC_3);	
}
#endif

#if OC_EXIST(4)
void _ISR _OC4Interrupt(void) {
	_OC4IF = 0;
	
	irq_cb[3](OC_4);	
}
#endif

#if OC_EXIST(5)
void _ISR _OC5Interrupt(void) {
	_OC5IF = 0;
	
	irq_cb[4](OC_5);	
}
#endif

#if OC_EXIST(6)
void _ISR _OC6Interrupt(void) {
	_OC6IF = 0;
	
	irq_cb[5](OC_6);	
}
#endif

#if OC_EXIST(7)
void _ISR _OC7Interrupt(void) {
	_OC7IF = 0;
	
	irq_cb[6](OC_7);	
}
#endif

#if OC_EXIST(8)
void _ISR _OC8Interrupt(void) {
	_OC8IF = 0;
	
	irq_cb[7](OC_8);	
}
#endif

/*@}*/
