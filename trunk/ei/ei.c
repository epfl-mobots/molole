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
	\defgroup ei External Interrupt
	
	Wrapper around External Interrupt, with a callback oriented interface.
*/
/*@{*/

/** \file
	Implementation of the wrapper around External Interrupt.
*/

#include <p33fxxxx.h>

#include "ei.h"
#include "../types/types.h"
#include "../error/error.h"



/** External Interrupt user data passed to the callback */
static void * udata[EI_MAX + 1];

/** Callback to the user-specified function */
static ei_callback cb[EI_MAX + 1];
 

#define GENERATE_EI_INIT(ei) if(ei_id == EI_## ei ) { \
									_INT## ei ##EP = polarity; \
									_INT## ei ##IF = 0; \
									_INT## ei ##IE = 0; /* Do not enable it */}
#define GENERATE_EI_DISABLE(ei) if(ei_id == EI_ ## ei ) { \
									_INT## ei ##IE = 0; }

#define GENERATE_EI_IRQ_HANDLER(ei) void _ISR _INT## ei ##Interrupt(void) { _INT## ei ##IF = 0; \
																		cb[ ei ](EI_## ei , udata[ ei ]);}


#define GENERATE_EI_ENABLE(ei) if(ei_id == EI_ ## ei ) { \
									_INT## ei ##IF = 0; \
									_INT## ei ##IE = 1; }



//-------------------
// Exported functions
//-------------------
/**
	Initialise an External Interrupt.
	
	\param	ei_id
			Identifier of the External Interrupt. Must be one of \ref ei_identifiers.
	\param	polarity
			Polarity of the external interrupt signal. Must be \ref EI_POSITIVE_EDGE or \ref EI_NEGATIVE_EDGE.
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 7 (highest priority)
*/
void ei_init(int ei_id, int polarity, int priority) {
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	ERROR_CHECK_RANGE(ei_id, EI_MIN, EI_MAX, EI_INVALID_ID);
	ERROR_CHECK_RANGE(polarity, EI_POSITIVE_EDGE, EI_NEGATIVE_EDGE, EI_INVALID_POLARITY);

#ifdef _INT0EP
	GENERATE_EI_ENABLE(0)
#endif
#ifdef _INT1EP
	GENERATE_EI_ENABLE(1)
#endif
#ifdef _INT2EP
	GENERATE_EI_ENABLE(2)
#endif
#ifdef _INT3EP
	GENERATE_EI_ENABLE(3)
#endif
#ifdef _INT4EP
	GENERATE_EI_ENABLE(4)
#endif
#ifdef _INT5EP
	GENERATE_EI_ENABLE(5)
#endif
#ifdef _INT6EP
	GENERATE_EI_ENABLE(6)
#endif
#ifdef _INT7EP
	GENERATE_EI_ENABLE(7)
#endif
}


/**
	Enable an External Interrupt.
	
	\param	ei_id
			Identifier of the External Interrupt. Must be one of \ref ei_identifiers.
	\param	callback
			User-specified function to call when an interrupt is triggered.
	\param 	user_data
			User data passed as callback argument
*/
void ei_enable(int ei_id, ei_callback callback, void * user_data) {
	ERROR_CHECK_RANGE(ei_id, EI_MIN, EI_MAX, EI_INVALID_ID);

	cb[ei_id] = callback;
	udata[ei_id] = user_data;

#ifdef _INT0EP
	GENERATE_EI_ENABLE(0)
#endif
#ifdef _INT1EP
	GENERATE_EI_ENABLE(1)
#endif
#ifdef _INT2EP
	GENERATE_EI_ENABLE(2)
#endif
#ifdef _INT3EP
	GENERATE_EI_ENABLE(3)
#endif
#ifdef _INT4EP
	GENERATE_EI_ENABLE(4)
#endif
#ifdef _INT5EP
	GENERATE_EI_ENABLE(5)
#endif
#ifdef _INT6EP
	GENERATE_EI_ENABLE(6)
#endif
#ifdef _INT7EP
	GENERATE_EI_ENABLE(7)
#endif
}


/**
	Disable an External Interrupt.
	
	\param	ei_id
			Identifier of the External Interrupt. Must be one of \ref ei_identifiers.
*/
void ei_disable(int ei_id) {
	ERROR_CHECK_RANGE(ei_id, EI_MIN, EI_MAX, EI_INVALID_ID);
#ifdef _INT0EP
	GENERATE_EI_DISABLE(0)
#endif
#ifdef _INT1EP
	GENERATE_EI_DISABLE(1)
#endif
#ifdef _INT2EP
	GENERATE_EI_DISABLE(2)
#endif
#ifdef _INT3EP
	GENERATE_EI_DISABLE(3)
#endif
#ifdef _INT4EP
	GENERATE_EI_DISABLE(4)
#endif
#ifdef _INT5EP
	GENERATE_EI_DISABLE(5)
#endif
#ifdef _INT6EP
	GENERATE_EI_DISABLE(6)
#endif
#ifdef _INT7EP
	GENERATE_EI_DISABLE(7)
#endif
}

//--------------------------
// Interrupt service routine
//--------------------------

#ifdef _INT0EP
	GENERATE_EI_IRQ_HANDLER(0)
#endif
#ifdef _INT1EP
	GENERATE_EI_IRQ_HANDLER(1)
#endif
#ifdef _INT2EP
	GENERATE_EI_IRQ_HANDLER(2)
#endif
#ifdef _INT3EP
	GENERATE_EI_IRQ_HANDLER(3)
#endif
#ifdef _INT4EP
	GENERATE_EI_IRQ_HANDLER(4)
#endif
#ifdef _INT5EP
	GENERATE_EI_IRQ_HANDLER(5)
#endif
#ifdef _INT6EP
	GENERATE_EI_IRQ_HANDLER(6)
#endif
#ifdef _INT7EP
	GENERATE_EI_IRQ_HANDLER(7)
#endif






