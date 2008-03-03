/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	
	Copyright (C) 2007 - 2008 Stephane Magnenat <stephane at magnenat dot net>,
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

#ifndef _MOLOLE_TYPES_H
#define _MOLOLE_TYPES_H

#include <p33fxxxx.h>

/**
	\defgroup types Types
	
	Some additional basic datatypes
*/
/*@{*/

/** \file
	\brief Definition of some additional basic datatypes.
*/

//! Boolean type for convenience
typedef int bool;

//! Boolean literals for convenience
enum bool_literals
{
	false = 0,	//!< truth literal
	true = 1	//!< false literal
};


//! Force GCC to not reorder the instruction before and after this macro 
#define barrier() __asm__ __volatile__("": : :"memory")

//! Save current interrupt priority level into flags and disable interrupts
#define IRQ_DISABLE(flags) 	do { \
								flags = SRbits.IPL; \
								SRbits.IPL = 7; \
								barrier();\
							} while(0)

//! Re-enable interrupts at interrupt priority level as in flags
#define IRQ_ENABLE(flags) 	do { \
								SRbits.IPL = flags; \
								barrier(); \
							 } while(0)

/*@}*/

#endif
