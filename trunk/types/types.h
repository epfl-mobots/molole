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


enum irq_prio
{
	IRQ_PRIO_MIN = 1,			//!< Lowest available priority
	IRQ_PRIO_1 = IRQ_PRIO_MIN,  
	IRQ_PRIO_2,
	IRQ_PRIO_3,
	IRQ_PRIO_4,
	IRQ_PRIO_5,
	IRQ_PRIO_6,
	IRQ_PRIO_MAX = IRQ_PRIO_6, //!< Highest available priority
	IRQ_PRIO_NMI = 7		   //!< This level is for non masquable interrupts. Even when IRQ_DISABLE() is used, this interrupt can run
};	

//! Force GCC to not reorder the instruction before and after this macro 
#define barrier() __asm__ __volatile__("": : :"memory")


//! Return the number of byte availabe on the stack
#define get_stack_space() ({ SPLIM - *((volatile int *) 0x1E); })
								
//! Set the current Interrupt priority level. Warning, use it only if you really know what you're doing.		
#define SET_IPL(ipl) do { \
						SRbits.IPL = ipl; \
						barrier(); \
					} while(0)

//! Save current interrupt priority level into flags and disable interrupts
#define IRQ_DISABLE(flags) 	do { \
								flags = SRbits.IPL; \
								SRbits.IPL = IRQ_PRIO_MAX; \
								barrier();\
							} while(0)

//! Re-enable interrupts at interrupt priority level as in flags
#define IRQ_ENABLE(flags) 	do { \
								SRbits.IPL = flags; \
								barrier(); \
							 } while(0)

//! Save current interrupt priority level into flags and disable interrupts. WARNING ! even NMI interrupts are disabled
#define IRQ_DISABLE_NMI_I_KNOW_WHAT_I_M_DOING(flags) do { \
														flags = SRbits.IPL; \
														SRbits.IPL = IRQ_PRIO_NMI; \
														barrier();\
													 } while(0)
//! Raise SRbits.IPL to "ipl" if lower do nothing
#define RAISE_IPL(flags, ipl) do{ \
								flags = SRbits.IPL; \
								if(flags < ipl) \
									SET_IPL(ipl); \
								barrier(); \
							} while(0)											

#ifndef NULL
#define NULL ((void *) 0)
#endif


/** Atomic and operation to prevent race conditions inside interrupts: *x = (*x) & y */
#define atomic_and(x,y) do { __asm__ volatile ("and.w %[yy], [%[xx]], [%[xx]]": : [xx] "r" (x), [yy] "r"(y): "cc","memory"); } while(0)
/** Atomic or operation to prevent race conditions inside interrupts: *x = (*x) | y */
#define atomic_or(x,y) do { __asm__ volatile ("ior.w %[yy], [%[xx]], [%[xx]]" : : [xx] "r" (x), [yy] "r"(y): "cc","memory"); } while(0)


/*@}*/

#endif
