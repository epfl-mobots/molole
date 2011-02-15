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

#ifndef _MOLOLE_CN_H
#define _MOLOLE_CN_H

#include "../types/types.h"

/** \addtogroup cn */
/*@{*/

/** \file
	Change notification wrapper definitions
*/

// Defines

/** change notification callback on interrupt */
typedef void (*cn_callback)(void);

enum cn_errors
{
	CN_ERROR_BASE = 0x1200,
	CN_ERROR_PU_AND_PD,				/**< Both pullup and pulldown have been selected. */
	CN_ERROR_INVALID_CHANNEL,			/**< This channel is not available on the current architecture. */
};

// Functions, doc in the .c

void cn_init(unsigned long interrupt_mask, unsigned long pull_up_mask, cn_callback callback, int priority);
void cn_add_notification(unsigned int channel, bool pullup, bool pulldown);
void cn_remove_notification(unsigned int channel);

void cn_enable_interrupt(void);
void cn_disable_interrupt(void);


/*@}*/

#endif
