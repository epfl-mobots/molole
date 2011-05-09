

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


#ifndef _MOLOLE_FLASH_H
#define _MOLOLE_FLASH_H

#include "../types/types.h"
#include <stdlib.h>

/** \addtogroup flash */
/*@{*/

/** \file
	Flash access definitions
*/

/** Errors Flash module can throw */
enum flash_errors
{
	FLASH_ERROR_BASE = 0x0E00,
	FLASH_UNALIGNED_ADDRESS,
	FLASH_BAD_SIZE,
};

#define INSTRUCTIONS_PER_PAGE  512
#define INSTRUCTIONS_PER_ROW   64

// Read the whole 24 bit word
unsigned long flash_read_instr(unsigned long addr);

// Read the 16 low bits
unsigned int flash_read_low(unsigned long addr);

// Read the 8 higer bits
unsigned char flash_read_high(unsigned long addr);

// Read each 24 bits word and put it in a buffer (no "phantom" 8 higher bits)
void flash_read_chunk(unsigned long addr, size_t size, unsigned char * buffer);

// Erase the page pointed by addr 
void flash_erase_page(unsigned long addr);

// Prepare a write operation, must be completed with "flash_complete_write();
// ONLY "WRITE" call are allowed between a prepare and a complete !
void flash_prepare_write(unsigned long addr);

// Write an instruction at the current writing address.
void flash_write_instruction(unsigned long data);

// write a buffer filled with 24 bits word data (no "Phantom" 8 higher bits)
void flash_write_buffer(unsigned char * data, size_t size);

// Finish a write. Flush all buffers etc ... 
void flash_complete_write(void);

#endif
