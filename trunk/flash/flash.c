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

#include "../types/types.h"

#include "flash.h"
#include "../error/error.h"


#define ERASE               0x4042
#define PROGRAM_ROW         0x4001
#define PROGRAM_WORD        0x4003



#define tblwtl(offset, data) do { \
			asm __volatile__ ("tblwtl %[d], [%[o]]" : /* No output */ \
												   : [d] "r" (data) , [o] "r" (offset)); \
								} while(0)
								
#define tblwth(offset, data) do { \
			asm __volatile__ ("tblwth %[d], [%[o]]" : /* No output */ \
												   : [d] "r" (data) , [o] "r" (offset)); \
								} while(0)

static unsigned int row_counter = 0;
static unsigned long current_addr = 0;
static unsigned int saved_tblpag;


static void  do_key_seq(void) {
	unsigned int flags;
	// Disable IRQ
	SET_AND_SAVE_CPU_IPL(flags, 7);
	
	asm __volatile__ (
	"disi    #5\n"
    "mov     #0x55,W0\n"
    "mov     W0, NVMKEY\n"
    "mov     #0xAA, W0\n"
    "mov     W0, NVMKEY\n"
    "bset    NVMCON, #15\n"
    "nop\n"
    "nop\n"
    "btsc    NVMCON, #15\n"
    "bra     $-2\n"
     : /* No output */
     : /* No input */ 
     : "w0", "cc", "memory");
     
     // Reenable IRQ
     RESTORE_CPU_IPL(flags);
     
     return
	;
}


unsigned long flash_read_instr(unsigned long addr) {
	unsigned int tmp;
	unsigned int high, low;
	tmp = TBLPAG;
	
	TBLPAG = addr >> 16;
	high = __builtin_tblrdh(addr & 0xFFFF);
	low = __builtin_tblrdl(addr & 0xFFFF);
	
	TBLPAG = tmp;
	
	return (unsigned long) high << 16 | low;
}

unsigned int flash_read_low(unsigned long addr) {
	unsigned int tmp;
	unsigned int low;
	tmp = TBLPAG;
	
	TBLPAG = addr >> 16;
	low = __builtin_tblrdl(addr & 0xFFFF);
	
	TBLPAG = tmp;
	
	return low;
}

unsigned char flash_read_high(unsigned long addr) {
	unsigned int tmp;
	unsigned int high;
	tmp = TBLPAG;
	
	TBLPAG = addr >> 16;
	high = __builtin_tblrdh(addr & 0xFFFF);
	
	TBLPAG = tmp;
	
	return high;
}

static void _frc_nc(unsigned long addr, size_t size, unsigned char * buffer) {
	unsigned int tmp;
	unsigned int data;
	size_t i;
	size_t instr = size / 3;
	size_t left = size % 3;
	
	tmp = TBLPAG;
	TBLPAG = addr >> 16;
	
	for(i = 0; i < instr; i++, addr+=2) {
		data = __builtin_tblrdl(addr & 0xFFFF);
		*buffer++ = data & 0xFF;
		*buffer++ = data >> 8;
		data =  __builtin_tblrdh(addr & 0xFFFF);
		*buffer++ = data & 0xFF;
	}
	if(left >= 1) {
		data = __builtin_tblrdl(addr & 0xFFFF);
		*buffer++ = data & 0xFF;
		if(left >= 2) 
			*buffer = data >> 8;
	}
	TBLPAG = tmp;
}

void flash_read_chunk(unsigned long addr, size_t size, unsigned char * buffer) {
	do {
		unsigned long next_tbl = ((addr >> 16) + 1) << 16;
		unsigned long bytes_nt = ((next_tbl - addr) / 2 ) * 3;
		unsigned long min = bytes_nt < size ? bytes_nt : size;
		_frc_nc(addr, min, buffer);
		addr += min / 3 * 2;
		buffer += min;
		size -= min;
	} while(size);
}

void flash_erase_page(unsigned long addr) {
	unsigned int tmp;

	if(addr & (INSTRUCTIONS_PER_PAGE * 2 - 1)) 
		ERROR(FLASH_UNALIGNED_ADDRESS, &addr);
	
	tmp = TBLPAG;
	TBLPAG = addr >> 16;
	
	NVMCON = ERASE;
	
	tblwtl(addr & 0xFFFF, addr & 0xFFFF);
	
	do_key_seq();
	
	TBLPAG = tmp;
}

void flash_flash_instr(unsigned long addr, unsigned long data) {
	unsigned int tmp;
	
	if(addr & 0x1)
		ERROR(FLASH_UNALIGNED_ADDRESS, &addr);
	
	tmp = TBLPAG;
	TBLPAG = addr >> 16;
	NVMCON = PROGRAM_WORD;
	
	tblwtl(addr & 0xFFFF, data & 0xFFFF);
	tblwth(addr & 0xFFFF, data >> 16);
	
	do_key_seq();
	
	TBLPAG = tmp;
}

static unsigned long _errata_latch_d[4];

void flash_prepare_write(unsigned long addr) {
	if(addr & (INSTRUCTIONS_PER_ROW * 2 - 1)) 
		ERROR(FLASH_UNALIGNED_ADDRESS, &addr);
	current_addr = addr;
	row_counter = 0;
	
	saved_tblpag = TBLPAG;
	TBLPAG = addr >> 16;
	NVMCON = PROGRAM_ROW;
}
void __fixup_errata(void) {
	int i;
	unsigned long fixup_addr = current_addr - 128; // Get the row base address
	fixup_addr |= 0x18; // errata address

	for(i = 0; i < 4; i++) {
		tblwtl((fixup_addr | (i << 5)) & 0xFFFF, _errata_latch_d[i] & 0xFFFF);
		tblwth((fixup_addr | (i << 5)) & 0xFFFF, _errata_latch_d[i] >> 16);
	}
}

void flash_write_instruction(unsigned long data) {
	if((current_addr & 0x1F) == 0x18) 
		_errata_latch_d[(current_addr >> 5) & 0x3] = data;
	
	tblwtl(current_addr & 0xFFFF, data & 0xFFFF);
	tblwth(current_addr & 0xFFFF, data >> 16);
	current_addr += 2;
	row_counter++;
	if(row_counter == INSTRUCTIONS_PER_ROW) {
		__fixup_errata();
		do_key_seq(); 
		TBLPAG = current_addr >> 16; 
		NVMCON = PROGRAM_ROW;
		row_counter = 0;
	}
}

void flash_write_buffer(unsigned char * data, size_t size) {
	size_t i;
	size_t instr = size / 3;
	size_t left = size % 3;
	
	for( i = 0; i < instr; i++, data += 3)
		flash_write_instruction(data[0] | (unsigned long) data[1] << 8 |  (unsigned long) data[2] << 16);
	
	if(left == 1) 
		flash_write_instruction(data[0]);
	if(left == 2)
		flash_write_instruction(data[0] | ((unsigned long) data[1] << 8));
	
}


void flash_complete_write(void) {
	while(row_counter) 
		flash_write_instruction(0xFFFFFFFF);
	TBLPAG = saved_tblpag;
}

