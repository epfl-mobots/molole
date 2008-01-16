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
	\defgroup DMA
	
	Wrapper around DMA, with a callback oriented interface.
	
	Device specific configurations are implemented in their respective modules.
*/
/*@{*/

/** \file
	Implementation of the wrapper around dsPIC33 DMA.
*/


//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "dma.h"
#include "../error/error.h"

/*
switch (channel)
	{
		case DMA_CHANNEL_0: break;
		case DMA_CHANNEL_1: break;
		case DMA_CHANNEL_2: break;
		case DMA_CHANNEL_3: break;
		case DMA_CHANNEL_4: break;
		case DMA_CHANNEL_5: break;
		case DMA_CHANNEL_6: break;
		case DMA_CHANNEL_7: break;
		default: ERROR(DMA_ERROR_INVALID_CHANNEL, &channel);
	}
*/

//-------------------
// Exported functions
//-------------------

void dma_init_channel(int channel, int request_source, int data_size, int transfer_dir, int interrupt_pos, int null_write, int addressing_mode, int operating_mode, unsigned offset_a, unsigned offset_b, void* peripheral_address, unsigned transfer_count, dma_callback callback)
{
	ERROR(GENERIC_ERROR_NOT_IMPLEMENTED, 0);

	// TODO: validate request source
	switch (channel)
	{
		case DMA_CHANNEL_0:
		{
			// first disable current transfers
			DMA0CONbits.CHEN = 0;
			
			DMA0REQbits.IRQSEL = request_source;
			DMA0CONbits.SIZE = data_size;
			DMA0CONbits.DIR = transfer_dir;
			DMA0CONbits.HALF = interrupt_pos;
			DMA0CONbits.NULLW = null_write;
			DMA0CONbits.AMODE = addressing_mode;
			DMA0CONbits.MODE = operating_mode;
			
			DMA0STA = offset_a;
			DMA0STB = offset_b;
			DMA0PAD = (volatile unsigned int)peripheral_address;
			DMA0CNT = transfer_count;
			
			// TODO: interrupt
			/*IFS0bits.DMA0IF = 0;
			IEC0bits.DMA0IE = 1;*/
		}
		break;
		
		// TODO: copy paste to others
		
		default: ERROR(DMA_ERROR_INVALID_CHANNEL, &channel);
	};
	// TODO
}

void dma_enable_channel(int channel)
{
	switch (channel)
	{
		case DMA_CHANNEL_0: DMA0CONbits.CHEN  = 1; break;
		case DMA_CHANNEL_1: DMA1CONbits.CHEN  = 1; break;
		case DMA_CHANNEL_2: DMA2CONbits.CHEN  = 1; break;
		case DMA_CHANNEL_3: DMA3CONbits.CHEN  = 1; break;
		case DMA_CHANNEL_4: DMA4CONbits.CHEN  = 1; break;
		case DMA_CHANNEL_5: DMA5CONbits.CHEN  = 1; break;
		case DMA_CHANNEL_6: DMA6CONbits.CHEN  = 1; break;
		case DMA_CHANNEL_7: DMA7CONbits.CHEN  = 1; break;
		default: ERROR(DMA_ERROR_INVALID_CHANNEL, &channel);
	}
}

void dma_disable_channel(int channel)
{
	switch (channel)
	{
		case DMA_CHANNEL_0: DMA0CONbits.CHEN  = 0; break;
		case DMA_CHANNEL_1: DMA1CONbits.CHEN  = 0; break;
		case DMA_CHANNEL_2: DMA2CONbits.CHEN  = 0; break;
		case DMA_CHANNEL_3: DMA3CONbits.CHEN  = 0; break;
		case DMA_CHANNEL_4: DMA4CONbits.CHEN  = 0; break;
		case DMA_CHANNEL_5: DMA5CONbits.CHEN  = 0; break;
		case DMA_CHANNEL_6: DMA6CONbits.CHEN  = 0; break;
		case DMA_CHANNEL_7: DMA7CONbits.CHEN  = 0; break;
		default: ERROR(DMA_ERROR_INVALID_CHANNEL, &channel);
	}
}

void dma_start_transfer(int channel)
{
	switch (channel)
	{
		case DMA_CHANNEL_0: DMA0REQbits.FORCE = 1; break;
		case DMA_CHANNEL_1: DMA1REQbits.FORCE = 1; break;
		case DMA_CHANNEL_2: DMA2REQbits.FORCE = 1; break;
		case DMA_CHANNEL_3: DMA3REQbits.FORCE = 1; break;
		case DMA_CHANNEL_4: DMA4REQbits.FORCE = 1; break;
		case DMA_CHANNEL_5: DMA5REQbits.FORCE = 1; break;
		case DMA_CHANNEL_6: DMA6REQbits.FORCE = 1; break;
		case DMA_CHANNEL_7: DMA7REQbits.FORCE = 1; break;
		default: ERROR(DMA_ERROR_INVALID_CHANNEL, &channel);
	}
}

//--------------------------
// Interrupt service routine
//--------------------------

// TODO


/*@}*/
