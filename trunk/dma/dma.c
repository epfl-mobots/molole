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
	\defgroup dma DMA
	
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

//-----------------------
// Structures definitions
//-----------------------

/** DMA wrapper data */
static dma_callback DMA_Data[8];


//-------------------
// Privates functions 
//-------------------

static unsigned int get_offset(void * addr)
{
	unsigned int offset;
	/* Special handling for addr == 0. It mean "Do not use this buffer" */
	if(!addr)
		return 0;
	offset = ((unsigned int) addr) - ((unsigned int) &_DMA_BASE);
	if(offset > ((unsigned int) addr) || offset >  ((unsigned int) &_DMA_BASE) + 0x2000)
	{
		ERROR(DMA_ERROR_INVALID_ADDRESS, &addr)
	}
	return offset;
}


//-------------------
// Exported functions
//-------------------

/**
	Configure a DMA channel.
	
	This function disable the channel if it was previously enabled, but does not re-enable it.
	You must call dma_enable_channel() after this call to do so.
	
	\param	channel
			DMA channel, from \ref DMA_CHANNEL_0 to \ref DMA_CHANNEL_7.
	\param	request_source
			Source of requests that can initiate DMA, one of \ref dma_requests_sources
	\param	data_size
			Size of data to transfer, one of \ref dma_data_sizes
	\param	transfer_dir
			Direction of transfer, one of \ref dma_transfer_direction
	\param	interrupt_pos
			Should DMA interrupt happens at half of transfer or when it is completed?, one of \ref dma_interrupt_position
	\param	null_write
			Should DMA write null to peripheral when writing doto to DPSRAM?, one of \ref dma_null_data_peripheral_write_mode_select
	\param	addressing_mode
			DMA Channel Addressing Mode, one of \ref dma_addressing_mode
	\param	operating_mode
			DMA Channel Operating Mode, one of \ref dma_operating_mode
	\param	a
			Buffer A inside the DMA memory.
	\param	b
			Buffer B inside the DMA memory.
	\param	peripheral_address
			Address of the peripheral, must be suitable for DMA.
	\param	transfer_count
			Amount of data (in unit of 1 or 2 bytes depending on data_size) per transfer
	\param	callback
			User-specified function to call when a buffer is half or fully filled (depends on dma_interrupt_position).
			If 0, DMA interrupt is disabled
*/
void dma_init_channel(int channel, int request_source, int data_size, int transfer_dir, int interrupt_pos, int null_write, int addressing_mode, int operating_mode, void * a, void * b, void* peripheral_address, unsigned transfer_count, dma_callback callback)
{
	// validate arguments
	if (request_source != DMA_INTERRUPT_SOURCE_INT_0 &&
		request_source != DMA_INTERRUPT_SOURCE_IC_1 &&
		request_source != DMA_INTERRUPT_SOURCE_OC_1 &&
		request_source != DMA_INTERRUPT_SOURCE_IC_2 &&
		request_source != DMA_INTERRUPT_SOURCE_OC_2 &&
		request_source != DMA_INTERRUPT_SOURCE_TIMER_2 &&
		request_source != DMA_INTERRUPT_SOURCE_TIMER_3 &&
		request_source != DMA_INTERRUPT_SOURCE_SPI_1 &&
		request_source != DMA_INTERRUPT_SOURCE_UART_1_RX &&
		request_source != DMA_INTERRUPT_SOURCE_UART_1_TX &&
		request_source != DMA_INTERRUPT_SOURCE_ADC_1 &&
		request_source != DMA_INTERRUPT_SOURCE_ADC_2 &&
		request_source != DMA_INTERRUPT_SOURCE_UART_2_RX &&
		request_source != DMA_INTERRUPT_SOURCE_UART_2_TX &&
		request_source != DMA_INTERRUPT_SOURCE_SPI_2 &&
		request_source != DMA_INTERRUPT_SOURCE_ECAN_1_RX &&
		request_source != DMA_INTERRUPT_SOURCE_ECAN_2_RX &&
		request_source != DMA_INTERRUPT_SOURCE_DCI &&
		request_source != DMA_INTERRUPT_SOURCE_ECAN_1_TX &&
		request_source != DMA_INTERRUPT_SOURCE_ECAN_2_TX
	)
		ERROR(DMA_ERROR_INVALID_REQUEST_SOURCE, &request_source);
	
	ERROR_CHECK_RANGE(data_size, 0, 1, DMA_ERROR_INVALID_DATA_SIZE);
	ERROR_CHECK_RANGE(transfer_dir , 0, 1, DMA_ERROR_INVALID_TRANSFER_DIRECTION);
	ERROR_CHECK_RANGE(interrupt_pos, 0, 1, DMA_ERROR_INVALID_INTERRUPT_POSITION);
	ERROR_CHECK_RANGE(null_write, 0, 1, DMA_ERROR_INVALID_WRITE_NULL_MODE);
	ERROR_CHECK_RANGE(addressing_mode, 0, 2, DMA_ERROR_INVALID_ADDRESSING_MODE);
	ERROR_CHECK_RANGE(operating_mode, 0, 3, DMA_ERROR_INVALID_OPERATING_MODE);
	
	// Work around Errata 38 at the expense of energy consumption
	if ((operating_mode == DMA_OPERATING_ONE_SHOT) || (operating_mode == DMA_OPERATING_ONE_SHOT_PING_PONG))
		clock_disable_idle();
	
	// setup DMA
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
			
			
			DMA0STA = get_offset(a);
			DMA0STB = get_offset(b);
			DMA0PAD = (volatile unsigned int)peripheral_address;
			DMA0CNT = transfer_count - 1;
			
			// enable interrupt if a callback function is provided
			_DMA0IF = 0;
			if (callback)
			{
				DMA_Data[0] = callback;
				_DMA0IE = 1;
			}
			else
				_DMA0IE = 0;
		}
		break;
		
		case DMA_CHANNEL_1:
		{
			// first disable current transfers
			DMA1CONbits.CHEN = 0;
			
			DMA1REQbits.IRQSEL = request_source;
			DMA1CONbits.SIZE = data_size;
			DMA1CONbits.DIR = transfer_dir;
			DMA1CONbits.HALF = interrupt_pos;
			DMA1CONbits.NULLW = null_write;
			DMA1CONbits.AMODE = addressing_mode;
			DMA1CONbits.MODE = operating_mode;
			
			DMA1STA = get_offset(a);
			DMA1STB = get_offset(b);
			DMA1PAD = (volatile unsigned int)peripheral_address;
			DMA1CNT = transfer_count - 1;
			
			// enable interrupt if a callback function is provided
			_DMA1IF = 0;
			if (callback)
			{
				DMA_Data[1] = callback;
				_DMA1IE = 1;
			}
			else
				_DMA1IE = 0;
		}
		break;
		
		case DMA_CHANNEL_2:
		{
			// first disable current transfers
			DMA2CONbits.CHEN = 0;
			
			DMA2REQbits.IRQSEL = request_source;
			DMA2CONbits.SIZE = data_size;
			DMA2CONbits.DIR = transfer_dir;
			DMA2CONbits.HALF = interrupt_pos;
			DMA2CONbits.NULLW = null_write;
			DMA2CONbits.AMODE = addressing_mode;
			DMA2CONbits.MODE = operating_mode;
			
			DMA2STA = get_offset(a);
			DMA2STB = get_offset(b);
			DMA2PAD = (volatile unsigned int)peripheral_address;
			DMA2CNT = transfer_count - 1;
			
			// enable interrupt if a callback function is provided
			_DMA2IF = 0;
			if (callback)
			{
				DMA_Data[2] = callback;
				_DMA2IE = 1;
			}
			else
				_DMA2IE = 0;
		}
		break;
		
		case DMA_CHANNEL_3:
		{
			// first disable current transfers
			DMA3CONbits.CHEN = 0;
			
			DMA3REQbits.IRQSEL = request_source;
			DMA3CONbits.SIZE = data_size;
			DMA3CONbits.DIR = transfer_dir;
			DMA3CONbits.HALF = interrupt_pos;
			DMA3CONbits.NULLW = null_write;
			DMA3CONbits.AMODE = addressing_mode;
			DMA3CONbits.MODE = operating_mode;
			
			DMA3STA = get_offset(a);
			DMA3STB = get_offset(b);
			DMA3PAD = (volatile unsigned int)peripheral_address;
			DMA3CNT = transfer_count - 1;
			
			// enable interrupt if a callback function is provided
			_DMA3IF = 0;
			if (callback)
			{
				DMA_Data[3] = callback;
				_DMA3IE = 1;
			}
			else
				_DMA3IE = 0;
		}
		break;
		
		case DMA_CHANNEL_4:
		{
			// first disable current transfers
			DMA4CONbits.CHEN = 0;
			
			DMA4REQbits.IRQSEL = request_source;
			DMA4CONbits.SIZE = data_size;
			DMA4CONbits.DIR = transfer_dir;
			DMA4CONbits.HALF = interrupt_pos;
			DMA4CONbits.NULLW = null_write;
			DMA4CONbits.AMODE = addressing_mode;
			DMA4CONbits.MODE = operating_mode;
			
			DMA4STA = get_offset(a);
			DMA4STB = get_offset(b);
			DMA4PAD = (volatile unsigned int)peripheral_address;
			DMA4CNT = transfer_count - 1;
			
			// enable interrupt if a callback function is provided
			_DMA4IF = 0;
			if (callback)
			{
				DMA_Data[4] = callback;
				_DMA4IE = 1;
			}
			else
				_DMA4IE = 0;
		}
		break;
		
		case DMA_CHANNEL_5:
		{
			// first disable current transfers
			DMA5CONbits.CHEN = 0;
			
			DMA5REQbits.IRQSEL = request_source;
			DMA5CONbits.SIZE = data_size;
			DMA5CONbits.DIR = transfer_dir;
			DMA5CONbits.HALF = interrupt_pos;
			DMA5CONbits.NULLW = null_write;
			DMA5CONbits.AMODE = addressing_mode;
			DMA5CONbits.MODE = operating_mode;
			
			DMA5STA = get_offset(a);
			DMA5STB = get_offset(b);
			DMA5PAD = (volatile unsigned int)peripheral_address;
			DMA5CNT = transfer_count - 1;
			
			// enable interrupt if a callback function is provided
			_DMA5IF = 0;
			if (callback)
			{
				DMA_Data[0] = callback;
				_DMA5IE = 1;
			}
			else
				_DMA5IE = 0;
		}
		break;
		
		case DMA_CHANNEL_6:
		{
			// first disable current transfers
			DMA6CONbits.CHEN = 0;
		
			DMA6REQbits.IRQSEL = request_source;
			DMA6CONbits.SIZE = data_size;
			DMA6CONbits.DIR = transfer_dir;
			DMA6CONbits.HALF = interrupt_pos;
			DMA6CONbits.NULLW = null_write;
			DMA6CONbits.AMODE = addressing_mode;
			DMA6CONbits.MODE = operating_mode;
			
			DMA6STA = get_offset(a);
			DMA6STB = get_offset(b);
			DMA6PAD = (volatile unsigned int)peripheral_address;
			DMA6CNT = transfer_count - 1;
			
			// enable interrupt if a callback function is provided
			_DMA6IF = 0;
			if (callback)
			{
				DMA_Data[6] = callback;
				_DMA6IE = 1;
			}
			else
				_DMA6IE = 0;
		}
		break;
		
		case DMA_CHANNEL_7:
		{
			// first disable current transfers
			DMA7CONbits.CHEN = 0;
			
			DMA7REQbits.IRQSEL = request_source;
			DMA7CONbits.SIZE = data_size;
			DMA7CONbits.DIR = transfer_dir;
			DMA7CONbits.HALF = interrupt_pos;
			DMA7CONbits.NULLW = null_write;
			DMA7CONbits.AMODE = addressing_mode;
			DMA7CONbits.MODE = operating_mode;
			
			DMA7STA = get_offset(a);
			DMA7STB = get_offset(b);
			DMA7PAD = (volatile unsigned int)peripheral_address;
			DMA7CNT = transfer_count - 1;
			
			// enable interrupt if a callback function is provided
			_DMA7IF = 0;
			if (callback)
			{
				DMA_Data[7] = callback;
				_DMA7IE = 1;
			}
			else
				_DMA7IE = 0;
		}
		break;
		
		default: ERROR(DMA_ERROR_INVALID_CHANNEL, &channel);
	};
}

/**
	Enable a DMA channel.
	
	You must call dma_init_channel() with the same channel number prior any call to this function.
	This function does not start any transfer; Transfers are started by interrupts from peripherals
	(see dma_requests_sources) or manually by calling dma_start_transfer() after this call.
	
	\param	channel
			DMA channel, from \ref DMA_CHANNEL_0 to \ref DMA_CHANNEL_7.
*/
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

/**
	Disable a DMA channel.
	
	\param	channel
			DMA channel, from \ref DMA_CHANNEL_0 to \ref DMA_CHANNEL_7.
*/
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

/**
	Manually start transfer on a DMA channel
	
	You must call dma_init_channel() with the same channel number prior any call to this function.
	
	\param	channel
			DMA channel, from \ref DMA_CHANNEL_0 to \ref DMA_CHANNEL_7.
*/
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

/**
	DMA 0 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR  _DMA0Interrupt(void)
{
	static int dmaBuffer = 0;

	// Clear interrupt flag
	_DMA0IF = 0;	

	// Call use-defined function with true as argument if first buffer, false if second buffer
	DMA_Data[0](DMA_CHANNEL_0, dmaBuffer == 0);
	dmaBuffer ^= 1;
	
}

/**
	DMA 1 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR  _DMA1Interrupt(void)
{
	static int dmaBuffer = 0;

	// Clear interrupt flag
	_DMA1IF = 0;
	
	// Call use-defined function with true as argument if first buffer, false if second buffer
	DMA_Data[1](DMA_CHANNEL_1, dmaBuffer == 0);
	dmaBuffer ^= 1;
}

/**
	DMA 2 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR  _DMA2Interrupt(void)
{
	static int dmaBuffer = 0;
	
	// Clear interrupt flag
	_DMA2IF = 0;

	// Call use-defined function with true as argument if first buffer, false if second buffer
	DMA_Data[2](DMA_CHANNEL_2, dmaBuffer == 0);
	dmaBuffer ^= 1;
}

/**
	DMA 3 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR  _DMA3Interrupt(void)
{
	static int dmaBuffer = 0;
	
	// Clear interrupt flag
	_DMA3IF = 0;

	// Call use-defined function with true as argument if first buffer, false if second buffer
	DMA_Data[3](DMA_CHANNEL_3, dmaBuffer == 0);
	dmaBuffer ^= 1;
}

/**
	DMA 4 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR  _DMA4Interrupt(void)
{
	static int dmaBuffer = 0;
	
	
	// Clear interrupt flag
	_DMA4IF = 0;

	// Call use-defined function with true as argument if first buffer, false if second buffer
	DMA_Data[4](DMA_CHANNEL_4, dmaBuffer == 0);
	dmaBuffer ^= 1;
}

/**
	DMA 5 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR  _DMA5Interrupt(void)
{
	static int dmaBuffer = 0;
	
	// Clear interrupt flag
	_DMA5IF = 0;

	// Call use-defined function with true as argument if first buffer, false if second buffer
	DMA_Data[5](DMA_CHANNEL_5, dmaBuffer == 0);
	dmaBuffer ^= 1;
}

/**
	DMA 6 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR  _DMA6Interrupt(void)
{
	static int dmaBuffer = 0;
	
	// Clear interrupt flag
	_DMA6IF = 0;

	// Call use-defined function with true as argument if first buffer, false if second buffer
	DMA_Data[6](DMA_CHANNEL_6, dmaBuffer == 0);
	dmaBuffer ^= 1;
}

/**
	DMA 7 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR  _DMA7Interrupt(void)
{
	static int dmaBuffer = 0;
	
	// Clear interrupt flag
	_DMA7IF = 0;

	// Call use-defined function with true as argument if first buffer, false if second buffer
	DMA_Data[7](DMA_CHANNEL_7, dmaBuffer == 0);
	dmaBuffer ^= 1;
}

/*@}*/
