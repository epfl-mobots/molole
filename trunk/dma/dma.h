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

#ifndef _MOLOLE_DMA_H
#define _MOLOLE_DMA_H

#include "../types/types.h"

/** \addtogroup dma */
/*@{*/

/** \file
	\brief A wrapper around dsPIC33 DMA.
*/

// Defines

/** Errors DMA can throw */
enum dma_errors
{
	DMA_ERROR_BASE = 0x0500,
	DMA_ERROR_INVALID_CHANNEL,				/**< The specified DMA channel is not one of dma_channels_identifiers. */
	DMA_ERROR_INVALID_REQUEST_SOURCE,		/**< The specified request source is not one of dma_requests_sources. */
	DMA_ERROR_INVALID_DATA_SIZE,			/**< The specified data size is not one of dma_data_sizes. */
	DMA_ERROR_INVALID_TRANSFER_DIRECTION,	/**< The specified transfer direction is not one of dma_transfer_direction. */
	DMA_ERROR_INVALID_INTERRUPT_POSITION,	/**< The specified data size is not one of dma_interrupt_position. */
	DMA_ERROR_INVALID_WRITE_NULL_MODE,		/**< The specified null data write mode is not one of dma_null_data_peripheral_write_mode_select. */
	DMA_ERROR_INVALID_ADDRESSING_MODE,		/**< The specified addressing mode is not one of dma_addressing_mode. */
	DMA_ERROR_INVALID_OPERATING_MODE,		/**< The specified operating mode is not one of dma_operating_mode. */
	DMA_ERROR_INVALID_ADDRESS				/**< The specified address is not a DMA address. Declare your dma storage space with __attribute__((space(dma))) */
};
	

/** Identifiers of available DMA channels. */
enum dma_channels_identifiers
{
	DMA_CHANNEL_0 = 0,						/**< DMA channel 0 */
	DMA_CHANNEL_1,							/**< DMA channel 1 */
	DMA_CHANNEL_2,							/**< DMA channel 2 */
	DMA_CHANNEL_3,							/**< DMA channel 3 */
	DMA_CHANNEL_4,							/**< DMA channel 4 */
	DMA_CHANNEL_5,							/**< DMA channel 5 */
	DMA_CHANNEL_6,							/**< DMA channel 6 */
	DMA_CHANNEL_7,							/**< DMA channel 7 */
};

/** Sources of requests that can initiate DMA. */
enum dma_requests_sources
{
	DMA_INTERRUPT_SOURCE_INT_0 = 0x0,		/**< External Interrupt 0 */
	DMA_INTERRUPT_SOURCE_IC_1 = 0x1,		/**< Input Capture 1 */
	DMA_INTERRUPT_SOURCE_OC_1 = 0x2,		/**< Output Compare 1 */
	DMA_INTERRUPT_SOURCE_IC_2 = 0x5,		/**< Input Capture 2 */
	DMA_INTERRUPT_SOURCE_OC_2 = 0x6,		/**< Output Compare 2 */
	DMA_INTERRUPT_SOURCE_TIMER_2 = 0x7,		/**< Timer 2 */
	DMA_INTERRUPT_SOURCE_TIMER_3 = 0x8,		/**< Timer 3 */
	DMA_INTERRUPT_SOURCE_SPI_1 = 0xa,		/**< SPI 1 Transfer Done */
	DMA_INTERRUPT_SOURCE_UART_1_RX = 0xb,	/**< UART 1 Receiver */
	DMA_INTERRUPT_SOURCE_UART_1_TX = 0xc,	/**< UART 1 Transmitter */
	DMA_INTERRUPT_SOURCE_ADC_1 = 0xd,		/**< ADC 1 Convert Done */
	DMA_INTERRUPT_SOURCE_ADC_2 = 0x15,		/**< ADC 2 Convert Done */
	DMA_INTERRUPT_SOURCE_UART_2_RX = 0x1e,	/**< UART 2 Receiver */
	DMA_INTERRUPT_SOURCE_UART_2_TX = 0x1f,	/**< UART 2 Transmitter */
	DMA_INTERRUPT_SOURCE_SPI_2 = 0x21,		/**< SPI 2 Transfer Done */
	DMA_INTERRUPT_SOURCE_ECAN_1_RX = 0x22,	/**< ECAN 1 RX Data Ready */
	DMA_INTERRUPT_SOURCE_ECAN_2_RX = 0x37,	/**< ECAN 2 RX Data Ready */
	DMA_INTERRUPT_SOURCE_DCI = 0x3c,		/**< CODEC Transfer Done */
	DMA_INTERRUPT_SOURCE_ECAN_1_TX = 0x46,	/**< ECAN 1 TX Data Request */
	DMA_INTERRUPT_SOURCE_ECAN_2_TX = 0x47,	/**< ECAN 2 TX Data Request */
};

/** size of data to transfer */
enum dma_data_sizes
{
	DMA_SIZE_WORD = 0,						/**< DMA transfers 2 bytes at a time */
	DMA_SIZE_BYTE = 1,						/**< DMA transfers 1 byte at a time*/
};

/** direction of transfer */
enum dma_transfer_direction
{
	DMA_DIR_FROM_PERIPHERAL_TO_RAM = 0,		/**< Read from Peripheral address, write to DPSRAM address */
	DMA_DIR_FROM_RAM_TO_PERIPHERAL = 1,		/**< Read from DPSRAM address, write to peripheral address */
};

/** should DMA interrupt happens at half of transfer or when it is completed? */
enum dma_interrupt_position
{
	DMA_INTERRUPT_AT_FULL = 0,				/**< Initiate interrupt when a whole buffer has been filled*/
	DMA_INTERRUPT_AT_HALF = 1				/**< Initiate interrupt when half of a buffer has been filled */
};

/** should DMA write null to peripheral when writing doto to DPSRAM? */
enum dma_null_data_peripheral_write_mode_select
{
	DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL = 0,	/**< Normal operation */
	DMA_WRITE_NULL_TO_PERIPHERAL = 1		/**< Null data write to peripheral in addition to DPSRAM write (transfer must be DMA_DIR_FROM_PERIPHERAL_TO_RAM) */
};

/** DMA Channel Addressing Mode  */
enum dma_addressing_mode
{
	DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT = 0,	/**< Register Indirect with Post-Increment mode */
	DMA_ADDRESSING_REGISTER = 1,							/**< Register Indirect without Post-Increment mode */
	DMA_ADDRESSING_PERIPHERAL_INDIRECT = 2					/**< Peripheral Indirect Addressing mode */
};

/** DMA Channel Operating Mode */
enum dma_operating_mode
{
	DMA_OPERATING_CONTINUOUS = 0,							/**< Continuous, Ping-Pong modes disabled */
	DMA_OPERATING_ONE_SHOT = 1,								/**< One-Shot, Ping-Pong modes disabled */
	DMA_OPERATING_CONTINUOUS_PING_PONG = 2,					/**< Continuous, Ping-Pong modes enabled */
	DMA_OPERATING_ONE_SHOT_PING_PONG = 3,					/**< One-Shot, Ping-Pong modes enabled (one block transfer from/to each DMA RAM buffer) */
};

/** DMA callback when a buffer is half or fully filled (depends on dma_interrupt_position) */
typedef void(*dma_callback)(int channel, bool first_buffer);


// Functions, doc in the .c

void dma_init_channel(int channel, int request_source, int data_size, int transfer_dir, int interrupt_pos, int null_write, int addressing_mode, int operating_mode, void * a, void * b, void* peripheral_address, unsigned transfer_count, dma_callback callback);

void dma_enable_channel(int channel);

void dma_disable_channel(int channel);

void dma_start_transfer(int channel);

/*@}*/

#endif
