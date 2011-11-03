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

#ifndef _MOLOLE_SPI_H
#define _MOLOLE_SPI_H


#include "../types/types.h"
#include "../gpio/gpio.h"

//--------------------
// Usage documentation
//--------------------

/** \addtogroup spi */
/*@{*/

/** \file
	Wrapper around SPI interface
*/

/** Error spi can throw */
enum spi_errors
{
	SPI_ERROR_BASE = 0x0C00,
	SPI_INVALID_ID,				/**< The specified ID is invalid */
	SPI_INVALID_TRANFERT_MODE,	/**< The specified transfert mode is invalid */
	SPI_INVALID_POLARITY,		/**< The specified polarity is invalid */
	SPI_INVALID_SPEED,			/**< The specified speed is invalid */
	SPI_INVALID_DATA_OUT_MODE,	/**< The specified data out mode is invalide */
	SPI_INVALID_SAMPLE_PHASE,	/**< The specified sample phase mode is invalid */
	SPI_INVALID_TRANSFERT,		/**< An invalide transfert has been requested */
};


enum spi_id
{
	/*! */ SPI_1 = 0,
	/*! */ SPI_2,
};

enum spi_tranfert_size
{
	/*! */ SPI_TRSF_BYTE = 0,
	/*! */ SPI_TRSF_WORD,
};

enum spi_clock_polarity
{
	/*! */ SPI_CLOCK_IDLE_HIGH = 1,
	/*! */ SPI_CLOCK_ACTIVE_LOW = 1,
	/*! */ SPI_CLOCK_IDLE_LOW = 0,
	/*! */ SPI_CLOCK_ACTIVE_HIGH = 0,
};

enum spi_data_out_mode
{
	/*! */ SPI_DATA_OUT_CLK_IDLE_TO_ACTIVE = 0,
	/*! */ SPI_DATA_OUT_CLK_ACTIVE_TO_IDLE,
};

enum spi_sample_phase
{
	/*! */ SPI_SAMPLE_PHASE_MIDDLE = 0,
	/*! */ SPI_SAMPLE_PHASE_END,
};
typedef void (*spi_transfert_done)(int spi_id);

void spi_init_master(int spi_id, unsigned int speed_khz, int dma_rx, int dma_tx, int transfert_mode, int polarity, int data_out_mode, int sample_phase, int priority);

void spi_start_transfert(int spi_id, void * tx_buffer, void * rx_buffer, unsigned int xch_count, gpio ss, spi_transfert_done callback);

void spi_transfert_sync(int spi_id, void * tx_buffer, void * rx_buffer, unsigned int xch_count, gpio ss);

	
typedef void (*spi_slave_data_cb)(int spi_id, unsigned int data);	
	
void spi_init_slave(int spi_id, int transfert_mode, int polarity, int data_out_mode, spi_slave_data_cb data_cb, int priority);	

void spi_slave_write(int spi_id, unsigned int data);

	
/*@}*/

#endif
