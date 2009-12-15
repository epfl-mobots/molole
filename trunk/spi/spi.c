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

#include "spi.h"
#include "../error/error.h"
#include "../clock/clock.h"
#include "../dma/dma.h"
#include "../gpio/gpio.h"

/*@{*/
/** \file
	Implementation of the SPI interface.
*/

//-----------------------
// Structures definitions
//-----------------------


/** Data for the SPI Interface */
static struct {
	int dma_rx;
	int dma_tx;
	spi_transfert_done callback;
	spi_slave_data_cb slave_callback;
	int priority;
	int data_size;
	gpio ss;
	int waiting;
	int rxtx;
} spi_status[2];



//------------------
// Private functions
//------------------

/** Callback of the spi1 DMA interrupt */
static void spi1_dma_cb(int __attribute__((unused)) channel, bool __attribute__((unused)) first_buffer) {
	gpio_write(spi_status[0].ss, true);
	
	if(spi_status[0].rxtx & 0x1)
		dma_disable_channel(spi_status[0].dma_tx);
	if(spi_status[0].rxtx & 0x2)
		dma_disable_channel(spi_status[0].dma_rx);
	
	if(spi_status[0].callback) 
		spi_status[0].callback(SPI_1);
}

/** Callback of the spi2 DMA interrupt */
static void spi2_dma_cb(int __attribute__((unused)) channel, bool __attribute__((unused)) first_buffer) {
	gpio_write(spi_status[1].ss, true);

	if(spi_status[1].rxtx & 0x1)
		dma_disable_channel(spi_status[1].dma_tx);
	if(spi_status[1].rxtx & 0x2)
		dma_disable_channel(spi_status[1].dma_rx);
	

	if(spi_status[1].callback) 
		spi_status[1].callback(SPI_2);
}

/** Used to implement a dummy semaphore-like mechanism */
static void spi_dummy_wait(int spi) {
	spi_status[spi].waiting = 0;
}

/* Interrupt function (slave mode) */
void _ISR _SPI2Interrupt(void) {
	_SPI2IF = 0;
	
	spi_status[1].slave_callback(SPI_2, SPI2BUF);
	SPI2STATbits.SPIROV = 0; // clear any overflow
}

void _ISR _SPI1Interrupt(void) {
	_SPI1IF = 0;
	
	spi_status[0].slave_callback(SPI_1, SPI1BUF);
	SPI1STATbits.SPIROV = 0; // clear any overflow
}
	


//-------------------
// Exported functions
//-------------------
/**
	Start an SPI transfert
	
	\param	spi_id
			SPI id. One of \ref spi_id.
	\param	tx_buffer
			The tx buffer pointer. Must be in DMA ram. Can be NULL.
	\param	rx_buffer
			The rx buffer pointer. Must be in DMA ram. Must be a valid buffer.
	\param	xch_count
			The number of spi transfert to do. A transfert size is choosed at \ref spi_init_master.
	\param	ss
			The chips select GPIO signal to use.
	\param 	callback
			The callback called when the transfert is done.
	
*/

void spi_start_transfert(int spi_id, void * tx_buffer, void * rx_buffer, unsigned int xch_count, gpio ss, spi_transfert_done callback) {

	int start_tx_dma;
	ERROR_CHECK_RANGE(spi_id, SPI_1, SPI_2, SPI_INVALID_ID);


	
	if(!xch_count || (!tx_buffer && !rx_buffer))  {
		ERROR(SPI_INVALID_TRANSFERT, 0);
	}

	if(spi_id == SPI_1) {
		/* Do a dummy read of the register and clean any overflow */
		(void *) SPI1BUF;
		SPI1STATbits.SPIROV = 0;

		if(!tx_buffer) {
			/* Configure RX with null write */
			dma_init_channel(spi_status[0].dma_rx, DMA_INTERRUPT_SOURCE_SPI_1, 	
					spi_status[0].data_size == SPI_TRSF_BYTE ? DMA_SIZE_BYTE : DMA_SIZE_WORD,
					DMA_DIR_FROM_PERIPHERAL_TO_RAM, DMA_INTERRUPT_AT_FULL, DMA_WRITE_NULL_TO_PERIPHERAL,
					DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT, DMA_OPERATING_ONE_SHOT,
					rx_buffer, 0, (void *) &SPI1BUF, xch_count, spi1_dma_cb);
			spi_status[0].rxtx = 2;
			dma_set_priority(spi_status[0].dma_rx, _SPI1IP);
			dma_enable_channel(spi_status[0].dma_rx);
			start_tx_dma = -1;
			
	
		} else {
			spi_status[0].rxtx = 1;
			/* configure RX DMA channel */
			// We MUST use it as the interrupt handler... otherwise the callback will
			// be called while the transfert is still running and we will deassert CS !
			dma_init_channel(spi_status[0].dma_rx, DMA_INTERRUPT_SOURCE_SPI_1, 	
				spi_status[0].data_size == SPI_TRSF_BYTE ? DMA_SIZE_BYTE : DMA_SIZE_WORD,
				DMA_DIR_FROM_PERIPHERAL_TO_RAM, DMA_INTERRUPT_AT_FULL, DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL,
				DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT, DMA_OPERATING_ONE_SHOT,
				rx_buffer, 0, (void *) &SPI1BUF, xch_count, spi1_dma_cb);
			dma_set_priority(spi_status[0].dma_rx, _SPI1IP);
			dma_enable_channel(spi_status[0].dma_rx);
			spi_status[0].rxtx |= 2;
				
			/* Configure TX DMA channel */
			dma_init_channel(spi_status[0].dma_tx, DMA_INTERRUPT_SOURCE_SPI_1, 	
				spi_status[0].data_size == SPI_TRSF_BYTE ? DMA_SIZE_BYTE : DMA_SIZE_WORD,
				DMA_DIR_FROM_RAM_TO_PERIPHERAL, DMA_INTERRUPT_AT_FULL, DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL,
				DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT, DMA_OPERATING_ONE_SHOT,
				tx_buffer, 0, (void *) &SPI1BUF, xch_count, 0);
			start_tx_dma = spi_status[0].dma_tx;
			dma_set_priority(spi_status[0].dma_tx, _SPI1IP);

			dma_enable_channel(spi_status[0].dma_tx);
		}
		spi_status[0].ss = ss;
		spi_status[0].callback = callback;

		gpio_write(ss, false);
		gpio_set_dir(ss, GPIO_OUTPUT);


		if(start_tx_dma == -1) 
			/** Do a null write on spibuf */
			SPI1BUF = 0;
		else
			dma_start_transfer(start_tx_dma);
			
	} else if (spi_id == SPI_2) {
		/* Do a dummy read of the register and clean any overflow */
		(void *) SPI2BUF;
		SPI2STATbits.SPIROV = 0;

		if(!tx_buffer) {
			/* Configure RX with null write */
			dma_init_channel(spi_status[1].dma_rx, DMA_INTERRUPT_SOURCE_SPI_2, 	
					spi_status[1].data_size == SPI_TRSF_BYTE ? DMA_SIZE_BYTE : DMA_SIZE_WORD,
					DMA_DIR_FROM_PERIPHERAL_TO_RAM, DMA_INTERRUPT_AT_FULL, DMA_WRITE_NULL_TO_PERIPHERAL,
					DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT, DMA_OPERATING_ONE_SHOT,
					rx_buffer, 0, (void *) &SPI2BUF, xch_count, spi2_dma_cb);
			spi_status[1].rxtx = 2;
			dma_set_priority(spi_status[1].dma_rx, _SPI2IP);
			dma_enable_channel(spi_status[1].dma_rx);
			start_tx_dma = -1;
	
		} else {
			spi_status[1].rxtx = 1;
			/* configure RX DMA channel */
			dma_init_channel(spi_status[1].dma_rx, DMA_INTERRUPT_SOURCE_SPI_2, 	
				spi_status[1].data_size == SPI_TRSF_BYTE ? DMA_SIZE_BYTE : DMA_SIZE_WORD,
				DMA_DIR_FROM_PERIPHERAL_TO_RAM, DMA_INTERRUPT_AT_FULL, DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL,
				DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT, DMA_OPERATING_ONE_SHOT,
				rx_buffer, 0, (void *) &SPI2BUF, xch_count, spi2_dma_cb);
				
			dma_set_priority(spi_status[1].dma_rx, _SPI2IP);
			dma_enable_channel(spi_status[1].dma_rx);
			spi_status[1].rxtx |= 2;
			
			/* Configure TX DMA channel */
			dma_init_channel(spi_status[1].dma_tx, DMA_INTERRUPT_SOURCE_SPI_2, 	
				spi_status[1].data_size == SPI_TRSF_BYTE ? DMA_SIZE_BYTE : DMA_SIZE_WORD,
				DMA_DIR_FROM_RAM_TO_PERIPHERAL, DMA_INTERRUPT_AT_FULL, DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL,
				DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT, DMA_OPERATING_ONE_SHOT,
				tx_buffer, 0, (void *) &SPI2BUF, xch_count, 0);
			start_tx_dma = spi_status[1].dma_tx;

			dma_set_priority(spi_status[1].dma_tx, _SPI2IP);
			dma_enable_channel(spi_status[1].dma_tx);
		}
		spi_status[1].ss = ss;
		spi_status[1].callback = callback;

		gpio_write(ss, false);
		gpio_set_dir(ss, GPIO_OUTPUT);
		

		if(start_tx_dma == -1) 
			/** Do a null write on spibuf */
			SPI1BUF = 0;
		else
			dma_start_transfer(start_tx_dma);
			
	} else {
		ERROR(SPI_INVALID_ID, &spi_id);
	}
}

/**
	Initialise an SPI device
	
	\param	spi_id
			SPI id. One of \ref spi_id.
	\param	speed_khz
			The speed of the SPI clock. The effective speed will not be higher than the specified speed, but can be slower.
	\param	dma_rx
			The DMA channel used for RX
	\param	dma_tx
			The DMA channel used for TX
	\param	transfert_mode
			The mode used for transfert. Must be one of \ref spi_tranfert_size.
	\param 	polarity
			The polarity used by the clock. Must be one of \ref spi_clock_polarity.
	\param 	data_out_mode
			Used to specify when the data out must happend on the clock transition. Must be one of \ref spi_data_out_mode.
	\param 	sample_phase
			Used to specify when the data must be sampled for RX. Must be one of \ref spi_sample_phase.
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 6 (highest normal priority)
*/
void spi_init_master(int spi_id, unsigned int speed_khz, int dma_rx, int dma_tx, int transfert_mode, int polarity, int data_out_mode, int sample_phase, int priority) {
	unsigned int ratio;
	unsigned long fcy;
	int i,j;

	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	ERROR_CHECK_RANGE(transfert_mode, SPI_TRSF_BYTE, SPI_TRSF_WORD, SPI_INVALID_TRANFERT_MODE);
	ERROR_CHECK_RANGE(polarity, SPI_CLOCK_IDLE_LOW, SPI_CLOCK_ACTIVE_LOW, SPI_INVALID_POLARITY);
	ERROR_CHECK_RANGE(data_out_mode, SPI_DATA_OUT_CLK_IDLE_TO_ACTIVE, SPI_DATA_OUT_CLK_ACTIVE_TO_IDLE, SPI_INVALID_DATA_OUT_MODE);
	ERROR_CHECK_RANGE(sample_phase, SPI_SAMPLE_PHASE_MIDDLE, SPI_SAMPLE_PHASE_END, SPI_INVALID_SAMPLE_PHASE);

	
	
	if(spi_id == SPI_1) {
		SPI1STAT = 0;
		SPI1CON1bits.DISSCK = 0;			/* Enable SCK */
		SPI1CON1bits.DISSDO = 0;			/* Enable SDO */
		SPI1CON1bits.MODE16 = transfert_mode;
		SPI1CON1bits.SMP = sample_phase;
		SPI1CON1bits.CKE = data_out_mode;
		SPI1CON1bits.SSEN = 0;				/* I'm not a slave */
		SPI1CON1bits.CKP = polarity;
		SPI1CON1bits.MSTEN = 1;				/* I'm a master ! */
		SPI1CON2 = 0x0;						/* Framing support completly disabled */
		_SPI1IP = priority;					/* Used to set the DMA priority */
		
		
		/* Get the "Optimal" speed: NOT > as asked speed*/
		if(speed_khz > 15000 || speed_khz == 0) {
			ERROR(SPI_INVALID_SPEED, &speed_khz);
		}
		
		fcy = clock_get_cycle_frequency();
		ratio = fcy / (((unsigned long)speed_khz) * 1000);
		

		if(ratio > 512) {
			ERROR(SPI_INVALID_SPEED, &speed_khz);
		}

		for(i = 1, j = 3; i <= 64; i <<= 2, j--) 
			if(ratio / i < 8) 
				break;

		SPI1CON1bits.PPRE = j;

		/* +1 is here to make sure we don't round at a higher frequency */
		ratio = ratio / i + 1; 

		for(i = 1, j = 7; i <= 8; i++, j--) 
			if(ratio <= i) 
				break;
		
		if(i == 9) 
			j = 0;

		SPI1CON1bits.SPRE = j;

		spi_status[0].dma_rx = dma_rx;
		spi_status[0].dma_tx = dma_tx;
		spi_status[0].priority = priority;
		spi_status[0].data_size = transfert_mode;
		SPI1STATbits.SPIEN = 1;				/* Enable SPI module */

	} else if (spi_id == SPI_2) { 
		SPI2STAT = 0;
		SPI2CON1bits.DISSCK = 0;			/* Enable SCK */
		SPI2CON1bits.DISSDO = 0;			/* Enable SDO */
		SPI2CON1bits.MODE16 = transfert_mode;
		SPI2CON1bits.SMP = sample_phase;
		SPI2CON1bits.CKE = data_out_mode;
		SPI2CON1bits.SSEN = 0;				/* I'm not a slave */
		SPI2CON1bits.CKP = polarity;
		SPI2CON1bits.MSTEN = 1;				/* I'm a master ! */
		SPI2CON2 = 0x0;						/* Framing support completly disabled */
		_SPI2IP = priority;

		/* Get the "Optimal" speed: NOT > as asked speed*/
		if(speed_khz > 15000 || speed_khz == 0) {
			ERROR(SPI_INVALID_SPEED, &speed_khz);
		}
		
		fcy = clock_get_cycle_frequency();
		ratio = (fcy)/ (((unsigned long)speed_khz) * 1000);
		
		if(ratio > 512) {
			ERROR(SPI_INVALID_SPEED, &speed_khz);
		}

		for(i = 1, j = 3; i <= 64; i <<= 2, j--) 
			if(ratio / i < 8) 
				break;

		SPI2CON1bits.PPRE = j;

		/* +1 is here to make sure we don't round at a higher frequency */
		ratio = ratio / i + 1; 

		for(i = 1, j = 7; i <= 8; i++, j--) 
			if(ratio <= i) 
				break;

		if(i == 9) 
			j = 0;
		SPI2CON1bits.SPRE = j;


		spi_status[1].dma_rx = dma_rx;
		spi_status[1].dma_tx = dma_tx;
		spi_status[1].priority = priority;
		spi_status[1].data_size = transfert_mode;
		SPI2STATbits.SPIEN = 1;				/* Enable SPI module  */

	} else {
		ERROR(SPI_INVALID_ID, &spi_id);
	}

}

/**
	Perform a busy-waiting SPI transfert
	
	\param	spi_id
			SPI id. One of \ref spi_id.
	\param	tx_buffer
			The tx buffer pointer. Must be in DMA ram. Can be NULL.
	\param	rx_buffer
			The rx buffer pointer. Must be in DMA ram. Can be NULL.
	\param	xch_count
			The number of spi transfert to do. A transfert size is choosed at \ref spi_init_master.
	\param	ss
			The chips select GPIO signal to use.
	
*/
void spi_transfert_sync(int spi_id, void * tx_buffer, void * rx_buffer, unsigned int xch_count, gpio ss) {
	ERROR_CHECK_RANGE(spi_id, SPI_1, SPI_2, SPI_INVALID_ID);
	
	spi_status[spi_id].waiting = 1;
	spi_start_transfert(spi_id, tx_buffer, rx_buffer, xch_count, ss, spi_dummy_wait);
	while(spi_status[spi_id].waiting)
		barrier(); // Cannot use Idle, since there is a race between the check on waiting and the pwrsv instruction
}
	
void spi_init_slave(int spi_id, int transfert_mode, int polarity, int data_out_mode, spi_slave_data_cb data_cb, int priority) {
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	ERROR_CHECK_RANGE(transfert_mode, SPI_TRSF_BYTE, SPI_TRSF_WORD, SPI_INVALID_TRANFERT_MODE);
	ERROR_CHECK_RANGE(polarity, SPI_CLOCK_IDLE_LOW, SPI_CLOCK_ACTIVE_LOW, SPI_INVALID_POLARITY);
	ERROR_CHECK_RANGE(data_out_mode, SPI_DATA_OUT_CLK_IDLE_TO_ACTIVE, SPI_DATA_OUT_CLK_ACTIVE_TO_IDLE, SPI_INVALID_DATA_OUT_MODE);

	
	if(spi_id == SPI_1) {
		SPI1STAT = 0;
		SPI1CON1bits.DISSCK = 0;			/* Enable SCK */
		SPI1CON1bits.DISSDO = 0;			/* Enable SDO */
		SPI1CON1bits.SMP = 0;
		SPI1CON1bits.MODE16 = transfert_mode;
		SPI1CON1bits.CKE = data_out_mode;
		SPI1CON1bits.SSEN = 0;				/* Errata 8, Slave Select pin is not working */
		SPI1CON1bits.CKP = polarity;
		SPI1CON1bits.MSTEN = 0;				/* I'm a slave ! */
		SPI1CON2 = 0x0;						/* Framing support completly disabled */
		_SPI1IP = priority;
		
		SPI1STATbits.SPIEN = 1;				/* Enable module */
	
		_SPI1IF = 0;
		_SPI1IE = 1;
		
		spi_status[0].slave_callback = data_cb;
		
	} else if(spi_id == SPI_2) {
		SPI2STAT = 0;
		SPI2CON1bits.DISSCK = 0;			/* Enable SCK */
		SPI2CON1bits.DISSDO = 0;			/* Enable SDO */
		SPI2CON1bits.SMP = 0;
		SPI2CON1bits.MODE16 = transfert_mode;
		SPI2CON1bits.CKE = data_out_mode;
		SPI2CON1bits.SSEN = 0;				/* Errata 8, Slave Select pin is not working */
		SPI2CON1bits.CKP = polarity;
		SPI2CON1bits.MSTEN = 0;				/* I'm a slave ! */
		SPI2CON2 = 0x0;						/* Framing support completly disabled */
		_SPI2IP = priority;
		
		SPI2STATbits.SPIEN = 1;				/* Enable module */
	
		_SPI2IF = 0;
		_SPI2IE = 1;
		
		spi_status[1].slave_callback = data_cb;
	
	} else {
		ERROR(SPI_INVALID_ID, &spi_id);
	}
	
	
}
void spi_slave_write(int spi_id, unsigned int data) {
	if(spi_id == SPI_1) 
		SPI1BUF = data;
	else if(spi_id == SPI_2)
		SPI2BUF = data;
	else {
		ERROR(SPI_INVALID_ID, &spi_id);
	}
}
	

/*@}*/

