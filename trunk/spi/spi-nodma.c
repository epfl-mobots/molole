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

#include "spi-nodma.h"
#include "../error/error.h"
#include "../clock/clock.h"
#include "../gpio/gpio.h"

/*@{*/
/** \file
	Implementation of the SPI_NODMA interface.
*/

//-----------------------
// Structures definitions
//-----------------------


/** Data for the SPI Interface */
static struct {
	int data_size;
	gpio ss;
} spi_status[2];


//-------------------
// Exported functions
//-------------------
/**
	Initialise an SPI device
	
	\param	spi_id
			SPI id. One of \ref spi_id.
	\param	speed_khz
			The speed of the SPI clock. The effective speed will not be higher than the specified speed, but can be slower.
	\param	transfert_mode
			The mode used for transfert. Must be one of \ref spi_tranfert_size.
	\param 	polarity
			The polarity used by the clock. Must be one of \ref spi_clock_polarity.
	\param 	data_out_mode
			Used to specify when the data out must happend on the clock transition. Must be one of \ref spi_data_out_mode.
	\param 	sample_phase
			Used to specify when the data must be sampled for RX. Must be one of \ref spi_sample_phase.
*/
void spi_nodma_init_master(int spi_id, unsigned int speed_khz, int transfert_mode, int polarity, int data_out_mode, int sample_phase) {
	unsigned int ratio;
	unsigned long fcy;
	int i,j;

	ERROR_CHECK_RANGE(transfert_mode, SPI_NODMA_TRSF_BYTE, SPI_NODMA_TRSF_WORD, SPI_NODMA_INVALID_TRANFERT_MODE);
	ERROR_CHECK_RANGE(polarity, SPI_NODMA_CLOCK_IDLE_LOW, SPI_NODMA_CLOCK_ACTIVE_LOW, SPI_NODMA_INVALID_POLARITY);
	ERROR_CHECK_RANGE(data_out_mode, SPI_NODMA_DATA_OUT_CLK_IDLE_TO_ACTIVE, SPI_NODMA_DATA_OUT_CLK_ACTIVE_TO_IDLE, SPI_NODMA_INVALID_DATA_OUT_MODE);
	ERROR_CHECK_RANGE(sample_phase, SPI_NODMA_SAMPLE_PHASE_MIDDLE, SPI_NODMA_SAMPLE_PHASE_END, SPI_NODMA_INVALID_SAMPLE_PHASE);

	
	
	if(spi_id == SPI_NODMA_1) {
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
		
		/* Get the "Optimal" speed: NOT > as asked speed*/
		if(speed_khz > 15000 || speed_khz == 0) {
			ERROR(SPI_NODMA_INVALID_SPEED, &speed_khz);
		}
		
		fcy = clock_get_cycle_frequency();
		ratio = fcy / (((unsigned long)speed_khz) * 1000);
		

		if(ratio > 512) {
			ERROR(SPI_NODMA_INVALID_SPEED, &speed_khz);
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
		spi_status[0].data_size = transfert_mode;

		SPI1STATbits.SPIEN = 1;				/* Enable SPI module */

	} else if (spi_id == SPI_NODMA_2) { 
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

		/* Get the "Optimal" speed: NOT > as asked speed*/
		if(speed_khz > 15000 || speed_khz == 0) {
			ERROR(SPI_NODMA_INVALID_SPEED, &speed_khz);
		}
		
		fcy = clock_get_cycle_frequency();
		ratio = (fcy)/ (((unsigned long)speed_khz) * 1000);
		
		if(ratio > 512) {
			ERROR(SPI_NODMA_INVALID_SPEED, &speed_khz);
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


		spi_status[1].data_size = transfert_mode;
		SPI2STATbits.SPIEN = 1;				/* Enable SPI module  */

	} else {
		ERROR(SPI_NODMA_INVALID_ID, &spi_id);
	}

}

/**
	Perform a busy-waiting SPI transfert
	
	\param	spi_id
			SPI id. One of \ref spi_id.
	\param	tx_buffer
			The tx buffer pointer. Can be NULL
	\param	rx_buffer
			The rx buffer pointer. Can be NULL.
	\param	xch_count
			The number of spi transfert to do. A transfert size is choosed at \ref spi_init_master.
	\param	ss
			The chips select GPIO signal to use.
	
*/
void spi_nodma_transfert_sync(int spi_id, void * tx_buffer, void * rx_buffer, unsigned int xch_count, gpio ss) {
	unsigned int temp;
	unsigned char * txc = tx_buffer;
	unsigned char * rxc = rx_buffer;
	unsigned int * txi = tx_buffer;
	unsigned int * rxi = rx_buffer;

	ERROR_CHECK_RANGE(spi_id, SPI_NODMA_1, SPI_NODMA_2, SPI_NODMA_INVALID_ID);

	if(spi_status[spi_id].data_size && (((unsigned int) tx_buffer) & 0x1)) {
		ERROR(SPI_NODMA_NONALIGNED_BUFFER, tx_buffer);
	}
	if(spi_status[spi_id].data_size && (((unsigned int) rx_buffer) & 0x1)) {
		ERROR(SPI_NODMA_NONALIGNED_BUFFER, rx_buffer);
	}

	gpio_write(ss, false);
	gpio_set_dir(ss, GPIO_OUTPUT);

	if(spi_id == SPI_NODMA_1) {
		// Dummy read, empty the rx buffer
		temp = SPI1BUF;
		while(xch_count--) {
			if(tx_buffer) {
				if(spi_status[spi_id].data_size)
					SPI1BUF = *txi++;
				else
					SPI1BUF = *txc++;
			} else {
				SPI1BUF = 0;
			}
			while(!SPI1STATbits.SPIRBF) barrier();
			if(rx_buffer) {
				if(spi_status[spi_id].data_size)
					*rxi++ = SPI1BUF;
				else
					*rxc++ = SPI1BUF;
			} else
				temp = SPI1BUF;
		}

	} else {
		temp = SPI2BUF;
		while(xch_count--) {
			if(tx_buffer) {
				if(spi_status[spi_id].data_size)
					SPI2BUF = *txi++;
				else
					SPI2BUF = *txc++;
			} else {
				SPI2BUF = 0;
			}
			while(!SPI2STATbits.SPIRBF) barrier();
			if(rx_buffer) {
				if(spi_status[spi_id].data_size)
					*rxi++ = SPI2BUF;
				else
					*rxc++ = SPI2BUF;
			} else
				temp = SPI2BUF;
		}
	}
	gpio_write(ss, true);
}
/*@}*/

