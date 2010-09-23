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

#include <p33fxxxx.h>

#include "can.h"
#include "../error/error.h"
#include "../dma/dma.h"
#include "../clock/clock.h"

/*! CAN config mode magic number */
#define CAN_CONFIG_MODE 0x4
/*! CAN disabled mode magic number */
#define CAN_DISABLE_MODE 0x1
/*! CAN normal (running) mode magic number */
#define CAN_NORMAL_MODE 0x0
/*! CAN loopback mode magic number
 \warning Buggy, see errata
 */
#define CAN_LOOPBACK_MODE 0x2


/*! CAN frame structure as seen by the can module
 */
struct s_can_buf
{
	unsigned int sid  __attribute__((packed));
	unsigned int eid  __attribute__((packed));
	unsigned int dlc  __attribute__((packed));
	unsigned int data[4]  __attribute__((packed));
	unsigned int stat  __attribute__((packed));
};

#define CANDRV_NUMBUF 32
static struct s_can_buf can_buf[CANDRV_NUMBUF] __attribute__((space(dma),aligned(CANDRV_NUMBUF * sizeof(struct s_can_buf))));

static can_frame_received_callback rx_cb;
static can_frame_sent_callback tx_done_cb;

struct speed_s
{
	unsigned int kbps;
	unsigned int CxCFG1;
	unsigned int CxCFG2;
};

//! Table to lookup CAN timing constants for 40 MHz operation, NULL terminated
const static struct speed_s __attribute__((space(auto_psv))) speed_tab_40[] = {
	{ .kbps = 1000,
// 10 Tq. 1 prop, 4 phase 1, 4 phase 2, 4 synchro
	  .CxCFG1 = 0x00C1, //0x0,
	  .CxCFG2 = 0x0398, //0x07FA,
	},
	{ .kbps = 500,
	  .CxCFG1 = 0x1,
	  .CxCFG2 = 0x07F4,
	},
	{ .kbps = 250,
	  .CxCFG1 = 0x3,
	  .CxCFG2 = 0x07FA,
	}, 
	{ .kbps = 100,
	  .CxCFG1 = 0x9,
	  .CxCFG2 = 0x07FA,
	},
	{ .kbps = 0,
	}
};

//! Table to lookup CAN timing constants for 30 MHz operation, NULL terminated
const static struct __attribute__((space(auto_psv))) speed_s speed_tab_30[] = {
	{ .kbps = 1000,
// 15 Tq, 1 prop, 7 phase 1, 6 phase 2, 4 synchro
	  .CxCFG1 = 0x00C0, // 0x0,
	  .CxCFG2 = 0x05B0, //0x04F8,
	},
	{ .kbps = 500,
	  .CxCFG1 = 0x1,
	  .CxCFG2 = 0x04F8,
	},
	{ .kbps = 250,
	  .CxCFG1 = 0x2,
	  .CxCFG2 = 0x07FA,
	}, 
	{ .kbps = 100,
	  .CxCFG1 = 0x9,
	  .CxCFG2 = 0x04F8,
	},
	{ .kbps = 0,
	}
};


/*! Reference count for the C1CTRL1bits.WIN bits */
static int confw_count;

/*! Increment the refeience count and set the CAN C1CTRL1bits.WIN bit
 * \sa confw_count can_release_confw
 */
static void can_grab_confw(void)
{
	confw_count++;
	C1CTRL1bits.WIN = 1;
}

/*! Decrement the reference count and clear the CAN C1CTRL1bits.WIN bit if 0
 * \sa confw_count can_grab_confw
 */
static void can_release_confw(void)
{
	if(!--confw_count)
		C1CTRL1bits.WIN = 0;
}

/*! Ask the can module to change runlevel
 * \param level Asked runlevel
 * \sa CAN_CONFIG_MODE CAN_DISABLE_MODE CAN_NORMAL_MODE CAN_LOOPBACK_MODE
 */
static void can_ask_runlevel(int level)
{
	if(C1CTRL1bits.OPMODE == level)
		return;
	C1CTRL1bits.REQOP=level;
	while(C1CTRL1bits.OPMODE != level);
}

/*! Initialize can's filters */
static void setup_can_filters(void)
{

	/* Setup every filter to act on the FIFO */
	can_grab_confw();
	C1BUFPNT1 = 0xFFFF;
	C1BUFPNT2 = 0xFFFF;
	C1BUFPNT3 = 0xFFFF;
	C1BUFPNT4 = 0xFFFF;

	/* Enable first filter ( so we have at least one enabled )*/
	C1FEN1 = 1;

	/* Setup every filter to 0, EID disabled ( but set to 0 ) */
	C1RXF0SID = 0;
	C1RXF1SID = 0;
	C1RXF2SID = 0;
	C1RXF3SID = 0;
	C1RXF4SID = 0;
	C1RXF5SID = 0;
	C1RXF6SID = 0;
	C1RXF7SID = 0;
	C1RXF8SID = 0;
	C1RXF9SID = 0;
	C1RXF10SID = 0;
	C1RXF11SID = 0;
	C1RXF12SID = 0;
	C1RXF13SID = 0;
	C1RXF14SID = 0;
	C1RXF15SID = 0;
	C1RXF0EID = 0;
	C1RXF1EID = 0;
	C1RXF2EID = 0;
	C1RXF3EID = 0;
	C1RXF4EID = 0;
	C1RXF5EID = 0;
	C1RXF6EID = 0;
	C1RXF7EID = 0;
	C1RXF8EID = 0;
	C1RXF9EID = 0;
	C1RXF10EID = 0;
	C1RXF11EID = 0;
	C1RXF12EID = 0;
	C1RXF13EID = 0;
	C1RXF14EID = 0;
	C1RXF15EID = 0;

	/* Every filter has mask source 0 */
	C1FMSKSEL1 = 0;
	C1FMSKSEL2 = 0;
	/* Setup every mask to 0x0, so we accept everything
	* match EID/SID, EID cleared */
	C1RXM0SID = 0;
	C1RXM1SID = 0;
	C1RXM2SID = 0;

	C1RXM0EID = 0;
	C1RXM1EID = 0;
	C1RXM2EID = 0;
	can_release_confw();
}

//! Setup the CAN buffers for our specific configuration: 1 as sending (because of CPU bug) and the rest as reception
static void setup_can_buffers(void)
{
	/* 32 buffers in DMA ram */
	C1FCTRLbits.DMABS = 0x6;

	/* Configure 31 buffer as FIFO for incoming packet
	* and 1 other as transmit buffers */

	C1FCTRLbits.FSA = 1; /* FIFO start at buffer 1 */

	/* Buffer 0, Transmit. */
	/* 0x80 == Transmit buffer, clear all error
	* no auto remote-transmit, low priority */
	C1TR01CON = 0x0080;
	C1TR23CON = 0x0000;
	C1TR45CON = 0x0000;
	C1TR67CON = 0x0000;

	/* Empty every buffers */
	C1RXFUL1 = 0;
	C1RXFUL2 = 0;
	C1RXOVF1 = 0;
	C1RXOVF2 = 0;
}

//! Configure CAN timing registers from current processor speed and requested CAN baud rate
static void can_set_speed(unsigned int speed)
{
	int i;
	unsigned int cpuclk = clock_get_target_bogomips();
	switch(cpuclk)
	{
		case 40:
			for(i = 0; speed_tab_40[i].kbps; i++)
			{
				if(speed_tab_40[i].kbps == speed)
				{
					C1CFG1 = speed_tab_40[i].CxCFG1;
					C1CFG2 = speed_tab_40[i].CxCFG2;
					return;
				}
			}
			ERROR(CAN_UNKNOWN_SPEED, &speed);
			break;
		case 30:
			for(i = 0; speed_tab_30[i].kbps; i++)
			{
				if(speed_tab_30[i].kbps == speed)
				{
					C1CFG1 = speed_tab_30[i].CxCFG1;
					C1CFG2 = speed_tab_30[i].CxCFG2;
					return;
				}
			}
			ERROR(CAN_UNKNOWN_SPEED, &speed);
			break;
		default:
			ERROR(CAN_UNKNOWN_CPU_CLOCK, &cpuclk);
	}
}


/**
	Init the CAN 1 subsystem.
	
	\param	frame_received_callback
			function to call when a new frame is received
	\param	frame_sent_callback
			function to call when a frame has been transmitted
	\param	dma_rx_channel
			DMA channel to use for reception, one of \ref dma_channels_identifiers
	\param	dma_tx_channel
			DMA channel to use for emission, one of \ref dma_channels_identifiers
	\param	trans_pin
			GPIO to control the transciever speed (slow/fast), can be \ref GPIO_MAKE_ID(GPIO_NONE,0) if this feature is not used.
	\param	kbaud_rate
			baud rate in kbps
	\param	priority
			Interrupt priority, from 1 (lowest priority) to 6 (highest normal priority)
*/
void can_init(can_frame_received_callback frame_received_callback, can_frame_sent_callback frame_sent_callback, int dma_rx_channel, int dma_tx_channel, gpio trans_pin, unsigned int kbaud_rate, int priority)
{
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	
	C1CTRL1bits.WIN = 0;
	can_ask_runlevel(CAN_CONFIG_MODE);
	setup_can_buffers();
	setup_can_filters();
	/* disable devicenet */
    C1CTRL2bits.DNCNT = 0;

	/* Enable interrupt */
    C1INTEbits.RBIE=1;
    C1INTEbits.TBIE=1;
    C1INTEbits.ERRIE=1;
    _C1IP = priority;
    _C1IF = 0;
    _C1IE = 1;
    

	/* Enable DMA */
	dma_init_channel(dma_rx_channel, DMA_INTERRUPT_SOURCE_ECAN_1_RX, DMA_SIZE_WORD,
					DMA_DIR_FROM_PERIPHERAL_TO_RAM, DMA_INTERRUPT_AT_FULL, 
					DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL, DMA_ADDRESSING_PERIPHERAL_INDIRECT,
					DMA_OPERATING_CONTINUOUS, can_buf, 0, (void *) &C1RXD, 
					sizeof(struct s_can_buf) / sizeof(unsigned int) - 1, 0);
	dma_enable_channel(dma_rx_channel);

	dma_init_channel(dma_tx_channel, DMA_INTERRUPT_SOURCE_ECAN_1_TX, DMA_SIZE_WORD,
					DMA_DIR_FROM_RAM_TO_PERIPHERAL, DMA_INTERRUPT_AT_FULL, 
					DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL, DMA_ADDRESSING_PERIPHERAL_INDIRECT,
					DMA_OPERATING_CONTINUOUS, can_buf, 0, (void *) &C1TXD, 
					sizeof(struct s_can_buf) / sizeof(unsigned int) - 1, 0);
	dma_enable_channel(dma_tx_channel);		

	/* Enable the trans. at high speed */
	gpio_write(trans_pin, 0);
	gpio_set_dir(trans_pin,GPIO_OUTPUT);
	
	rx_cb = frame_received_callback;
	tx_done_cb = frame_sent_callback;

	can_set_speed(kbaud_rate);

	can_ask_runlevel(CAN_NORMAL_MODE);
}

/**
	Send a frame on CAN 1.
	
	\param	frame
			frame to send
	
	\return	return true if there was enough space to send the frame, false if the send buffer is full.
*/
bool can_send_frame(const can_frame *frame)
{
	if(!C1TR01CONbits.TXREQ0)
	{
		/* Copy the frame in buffer 0 */
		can_buf[0].sid = ((int) frame->id) << 2;
		can_buf[0].eid = 0;
		can_buf[0].dlc = (int) frame->len;
		can_buf[0].data[0] = ((int *) frame->data)[0];
		can_buf[0].data[1] = ((int *) frame->data)[1];
		can_buf[0].data[2] = ((int *) frame->data)[2];
		can_buf[0].data[3] = ((int *) frame->data)[3];

		/* Ask the transfert */
        C1TR01CONbits.TXREQ0 = 1;
		return true;
	}
	return false;
}

/**
	Disable the CAN device.
*/
void can_disable(void)
{
	can_ask_runlevel(CAN_DISABLE_MODE);
}

/** 
	Reenable the previously disabled CAN device.
*/
void can_enable(void)
{
	can_ask_runlevel(CAN_NORMAL_MODE);
}

/**
	Check if there is room to send a frame.
	
	\return	return true if there is enough space to send the frame, false if the send buffer is full.
*/
bool can_is_frame_room(void)
{
	return (C1TR01CONbits.TXREQ0 != 1);
}


/** Check if the FIFO has someting in.
	\return return -1 if empty, the next buffer to read otherwise
*/
static int can_get_next_rx(void) {
	int bufn = C1FIFObits.FNRB;
	if(bufn > 15) {
		if((C1RXFUL2 >> (bufn - 16)) & 0x1)
			return bufn;
		else 
			return -1;
	} else {
		if((C1RXFUL1 >> bufn) & 0x1)
			return bufn;
		else
			return -1;
	}
}	

/*! The CAN RX interrupt handler
 * \sa _C1Interrupt
 */
static void can_rx(void)
{
	int bufn = 0;
	can_frame frame;

	while((bufn = can_get_next_rx()) != -1)
	{
		frame.id = (can_buf[bufn].sid >> 2) & 0x7FF;
		frame.len = can_buf[bufn].dlc & 0xF;
		((int *) frame.data)[0] = can_buf[bufn].data[0];
		((int *) frame.data)[1] = can_buf[bufn].data[1];
		((int *) frame.data)[2] = can_buf[bufn].data[2];
		((int *) frame.data)[3] = can_buf[bufn].data[3];
	
		if(bufn > 15) {
			C1RXFUL2 &= ~(1<<(bufn-16));
			C1RXOVF2 &= ~(1<<(bufn-16));
		} else {
			C1RXFUL1 &= ~(1<<bufn);
			C1RXOVF1 &= ~(1<<bufn);
		}
	
		rx_cb(&frame);
	}
}

static void can_tx(void)
{
	if(!C1TR01CONbits.TXREQ0) 
		if(tx_done_cb)
			tx_done_cb();
}


void _ISR _C1Interrupt(void)
{
	IFS2bits.C1IF = 0;
	if(C1INTFbits.RBIF)
	{
		C1INTFbits.RBIF = 0;
		can_rx();
	}
	if(C1INTFbits.TBIF)
	{
		C1INTFbits.TBIF = 0;
		can_tx();
	}
	if(C1INTFbits.ERRIF)
	{
		/* FIXME */
		C1INTFbits.ERRIF = 0;
	}
	if(C1INTFbits.IVRIF)
	{
		/* FIXME */
		C1INTFbits.IVRIF = 0;
	}
}
