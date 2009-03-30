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

//--------------------
// Usage documentation
//--------------------

/**
	\defgroup uart_soft_fc UART With software flow control
	
	Wrappers around UART, with a callback oriented interface.
	
	\section Usage
	
	The usage is the same as the normal uart module. Only the init routine change.
*/
/*@{*/

/** \file
	Implementation of functions common to wrapper around UART.
*/


//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "uart-software-fc.h"
#include "../error/error.h"
#include "../clock/clock.h"
#include "../gpio/gpio.h"
#include "../timer/timer.h"


#define FIFO_POWER_SIZE 5
#define STOP_RX_LEVEL (1 << (FIFO_POWER_SIZE - 1))
#define FIFO_MASK ((1 << FIFO_POWER_SIZE) - 1)
#define FIFO_SIZE (1 << FIFO_POWER_SIZE)

//-----------------------
// Structures definitions
//-----------------------

/** UART wrapper data */
typedef struct 
{
	uart_byte_received byte_received_callback; /**< function to call when a new byte is received */
	uart_tx_ready tx_ready_callback; /**< function to call when a byte has been transmitted */
	bool user_program_busy; /**< true if user program is busy and cannot read any more data, false otherwise */
	unsigned char bh_ipl; /**< the rx interrupt callback priority level */
	unsigned char th_ipl; /**< The hardware RX priority level */
	void* user_data; /**< pointer to user-specified data to be passed in interrupt, may be 0 */
	unsigned char internal_buffer[FIFO_SIZE]; /**< internal buffer to handle flow control */
	// No need to protect the fifo: only 1 write and reader at any time and size of power of 2 used
	unsigned int fifo_r; /**< Fifo read pointer */
	unsigned int fifo_w; /**< Fifo write pointer */
	gpio cts; /**< CTS line */
	gpio rts; /**< RTS line */
	int timer_id;	/**< Timer to poll the RTS line */
	int stop_tx; 
	unsigned int fake_timer;
} UART_Data;

/** data for UART 1 wrapper */
static UART_Data UART_1_Data;

/** data for UART 2 wrapper */
static UART_Data UART_2_Data;


//-------------------
// Exported functions
//-------------------
void uart2_timer_cb(int __attribute((unused)) timer_id);
void uart1_timer_cb(int __attribute((unused)) timer_id);

/**
	Init an UART subsystem.
	
	The parameters are 8 bits, 1 stop bit, no parity.
	
	\param	uart_id
			identifier of the UART, \ref UART_1 or \ref UART_2
	\param	baud_rate
			baud rate in bps
	\param  cts
			The CTS gpio line
	\param rts
			The RTS gpio line
	\param timer_id
			The timer used to poll the CTS line
	\param	byte_received_callback
			function to call when a new byte is received
	\param	byte_transmitted_callback
			function to call when a byte has been transmitted
	\param  th_priority
			Hardware fifo read interrupt priority, from bh_priority + 1 to 7 (NMI priority)
	\param 	bh_priority
			The callback interrupt priority, from 0 (lowest priority) to 6 (highest normal priority)
	\param 	user_data
			Pointer to user-specified data to be passed in interrupt, may be 0
*/
void uart_init(int uart_id, unsigned long baud_rate, gpio cts, gpio rts, int timer_id, uart_byte_received byte_received_callback, uart_tx_ready tx_ready_callback, int th_priority, int bh_priority, void* user_data)
{
	ERROR_CHECK_RANGE(th_priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	ERROR_CHECK_RANGE(bh_priority, 0, 6, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	if(bh_priority >= th_priority) {
		ERROR(GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY, &bh_priority);
	}

	if (uart_id == UART_1)
	{
		// Store callback functions
		UART_1_Data.byte_received_callback = byte_received_callback;
		UART_1_Data.tx_ready_callback = tx_ready_callback;
		UART_1_Data.user_data = user_data;
		UART_1_Data.cts = cts;
		UART_1_Data.rts = rts;
		UART_1_Data.timer_id = timer_id;
		UART_1_Data.th_ipl = th_priority;
		UART_1_Data.bh_ipl = bh_priority;
		
		gpio_write(rts, true);
		gpio_set_dir(rts, GPIO_OUTPUT);
		gpio_set_dir(cts, GPIO_INPUT);
		
		// Setup baud rate
		/*
		UART high speed mode is buggy on current dsPIC 33, see erratas for details
		if (baud_rate <= CLOCK_FCY / 16)
		{
			U1MODEbits.BRGH = 0;// Low Speed mode
			U1BRG = (clock_get_cycle_frequency() / baud_rate) / 16 - 1;
		}
		else
		{
			U1MODEbits.BRGH = 1;// High Speed mode
			U1BRG = clock_get_cycle_frequency() / (4*baud_rate) - 1;
		}
		*/
		U1MODEbits.BRGH = 0;	// Low Speed mode
		U1BRG = (clock_get_cycle_frequency()/baud_rate) / 16 - 1;
		
		if(baud_rate < 1000) 
			baud_rate = 1000;
			
		timer_init(timer_id, (1000000UL/(baud_rate/100)), 6); /* 1/(baud/100) s, maximum 0.1 sec.*/
		timer_enable_interrupt(timer_id, uart1_timer_cb, bh_priority);
		
		// Setup other parameters
		U1MODEbits.USIDL = 0;	// Continue module operation in idle mode
		U1MODEbits.STSEL = 0;	// 1-stop bit
		U1MODEbits.PDSEL = 0;	// No Parity, 8-data bits
		U1MODEbits.ABAUD = 0;	// Autobaud Disabled
		
		U1MODEbits.UEN = 0;	// Do not do any hardware flow control. RTS and CTS are left as GPIO
		
		
		// Setup interrupts
		_U1RXIF = 0;			// clear the reception interrupt
		_U1RXIP = th_priority;   	// set the reception interrupt priority
		_U1RXIE = 1;			// enable the reception interrupt
		
		U1MODEbits.UARTEN = 1;	// Enable UART
		U1STAbits.UTXEN = 1; 	// Enable transmit

		_U1TXIP = bh_priority;   	// set the transmission interrupt priority		
		_U1TXIF = 0;			// clear the transmission interrupt
		_U1TXIE = 1;			// enable the transmission interrupt
	
	
		
		gpio_write(rts, false);
	}
	else if (uart_id == UART_2)
	{
		// Store callback functions
		UART_2_Data.byte_received_callback = byte_received_callback;
		UART_2_Data.tx_ready_callback = tx_ready_callback;
		UART_2_Data.user_data = user_data;
		UART_2_Data.cts = cts;
		UART_2_Data.rts = rts;
		UART_2_Data.timer_id = timer_id;
		UART_2_Data.th_ipl = th_priority;
		UART_2_Data.bh_ipl = bh_priority;
		
		gpio_write(rts, true);
		gpio_set_dir(rts, GPIO_OUTPUT);
		gpio_set_dir(cts, GPIO_INPUT);
		
		// Setup baud rate
		/*
		UART high speed mode is buggy on current dsPIC 33, see erratas for details
		if (baud_rate <= clock_get_cycle_frequency() / 16)
		{
			U2MODEbits.BRGH = 0;// Low Speed mode
			U2BRG = (clock_get_cycle_frequency() / baud_rate) / 16 - 1;
		}
		else
		{
			U2MODEbits.BRGH = 1;// High Speed mode
			U2BRG = clock_get_cycle_frequency() / (4*baud_rate) - 1;
		}
		*/
		U2MODEbits.BRGH = 0;	// Low Speed mode
		U2BRG = (clock_get_cycle_frequency()/baud_rate) / 16 - 1;
		
		if(baud_rate < 1000) 
			baud_rate = 1000;
			
		timer_init(timer_id, (1000000UL/(baud_rate/100)), 6); /* 1/(baud/100) s, maximum 0.1 sec.*/
		timer_enable_interrupt(timer_id, uart2_timer_cb, bh_priority);
		
		// Setup other parameters
		U2MODEbits.USIDL = 0;	// Continue module operation in idle mode
		U2MODEbits.STSEL = 0;	// 1-stop bit
		U2MODEbits.PDSEL = 0;	// No Parity, 8-data bits
		U2MODEbits.ABAUD = 0;	// Autobaud Disabled

		U2MODEbits.UEN = 0;	// Do not do any hardware flow control. RTS and CTS are left as GPIO
		
		// Setup interrupts
		_U2RXIF = 0;			// clear the reception interrupt
		_U2RXIP = th_priority; 	
		_U2RXIE = 1;			// enable the reception interrupt
		
		U2MODEbits.UARTEN = 1;	// Enable UART
		U2STAbits.UTXEN = 1; 	// Enable transmit
		
		_U2TXIP = bh_priority;  // set the transmission interrupt priority
		_U2TXIF = 0;			// clear the transmission interrupt
		_U2TXIE = 1;			// enable the transmission interrupt
		
		gpio_write(rts, false);
	}
	else
	{
		ERROR(UART_ERROR_INVALID_ID, &uart_id);
	}
}

/**
	Transmit a byte on UART.
	
	\param	uart_id
			identifier of the UART, \ref UART_1 or \ref UART_2
	\param	data
			byte to transmit
	\return true if byte was transmitted, false if transmit buffer was full
*/
bool uart_transmit_byte(int uart_id, unsigned char data)
{
	int flags;
	if (uart_id == UART_1)
	{	
		flags = uart_disable_tx_interrupt(uart_id);
		if (gpio_read(UART_1_Data.cts)) {
			UART_1_Data.stop_tx = 1;
			uart_enable_tx_interrupt(uart_id, flags);
			timer_enable(UART_1_Data.timer_id);
			return false;
		}
		
		if(U1STAbits.UTXBF) {
			uart_enable_tx_interrupt(uart_id, flags);
			return false;
		}
		
		U1TXREG = data;
		
		uart_enable_tx_interrupt(uart_id, flags);
		return true;
	}
	else if (uart_id == UART_2)
	{
		flags = uart_disable_tx_interrupt(uart_id);
		if (gpio_read(UART_2_Data.cts)) {
			UART_2_Data.stop_tx = 1;
			uart_enable_tx_interrupt(uart_id, flags);
			timer_enable(UART_2_Data.timer_id);
			return false;
		}
		if(U2STAbits.UTXBF) {
			uart_enable_tx_interrupt(uart_id, flags);
			return false;
		}
		
		U2TXREG = data;
		
		uart_enable_tx_interrupt(uart_id, flags);
		
		return true;
	} else {
		ERROR(UART_ERROR_INVALID_ID, &uart_id);
	}
}

/**
	Read pending data on UART until there is no more data or callback returned false.
	
	\param	uart_id
			identifier of the UART, \ref UART_1 or \ref UART_2
*/
void uart_read_pending_data(int uart_id)
{
	if (uart_id == UART_1)
	{
		if (UART_1_Data.user_program_busy)
		{
			barrier();
			// It is valid to access the fifo here only because UART_1_Data.user_program_busy == true
			// So the softirq will not access it concurrently
			while(UART_1_Data.fifo_w - UART_1_Data.fifo_r)
			{
				if (UART_1_Data.byte_received_callback(UART_1, UART_1_Data.internal_buffer[(UART_1_Data.fifo_r++) & FIFO_MASK], UART_1_Data.user_data) == false)
				{
					if(UART_1_Data.fifo_w - UART_1_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_1_Data.rts, false);
					return;
				}
			}
			if(UART_1_Data.fifo_w - UART_1_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_1_Data.rts, false);
			UART_1_Data.user_program_busy = false;
		}
	}
	else if (uart_id == UART_2)
	{
		if (UART_2_Data.user_program_busy)
		{
			barrier();
			// It is valid to access the fifo here only because UART_2_Data.user_program_busy == true
			// So the softirq will not access it concurrently
			while(UART_2_Data.fifo_w - UART_2_Data.fifo_r)
			{
				if (UART_2_Data.byte_received_callback(UART_2, UART_2_Data.internal_buffer[(UART_2_Data.fifo_r++) & FIFO_MASK], UART_2_Data.user_data) == false)
				{
					if(UART_2_Data.fifo_w - UART_2_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_2_Data.rts, false);
					return;
				}
			}
			if(UART_2_Data.fifo_w - UART_2_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_2_Data.rts, false);
			UART_2_Data.user_program_busy = false;
		}
	}
	else
	{
		ERROR(UART_ERROR_INVALID_ID, &uart_id);
	}
}

/**
	Disable the TX interrupt
	
	\param	uart_id
			identifier of the UART, \ref UART_1 or \ref UART_2
*/
int uart_disable_tx_interrupt(int uart_id) {
	int flags = 0;
	if(uart_id == UART_1) {
		// First Uart irq, then the timer. The order IS important
		if(_U1TXIE) {
			flags |= 0x1;
			_U1TXIE = 0;
		} 
		if(UART_1_Data.stop_tx == 1) {
			timer_disable(UART_1_Data.timer_id);
			UART_1_Data.stop_tx = 2;
			flags |= 0x2;
		}
		return flags;

	} else if(uart_id == UART_2) {
		// First Uart irq, then the timer. The order IS important
		if(_U2TXIE) {
			flags |= 0x1;
			_U2TXIE = 0;
		} 
		if(UART_2_Data.stop_tx == 1) {
			timer_disable(UART_2_Data.timer_id);
			UART_2_Data.stop_tx = 2;
			flags |= 0x2;
		}
		return flags;

	} else {
		ERROR(UART_ERROR_INVALID_ID, &uart_id);
	}
}

/**
	Re-enable the TX interrupt
	
	\param	uart_id
			identifier of the UART, \ref UART_1 or \ref UART_2
	\param flags
			The return value of the \ref uart_disable_tx_interrupt function
*/
void uart_enable_tx_interrupt(int uart_id, int flags) {
	if(uart_id == UART_1) {
		if(flags & 0x1) 
			_U1TXIE = 1;
		if(flags & 0x2) {
			UART_1_Data.stop_tx = 1;
			timer_enable(UART_1_Data.timer_id);
		}
	} else if(uart_id == UART_2) {
		if(flags & 0x1) 
			_U2TXIE = 1;
		if(flags & 0x2) {
			UART_2_Data.stop_tx = 1;
			timer_enable(UART_2_Data.timer_id);
		}

	} else {	
		ERROR(UART_ERROR_INVALID_ID, &uart_id);
	}
}


//--------------------------
// Interrupt service routine
//--------------------------

/**
	UART 1 Reception Interrupt Service Routine.
 
	Call the user-defined function if user program is not busy.
*/
static unsigned int __attribute((near)) retaddr1;
void __attribute((interrupt(preprologue("push w0\n"
										"mov [w15-4],   w0\n"
										"mov w0, _retaddr1\n"
										"pop w0\n"))))  _U1RXInterrupt(void)
{
	static int inside_softirq;
	
	
// TOP HALF PART
	_U1RXIF = 0;			// Clear reception interrupt flag

	while(U1STAbits.URXDA) {
		_U1RXIF = 0;			// Clear reception interrupt flag
								// Why ? because if we recieve a character
								// while the callback run, we don't want to get recalled immediatly 
								// after going out
		// warning: I do not protect the fifo against overrun, because flow control will not allow us to fill it completly
		if(U1STAbits.FERR)
			// Frame error, grabbage on uart
			(void *) U1RXREG;
		else
			UART_1_Data.internal_buffer[(UART_1_Data.fifo_w++) & FIFO_MASK] = U1RXREG;
		if(UART_1_Data.fifo_w - UART_1_Data.fifo_r > STOP_RX_LEVEL) 
			gpio_write(UART_1_Data.rts, true);
			
	}
	
	// Work around for the dsPIC33 Rev. A2 Silicon Errata
	// Clear Receive Buffer Overrun Error if any, possible despite the use of hardware handshake
	
	if(U1STAbits.OERR)
		U1STAbits.OERR = 0;	
	
	// We are already in the softirq part, avoid recursion
	if(inside_softirq)
		return;
		
	// We preempted something with same or higher priority, don't run the bh now.
	if(retaddr1 >> 13 >= UART_1_Data.bh_ipl) {
		// Defer the irq to the TX interrupt with the same priority
		UART_1_Data.fake_timer = 1;
		if(!timer_force_interrupt(UART_1_Data.timer_id)) {
			UART_1_Data.fake_timer = 2;
		}

		return;	
	}
	
	inside_softirq = 1;
	
	while(UART_1_Data.fifo_w - UART_1_Data.fifo_r && !UART_1_Data.user_program_busy) {
		barrier();
		SET_IPL(UART_1_Data.bh_ipl);
	
		// BOTTOM HALF PART
		//----------
		
		if (!UART_1_Data.user_program_busy)
		{
			while(UART_1_Data.fifo_w - UART_1_Data.fifo_r)
			{
				if (UART_1_Data.byte_received_callback(UART_1, UART_1_Data.internal_buffer[(UART_1_Data.fifo_r++) & FIFO_MASK], UART_1_Data.user_data) == false)
				{
					if(UART_1_Data.fifo_w - UART_1_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_1_Data.rts, false);
					
					UART_1_Data.user_program_busy = true;
					break;
				} else {
					if(UART_1_Data.fifo_w - UART_1_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_1_Data.rts, false);
				}
			}
		}
		
		
		//-----------
		// END OF BOTTOM HALF PART
		
		// To do this hardirq-safe I must protect the thiny race window while I set inside_softirq = 0 and "retfie" is executed.
		// So, raise our priority level to the irq level
		SET_IPL(UART_1_Data.th_ipl);
	}
	
	
	inside_softirq = 0;
}

/**
	UART 1 Transmission Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _U1TXInterrupt(void)
{
	unsigned char data;

	_U1TXIF = 0;			// Clear transmission interrupt flag

	if(gpio_read(UART_1_Data.cts)) {
		// Stop TX and start polling timer
		if(!UART_1_Data.stop_tx) {
			UART_1_Data.stop_tx = 1;
			timer_enable(UART_1_Data.timer_id);
		}
		return;
	}

	while(!U1STAbits.UTXBF && UART_1_Data.tx_ready_callback(UART_1, &data, UART_1_Data.user_data))
		U1TXREG = data;
}

/** UART 1 TX flow control timer
 	
 	Check if the CTS line is still high
*/
 
void uart1_timer_cb(int  timer_id) {
	unsigned char data;
	
	if(UART_1_Data.fake_timer) {
		if(UART_1_Data.fake_timer == 2) 
			timer_disable_interrupt(timer_id);

		UART_1_Data.fake_timer = 0;
		if (!UART_1_Data.user_program_busy)
		{
			while(UART_1_Data.fifo_w - UART_1_Data.fifo_r)
			{
				if (UART_1_Data.byte_received_callback(UART_1, UART_1_Data.internal_buffer[(UART_1_Data.fifo_r++) & FIFO_MASK], UART_1_Data.user_data) == false)
				{
					if(UART_1_Data.fifo_w - UART_1_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_1_Data.rts, false);
					
					UART_1_Data.user_program_busy = true;
					break;
				} else {
					if(UART_1_Data.fifo_w - UART_1_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_1_Data.rts, false);
				}
			}
		}
		return;
	}
	
	if(!gpio_read(UART_1_Data.cts)) {
		// Restart TX
		if(UART_1_Data.stop_tx)
			if(!U1STAbits.UTXBF && UART_1_Data.tx_ready_callback(UART_1, &data, UART_1_Data.user_data))
				U1TXREG = data;
		
		timer_disable(UART_1_Data.timer_id);
		UART_1_Data.stop_tx = 0;	
	}
}
 

/**
	UART 2 Reception Interrupt Service Routine.
 
	Call the user-defined function if user program is not busy.
*/
static unsigned int __attribute((near)) retaddr2;
void __attribute((interrupt(preprologue("push w0\n"
										"mov [w15-4],   w0\n"
										"mov w0, _retaddr2\n"
										"pop w0\n")))) _U2RXInterrupt(void)
{
	static int inside_softirq;
	
	
// TOP HALF PART
	_U2RXIF = 0;			// Clear reception interrupt flag

	while(U2STAbits.URXDA) {
		_U2RXIF = 0;			// Clear reception interrupt flag
								// Why ? because if we recieve a character
								// while the callback run, we don't want to get recalled immediatly 
								// after going out
		// warning: I do not protect the fifo against overrun, because flow control will not allow us to fill it completly
		if(U2STAbits.FERR)
			// Frame error, grabbage on uart
			(void *) U2RXREG;
		else
			UART_2_Data.internal_buffer[(UART_2_Data.fifo_w++) & FIFO_MASK] = U2RXREG;
		if(UART_2_Data.fifo_w - UART_2_Data.fifo_r > STOP_RX_LEVEL) 
			gpio_write(UART_2_Data.rts, true);
			
	}
	
	// Work around for the dsPIC33 Rev. A2 Silicon Errata
	// Clear Receive Buffer Overrun Error if any, possible despite the use of hardware handshake
	
	if(U2STAbits.OERR)
		U2STAbits.OERR = 0;	
	
	// We are already in the softirq part, avoid recursion
	if(inside_softirq)
		return;
		
		// We preempted something with same or higher priority, don't run the bh now.
	if(retaddr2 >> 13 >= UART_2_Data.bh_ipl) {
		UART_2_Data.fake_timer = 1;
		if(!timer_force_interrupt(UART_2_Data.timer_id)) {
			UART_2_Data.fake_timer = 2;
		}
		return;
	}
	
	inside_softirq = 1;
	
	while(UART_2_Data.fifo_w - UART_2_Data.fifo_r && !UART_2_Data.user_program_busy) {
		barrier();
		SET_IPL(UART_2_Data.bh_ipl);
	
		// BOTTOM HALF PART
		//----------
		
		if (!UART_2_Data.user_program_busy)
		{
			while(UART_2_Data.fifo_w - UART_2_Data.fifo_r)
			{
				if (UART_2_Data.byte_received_callback(UART_2, UART_2_Data.internal_buffer[(UART_2_Data.fifo_r++) & FIFO_MASK], UART_2_Data.user_data) == false)
				{
					if(UART_2_Data.fifo_w - UART_2_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_2_Data.rts, false);
					
					UART_2_Data.user_program_busy = true;
					break;
				} else {
					if(UART_2_Data.fifo_w - UART_2_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_2_Data.rts, false);
				}
			}
		}
		
		
		//-----------
		// END OF BOTTOM HALF PART
		
		// To do this hardirq-safe I must protect the thiny race window while I set inside_softirq = 0 and "retfie" is executed.
		// So, raise our priority level to the max
		SET_IPL(UART_2_Data.th_ipl);
	}
	
	
	inside_softirq = 0;
}

/**
	UART 2 Transmission Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _U2TXInterrupt(void)
{
	unsigned char data;

	_U2TXIF = 0;			// Clear transmission interrupt flag

	if(gpio_read(UART_2_Data.cts)) {
		// Stop TX and start polling timer
		if(!UART_2_Data.stop_tx) {
			UART_2_Data.stop_tx = 1;
			timer_enable(UART_2_Data.timer_id);
		}
		return;
	}

	while(!U2STAbits.UTXBF && UART_2_Data.tx_ready_callback(UART_2, &data, UART_2_Data.user_data))
		U2TXREG = data;
}

/** UART 2 TX flow control timer
 	
 	Check if the CTS line is still high
*/
 
void uart2_timer_cb(int __attribute((unused)) timer_id) {
	unsigned char data;
	
	if(UART_2_Data.fake_timer) {
		if(UART_2_Data.fake_timer == 2) 
			timer_disable_interrupt(timer_id);
			
		UART_2_Data.fake_timer = 0;
		if (!UART_2_Data.user_program_busy)
		{
			while(UART_2_Data.fifo_w - UART_2_Data.fifo_r)
			{
				if (UART_2_Data.byte_received_callback(UART_2, UART_2_Data.internal_buffer[(UART_2_Data.fifo_r++) & FIFO_MASK], UART_2_Data.user_data) == false)
				{
					if(UART_2_Data.fifo_w - UART_2_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_2_Data.rts, false);
					
					UART_2_Data.user_program_busy = true;
					break;
				} else {
					if(UART_2_Data.fifo_w - UART_2_Data.fifo_r < STOP_RX_LEVEL) 
						// Restart RX
						gpio_write(UART_2_Data.rts, false);
				}
			}
		}
		return;
	}
	
	if(!gpio_read(UART_2_Data.cts)) {
		// Restart TX
		if(UART_2_Data.stop_tx)
			if(!U2STAbits.UTXBF && UART_2_Data.tx_ready_callback(UART_2, &data, UART_2_Data.user_data))
				U2TXREG = data;
		timer_disable(UART_2_Data.timer_id);
		UART_2_Data.stop_tx = 0;
	}
}
 


/*@}*/
