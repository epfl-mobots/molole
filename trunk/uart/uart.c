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
	\defgroup uart UART
	
	Wrappers around UART, with a callback oriented interface.
	
	\section Usage
	
	The following codes echos incoming data on both UARTs:
\code
// for Idle()
#include <p33fxxxx.h>
// use several molole libraries
#include <types/types.h>
#include <error/error.h>
#include <uart/uart.h>
#include <clock/clock.h>

// functions for callbacks
void uart1_byte_received(int uart_id, unsigned char data)
{
	uart_1_transmit_byte(data);
}

bool uart1_byte_transmitted(int uart_id, unsigned char* data)
{
	return false;
}

void uart2_byte_received(int uart_id, unsigned char data)
{
	uart_2_transmit_byte(data);
}

bool uart2_byte_transmitted(int uart_id, unsigned char* data)
{
	return false;
}

int main(void)
{
	// initialise clocks
	clock_init_internal_rc();
	
	// initialise both UART at 115200, no flow control
	uart_1_init(115200, false, uart1_byte_received, uart1_byte_transmitted, 1);
	uart_2_init(115200, false, uart2_byte_received, uart2_byte_transmitted, 1);
	
	// sleep
	while(1) 
	{
		Idle();	// do nothing
	}
	return 0;
}
\endcode
*/
/*@{*/

/** \file
	Implementation of functions common to wrapper around UART.
*/


//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "uart.h"
#include "../clock/clock.h"

//-----------------------
// Structures definitions
//-----------------------

/** UART wrapper data */
typedef struct 
{
	uart_byte_received byte_received_callback; /**< function to call when a new byte is received */
	uart_byte_transmitted byte_transmitted_callback; /**< function to call when a byte has been transmitted */
} UART_Data;

/** data for UART 1 wrapper */
static UART_Data UART_1_Data;

/** data for UART 2 wrapper */
static UART_Data UART_2_Data;


//-------------------
// Exported functions
//-------------------

/**
	Init UART 1 subsystem.
	
	The parameters are 8 bits, 1 stop bit, no parity.
	
	\param	baud_rate
			baud rate in bps
	\param	byte_received_callback
			function to call when a new byte is received
	\param	byte_transmitted_callback
			function to call when a byte has been transmitted
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 7 (highest priority)
*/
void uart_1_init(unsigned long baud_rate, bool hardware_flow_control, uart_byte_received byte_received_callback, uart_byte_transmitted byte_transmitted_callback, int priority)
{
	// Store callback functions
	UART_1_Data.byte_received_callback = byte_received_callback;
	UART_1_Data.byte_transmitted_callback = byte_transmitted_callback;
	
	
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
	
	// Setup other parameters
	U1MODEbits.USIDL = 0;	// Continue module operation in idle mode
	U1MODEbits.STSEL = 0;	// 1-stop bit
	U1MODEbits.PDSEL = 0;	// No Parity, 8-data bits
	U1MODEbits.ABAUD = 0;	// Autobaud Disabled
	if (hardware_flow_control)
		U1MODEbits.UEN = 2;	// Do hardware flow control. Use RTS and CTS, but not use BCLK
	else
		U1MODEbits.UEN = 0;	// Do not do any hardware flow control. RTS and CTS are left as GPIO
	
	// Setup interrupts
	_U1RXIF = 0;			// clear the reception interrupt
	_U1RXIP = priority;   	// set the reception interrupt priority
	_U1RXIE = 1;			// enable the reception interrupt
	
	_U1TXIF = 0;			// clear the transmission interrupt
	_U1TXIP = priority;   	// set the transmission interrupt priority
	_U1TXIE = 1;			// enable the transmission interrupt

	U1MODEbits.UARTEN = 1;	// Enable UART
	U1STAbits.UTXEN = 1; 	// Enable transmit
}

/**
	Transmit a byte on UART 1.
	
	\param	data
			byte to transmit
	\return true if byte was transmitted, false if transmit buffer was full
*/
bool uart_1_transmit_byte(unsigned char data)
{
	if (U1STAbits.UTXBF)
		return false;
	
	U1TXREG = data;
	return true;
}


/**
	Init UART 2 subsystem.
	
	\param	baud_rate
			baud rate in bps
	\param	byte_received_callback
			function to call when a new byte is received
	\param	byte_transmitted_callback
			function to call when a byte has been transmitted
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 7 (highest priority)
*/
void uart_2_init(unsigned long baud_rate, bool hardware_flow_control, uart_byte_received byte_received_callback, uart_byte_transmitted byte_transmitted_callback, int priority)
{
	// Store callback functions
	UART_2_Data.byte_received_callback = byte_received_callback;
	UART_2_Data.byte_transmitted_callback = byte_transmitted_callback;
	
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
	
	// Setup other parameters
	U2MODEbits.USIDL = 0;	// Continue module operation in idle mode
	U2MODEbits.STSEL = 0;	// 1-stop bit
	U2MODEbits.PDSEL = 0;	// No Parity, 8-data bits
	U2MODEbits.ABAUD = 0;	// Autobaud Disabled
	if (hardware_flow_control)
		U2MODEbits.UEN = 2;	// Do hardware flow control. Use RTS and CTS, but not use BCLK
	else
		U2MODEbits.UEN = 0;	// Do not do any hardware flow control. RTS and CTS are left as GPIO
	
	// Setup interrupts
	_U2RXIF = 0;			// clear the reception interrupt
	_U2RXIP = priority;   	// set the reception interrupt priority
	_U2RXIE = 1;			// enable the reception interrupt
	
	_U2TXIF = 0;			// clear the transmission interrupt
	_U2TXIP = priority;   	// set the transmission interrupt priority
	_U2TXIE = 1;			// enable the transmission interrupt

	U2MODEbits.UARTEN = 1;	// Enable UART
	U2STAbits.UTXEN = 1; 	// Enable transmit
}

/**
	Transmit a byte on UART 2.
	
	\param	data
			byte to transmit
	\return true if byte was transmitted, false if transmit buffer was full
*/
bool uart_2_transmit_byte(unsigned char data)
{
	if (U2STAbits.UTXBF)
		return false;
	
	U2TXREG = data;
	return true;
}



//--------------------------
// Interrupt service routine
//--------------------------

/**
	UART 1 Reception Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _U1RXInterrupt(void)
{
	UART_1_Data.byte_received_callback(UART_1, U1RXREG);
	
	_U1RXIF = 0;			// Clear reception interrupt flag
}

/**
	UART 1 Transmission Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _U1TXInterrupt(void)
{
	unsigned char data;
	if (UART_1_Data.byte_transmitted_callback(UART_1, &data))
		U1TXREG = data;
	
	_U1TXIF = 0;			// Clear transmission interrupt flag
}

/**
	UART 2 Reception Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _U2RXInterrupt(void)
{
	UART_2_Data.byte_received_callback(UART_2, U2RXREG);
	
	_U2RXIF = 0;			// Clear reception interrupt flag
}

/**
	UART 2 Transmission Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _U2TXInterrupt(void)
{
	unsigned char data;
	if (UART_2_Data.byte_transmitted_callback(UART_2, &data))
		U2TXREG = data;
	
	_U2TXIF = 0;			// Clear transmission interrupt flag
}


/*@}*/
