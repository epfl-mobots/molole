/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	Copyright (C) 2007 Stephane Magnenat <stephane at magnenat dot net>
	
	Mobots group http://mobots.epfl.ch
	Robotics system laboratory http://lsro.epfl.ch
	EPFL Ecole polytechnique federale de Lausanne: http://www.epfl.ch
	
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
UART_Data UART_1_Data;

/** data for UART 2 wrapper */
UART_Data UART_2_Data;


//-------------------
// Exported functions
//-------------------

/**
	Init UART 1 subsystem.
	
	\param	baud_rate
			baud rate in bps
	\param	byte_received_callback
			function to call when a new byte is received
	\param	byte_transmitted_callback
			function to call when a byte has been transmitted
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 7 (highest priority)
*/
void uart_1_init(unsigned long baud_rate, uart_byte_received byte_received_callback, uart_byte_transmitted byte_transmitted_callback, int priority)
{
	// Store callback functions
	UART_1_Data.byte_received_callback = byte_received_callback;
	UART_1_Data.byte_transmitted_callback = byte_transmitted_callback;
	
	// Setup baud rate
	if (baud_rate <= CLOCK_FCY / 16)
	{
		U1MODEbits.BRGH = 0;// Low Speed mode
		U1BRG = (((unsigned long)CLOCK_FCY)/baud_rate) / 16 - 1;
	}
	else
	{
		U1MODEbits.BRGH = 1;// High Speed mode
		U1BRG = ((unsigned long)CLOCK_FCY) / (4*baud_rate) - 1;
	}
	
	// Setup other parameters
	U1MODEbits.USIDL = 0;	// Continue module operation in idle mode
	U1MODEbits.STSEL = 0;	// 1-stop bit
	U1MODEbits.PDSEL = 0;	// No Parity, 8-data bits
	U1MODEbits.ABAUD = 0;	// Autobaud Disabled
	
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
void uart_2_init(unsigned long baud_rate, uart_byte_received byte_received_callback, uart_byte_transmitted byte_transmitted_callback, int priority)
{
	// Store callback functions
	UART_2_Data.byte_received_callback = byte_received_callback;
	UART_2_Data.byte_transmitted_callback = byte_transmitted_callback;
	
	// Setup baud rate
	if (baud_rate <= CLOCK_FCY / 16)
	{
		U2MODEbits.BRGH = 0;// Low Speed mode
		U2BRG = (((unsigned long)CLOCK_FCY)/baud_rate) / 16 - 1;
	}
	else
	{
		U2MODEbits.BRGH = 1;// High Speed mode
		U2BRG = ((unsigned long)CLOCK_FCY) / (4*baud_rate) - 1;
	}
	
	// Setup other parameters
	U2MODEbits.USIDL = 0;	// Continue module operation in idle mode
	U2MODEbits.STSEL = 0;	// 1-stop bit
	U2MODEbits.PDSEL = 0;	// No Parity, 8-data bits
	U2MODEbits.ABAUD = 0;	// Autobaud Disabled
	
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
	UART_1_Data.byte_received_callback(1, U1RXREG);
	
	_U1RXIF = 0;			// Clear reception interrupt flag
}

/**
	UART 1 Transmission Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _U1TXInterrupt(void)
{
	unsigned char data;
	if (UART_1_Data.byte_transmitted_callback(1, &data))
		U1TXREG = data;
	
	_U1TXIF = 0;			// Clear transmission interrupt flag
}

/**
	UART 2 Reception Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _U2RXInterrupt(void)
{
	UART_2_Data.byte_received_callback(2, U2RXREG);
	
	_U2RXIF = 0;			// Clear reception interrupt flag
}

/**
	UART 2 Transmission Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _U2TXInterrupt(void)
{
	unsigned char data;
	if (UART_2_Data.byte_transmitted_callback(2, &data))
		U2TXREG = data;
	
	_U2TXIF = 0;			// Clear transmission interrupt flag
}


/*@}*/
