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
	\defgroup serialio Serial input/output library
	
	An input/output library using UART.
	It provides buffered operations, basic types parsing, and terminal support.
	It is internally state-less, all state is contained in \ref Serial_IO_State.
*/
/*@{*/

/** \file
	The implementation of a serial input/output library
*/


//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "serial-io.h"
#include "../error/error.h"

//-------------------
// Internal functions
//-------------------

/** Callback when a byte is received.
	Return true if new data is accepted, false otherwise. */
bool serial_io_byte_received(int uart_id, unsigned char data, void* user_data)
{
	Serial_IO_State* state = (Serial_IO_State*)user_data;
	
	state->reception_buffer[state->reception_buffer_reception_pos] = (char)data;
	
	state->reception_buffer_reception_pos = (state->reception_buffer_reception_pos + 1) % SERIAL_IO_BUFFERS_SIZE;
	
	// block if full
	if ((state->reception_buffer_reception_pos + 1) % SERIAL_IO_BUFFERS_SIZE == state->reception_buffer_read_pos)
		return false;
	else
		return true;
}

/** Callback when a byte has been transmitted.
	Return true if a new one should be sent, false otherwise. */
bool serial_io_byte_transmitted(int uart_id, unsigned char* data, void* user_data)
{
	Serial_IO_State* state = (Serial_IO_State*)user_data;
	
	if (state->transmission_buffer_transmit_pos == state->transmission_buffer_write_pos)
		return false;
	
	*data = state->transmission_buffer[state->transmission_buffer_transmit_pos];
	
	state->transmission_buffer_transmit_pos = (state->transmission_buffer_transmit_pos + 1) % SERIAL_IO_BUFFERS_SIZE;
	
	return true;
}

/** Return true if c is a digit */
bool is_digit(char c)
{
	if (c >= '0' && c <= '9')
		return true;
	else
		return false;
}

//-------------------
// Exported functions
//-------------------

/**
	Initialize a serial input/output stream.
	
	This functions initializes the stream itself and open the serial port at 8 bits, 1 stop bit, no parity.
	
	\param	state
			serial input/output stream
	\param	uart_id
			identifier of the UART to use, may be \ref UART_1 or \ref UART_2
	\param	baud_rate
			baud rate in bps
	\param	hardware_flow_control
			wether hardware flow control (CTS/RTS) should be used or not
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 7 (highest priority)
*/
void serial_io_init(Serial_IO_State* state, int uart_id, unsigned long baud_rate, bool hardware_flow_control, int priority)
{
	// init descriptor struct
	state->uart_id = uart_id;
	state->reception_buffer_reception_pos = 0;
	state->reception_buffer_read_pos = 0;
	state->transmission_buffer_transmit_pos = 0;
	state->transmission_buffer_write_pos = 0;
	
	// init UART and pass state as the user data
	uart_init(uart_id, baud_rate, hardware_flow_control, serial_io_byte_received, serial_io_byte_transmitted, priority, state);
}

/**
	Return wether there is data available for reading in the reception buffer.
	
	\param	state
			serial input/output stream
	\return	true if there is data available for reading, false otherwise
*/
bool serial_io_is_data(Serial_IO_State* state)
{
	return state->reception_buffer_read_pos != state->reception_buffer_reception_pos;
}

/**
	Return the heading character in the reception buffer, but do not remove it from the buffer.
	
	Wait if until there is one character available.
	
	\param	state
			serial input/output stream
	\return	the heading character of the reception buffer
*/
char serial_io_peek_char(Serial_IO_State* state)
{
	// wait while software buffer is empty
	while (!serial_io_is_data(state))
		Idle();
	
	// read data from software buffer
	return state->reception_buffer[state->reception_buffer_read_pos];
}

/**
	Return the heading character in the reception buffer, and remove it from the buffer.
	
	Wait if until there is one character available.
	
	\param	state
			serial input/output stream
	\return	the heading character of the reception buffer
*/
char serial_io_get_char(Serial_IO_State* state)
{
	char c = serial_io_peek_char(state);
	
	state->reception_buffer_read_pos = (state->reception_buffer_read_pos + 1) % SERIAL_IO_BUFFERS_SIZE;
	
	// unblock if previously blocked
	uart_read_pending_data(state->uart_id);
	
	return c;
}

/**
	Return the head of the reception buffer as an unsigned, and remove it from the buffer.
	
	Wait if until there is one character available.
	If the head is not an unsigned, return 0 and do not remove anything.
	
	\param	state
			serial input/output stream
	\return	the head of the reception buffer as an unsigned.
*/
unsigned serial_io_get_unsigned(Serial_IO_State* state)
{
	unsigned val = 0;
	while (is_digit(serial_io_peek_char(state)))
	{
		val *= 10;
		val += (unsigned)(serial_io_get_char(state) - '0');
	}
	return val;
}

static bool is_hex(char c) {
	if (c >= '0' && c <= '9')
		return true;
	if (c >= 'a' && c <= 'f')
		return true;
	if (c >= 'A' && c <= 'F')
		return true;
	return false;
}

static unsigned int c_to_hex(char c) {
	if(c >= '0' && c <= '9') 
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return 10 + c - 'a';

	return 10 + c - 'A';
}

/**
	Return the head of the reception buffer as an int, and remove it from the buffer.
	Parsed as hexadecimal number
	
	Wait if until there is one character available.
	If the head is not an int, return 0 and do not remove anything.
	
	\param	state
			serial input/output stream
	\return	the head of the reception buffer as an int.
*/
unsigned serial_io_get_hex(Serial_IO_State* state)
{
	unsigned val = 0;
	while (is_hex(serial_io_peek_char(state)))
	{
		val *= 16;
		val += c_to_hex(serial_io_get_char(state));
	}
	return val;
}

/**
	Return the head of the reception buffer as an int, and remove it from the buffer.
	Parsed as decimal number
	
	Wait if until there is one character available.
	If the head is not an int, return 0 and do not remove anything.
	
	\param	state
			serial input/output stream
	\return	the head of the reception buffer as an int.
*/
int serial_io_get_int(Serial_IO_State* state)
{
	if (serial_io_peek_char(state) == '-')
	{
		serial_io_get_char(state);
		return -(int)serial_io_get_unsigned(state);
	}
	else
	{
		return (int)serial_io_get_unsigned(state);
	}
}

/**
	Read a specific amount of byte from the reception buffer.
	
	Wait if until all data is available.
	length is allowed to be bigger than the reception buffer, as each byte is read directly when it is available.
	
	\param	state
			serial input/output stream
	\param	buffer
			pointer to location to store data to
	\param	length
			number of bytes to read
*/
void serial_io_get_buffer(Serial_IO_State* state, char* buffer, unsigned length)
{
	unsigned pos = 0;
	while (pos < length)
		buffer[pos++] = serial_io_get_char(state);
}

/**
	Queue a character to the transmission buffer.
	
	If the buffer is full, waits until there is room for the character.
	
	\param	state
			serial input/output stream
	\param	c
			character to send
*/
void serial_io_send_char(Serial_IO_State* state, char c)
{
	// if we were able to send directly, return
	if ((state->transmission_buffer_write_pos == state->transmission_buffer_transmit_pos) && (uart_transmit_byte(state->uart_id, c)))
		return;
	
	// wait while software buffer is full
	while ((state->transmission_buffer_write_pos + 1) % SERIAL_IO_BUFFERS_SIZE == state->transmission_buffer_transmit_pos)
		Idle();
	
	// write data to software buffer
	// NOTE: race condition will NOT happen, because hardware buffer is 4 bytes and the following instruction execute faster that the transmission
	// of 4 bytes. Thus, we are sure that if we use the software buffer, the interrupt will at be called after the end of this function, so data
	// will be transmitted correctly.
	state->transmission_buffer[state->transmission_buffer_write_pos] = c;
	
	state->transmission_buffer_write_pos = (state->transmission_buffer_write_pos + 1) % SERIAL_IO_BUFFERS_SIZE;
}

/**
	Queue a string to the transmission buffer.
	
	If the buffer is full, waits until there is room for the string.
	The string is allowed to be bigger than the transmission buffer, as this function waits while the buffer is full.
	
	\param	state
			serial input/output stream
	\param	string
			null terminated string to send
*/
void serial_io_send_string(Serial_IO_State* state, const char* string)
{
	while (*string)
		serial_io_send_char(state, *string++);
}

/**
	Queue a specific amount of byte to the transmission buffer.
	
	If the buffer is full, waits until there is room all the data.
	length is allowed to be bigger than the transmission buffer, as this function waits while the buffer is full.
	
	\param	state
			serial input/output stream
	\param	buffer
			pointer to location to read data from
	\param	length
			number of bytes to send
*/
void serial_io_send_buffer(Serial_IO_State* state, const char* buffer, unsigned length)
{
	unsigned pos = 0;
	while (pos < length)
		serial_io_send_char(state, buffer[pos++]);
}

/**
	Queue an unsigned to the transmission buffer.
	
	If the buffer is full, waits until there is room for the unsigned.
	
	\param	state
			serial input/output stream
	\param	value
			unsigned to send
	\param	alignment
			One of \ref serial_io_print_alignment : if other than \ref SERIAL_IO_ALIGN_COMPACT, extend small numbers with empty space so that numbers always are of constant width.
*/
void serial_io_send_unsigned(Serial_IO_State* state, unsigned value, int alignment)
{
	unsigned div;
	unsigned padding = 0;
	bool hasSent = false;
	for (div = 10000; div > 0; div /= 10)
	{
		unsigned disp = value / div;
		value %= div;

		if ((disp != 0) || (hasSent) || (div == 1))
		{
			hasSent = true;
			serial_io_send_char(state, '0' + disp);
		}
		else if (alignment == SERIAL_IO_ALIGN_RIGHT)
		{
			serial_io_send_char(state, ' ');
		}
		else if (alignment == SERIAL_IO_ALIGN_LEFT)
		{
			padding++;
		}
	}
	
	while (padding--)
		serial_io_send_char(state, ' ');
}

void serial_io_send_hex(Serial_IO_State* state, unsigned int value, int alignment) {
	int shift;
	unsigned padding = 0;
	bool hasSent = false;
	for (shift = 12; shift >= 0; shift -= 4)
	{
		unsigned disp = ((value & (0xF << shift)) >> shift);

		if ((disp != 0) || (hasSent) || (shift == 0))
		{
			hasSent = true;
			if(disp < 0xA)
				serial_io_send_char(state, '0' + disp);
			else
				serial_io_send_char(state, 'A' + disp - 0xA);
		}
		else if (alignment == SERIAL_IO_ALIGN_RIGHT)
		{
			serial_io_send_char(state, ' ');
		}
		else if (alignment == SERIAL_IO_ALIGN_LEFT)
		{
			padding++;
		}
	}
	
	while (padding--)
		serial_io_send_char(state, ' ');

}

/**
	Queue an integer to the transmission buffer.
	
	If the buffer is full, waits until there is room for the integer.
	
	\param	state
			serial input/output stream
	\param	value
			integer to send
	\param	alignment
			One of \ref serial_io_print_alignment : if other than \ref SERIAL_IO_ALIGN_COMPACT, extend small numbers with empty space so that numbers always are of constant width.
*/
void serial_io_send_int(Serial_IO_State* state, int value, int alignment)
{
	if (value < 0)
	{
		serial_io_send_char(state, '-');
		value = -value;
	}
	else if (alignment != SERIAL_IO_ALIGN_COMPACT)
	{
		serial_io_send_char(state, ' ');
	}
	serial_io_send_unsigned(state, (unsigned)value, alignment);
}

/**
	Clear the screen of an ANSI terminal.
	
	\param	state
			serial input/output stream
*/
void serial_io_clear_screen(Serial_IO_State* state)
{
	serial_io_send_string(state, "\x1B[2J");
}

/**
	Clear the current line of an ANSI terminal.
	
	\param	state
			serial input/output stream
*/
void serial_io_clear_line(Serial_IO_State* state)
{
	serial_io_send_string(state, "\x1B[K");
}

/**
	Move insertion cursor of an ANSI terminal.
	
	This changes the position new characters are inserted.
	
	\param	state
			serial input/output stream
	\param	row
			row, starting from 1 at top
	\param	col
			column, starting from 1 at left
*/
void serial_io_move_cursor(Serial_IO_State* state, unsigned row, unsigned col)
{
	serial_io_send_string(state, "\x1B[");
	serial_io_send_unsigned(state, row, SERIAL_IO_ALIGN_COMPACT);
	serial_io_send_char(state, ';');
	serial_io_send_unsigned(state, col, SERIAL_IO_ALIGN_COMPACT);
	serial_io_send_char(state, 'H');
}

/*@}*/
