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
	\defgroup serial-io Serial input/output library
	
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


bool serial_io_is_data(Serial_IO_State* state)
{
	return state->reception_buffer_read_pos != state->reception_buffer_reception_pos;
}

char serial_io_peek_char(Serial_IO_State* state)
{
	// wait while software buffer is empty
	while (!serial_io_is_data(state))
		Idle();
	
	// read data from software buffer
	return state->reception_buffer[state->reception_buffer_read_pos];
}

char serial_io_get_char(Serial_IO_State* state)
{
	char c = serial_io_peek_char(state);
	
	state->reception_buffer_read_pos = (state->reception_buffer_read_pos + 1) % SERIAL_IO_BUFFERS_SIZE;
	
	// unblock if previously blocked
	uart_read_pending_data(state->uart_id);
	
	return c;
}

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

void serial_io_get_buffer(Serial_IO_State* state, char* buffer, unsigned length)
{
	unsigned pos = 0;
	while (pos < length)
		buffer[pos++] = serial_io_get_char(state);
}


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

void serial_io_send_string(Serial_IO_State* state, const char* string)
{
	while (*string)
		serial_io_send_char(state, *string++);
}

void serial_io_send_buffer(Serial_IO_State* state, const char* buffer, unsigned length)
{
	unsigned pos = 0;
	while (pos < length)
		serial_io_send_char(state, buffer[pos++]);
}

void serial_io_send_unsigned(Serial_IO_State* state, unsigned value)
{
	unsigned div;
	bool hasSent = false;
	for (div = 10000; div > 0; div /= 10)
	{
		unsigned disp = value / div;
		value %= div;

		if ((disp != 0) || (hasSent))
		{
			hasSent = true;
			serial_io_send_char(state, '0' + disp);
		}
	}
	if (!hasSent)
		serial_io_send_char(state, '0');
}

void serial_io_send_int(Serial_IO_State* state, int value)
{
	if (value < 0)
	{
		serial_io_send_char(state, '-');
		value = -value;
	}
	serial_io_send_unsigned(state, (unsigned)value);
}

void serial_io_clear_screen(Serial_IO_State* state)
{
	serial_io_send_string(state, "\x1B[2J");
}

void serial_io_clear_line(Serial_IO_State* state)
{
	serial_io_send_string(state, "\x1B[K");
}

// the upper left corner is 1,1
void serial_io_move_cursor(Serial_IO_State* state, unsigned row, unsigned col)
{
	serial_io_send_string(state, "\x1B[");
	serial_io_send_unsigned(state, row);
	serial_io_send_char(state, ';');
	serial_io_send_unsigned(state, col);
	serial_io_send_char(state, 'H');
}

/*@}*/
