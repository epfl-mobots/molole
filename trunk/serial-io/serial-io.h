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

#ifndef _MOLOLE_SERIAL_IO_H
#define _MOLOLE_SERIAL_IO_H

#include "../types/types.h"
#include "../uart/uart.h"

/** \addtogroup serialio */
/*@{*/

/** \file
	\brief The interface of a serial input/output library
*/

// Defines

/** Sizes of read and write buffers */
#define SERIAL_IO_BUFFERS_SIZE 64

/** Possible alignment when sending numbers */
enum serial_io_print_alignment
{
	SERIAL_IO_ALIGN_COMPACT = 0,	/**< compact number, do not align */
	SERIAL_IO_ALIGN_LEFT,			/**< align number at left */
	SERIAL_IO_ALIGN_RIGHT			/**< align number at right */
};

// Structures definitions

typedef struct
{
	int uart_id;
	
	unsigned reception_buffer_read_pos;
	unsigned reception_buffer_reception_pos;
	char reception_buffer[SERIAL_IO_BUFFERS_SIZE];
	
	unsigned transmission_buffer_write_pos;
	unsigned transmission_buffer_transmit_pos;
	char transmission_buffer[SERIAL_IO_BUFFERS_SIZE];
} Serial_IO_State;

// Functions, doc in the .c

void serial_io_init(Serial_IO_State* state, int uart_id, unsigned long baud_rate, bool hardware_flow_control, int priority);


bool serial_io_is_data(Serial_IO_State* state);

char serial_io_get_char(Serial_IO_State* state);

unsigned serial_io_get_unsigned(Serial_IO_State* state);

int serial_io_get_int(Serial_IO_State* state);

void serial_io_get_buffer(Serial_IO_State* state, char* buffer, unsigned length);


void serial_io_send_char(Serial_IO_State* state, char c);

void serial_io_send_string(Serial_IO_State* state, const char* string);

void serial_io_send_buffer(Serial_IO_State* state, const char* buffer, unsigned length);

void serial_io_send_unsigned(Serial_IO_State* state, unsigned value, int alignment);

void serial_io_send_int(Serial_IO_State* state, int value, int alignment);


void serial_io_clear_screen(Serial_IO_State* state);

void serial_io_clear_line(Serial_IO_State* state);

void serial_io_move_cursor(Serial_IO_State* state, unsigned row, unsigned col);

/*@}*/

#endif
