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

#ifndef _MOLOLE_SERIAL_IO_H
#define _MOLOLE_SERIAL_IO_H

#include "../types/types.h"
#include "../uart/uart.h"

/** \addtogroup serial-io */
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
	SERIAL_IO_ALIGN_COMPACT = 0,						/**< compact number, do not align */
	SERIAL_IO_ALIGN_LEFT,								/**< align number at left */
	SERIAL_IO_ALIGN_RIGHT,								/**< align number at right */
	SERIAL_IO_ALIGN_FILL								/**< fill number leading with 0 */
};

// Structures definitions

/** Data associated with a serial input/output stream; basically a twin producer/consumer buffer. */
typedef struct
{
	int uart_id;										/**< identifier of the UART the stream is attached to, may be \ref UART_1 or \ref UART_2 */
	
	unsigned reception_buffer_read_pos;					/**< position of reading (from the user code) in the reception buffer */
	unsigned reception_buffer_reception_pos;			/**< position of reception (from the interrupt code) in the reception buffer */
	char reception_buffer[SERIAL_IO_BUFFERS_SIZE];		/**< reception buffer */
	
	unsigned transmission_buffer_write_pos;				/**< position of writing (from the user code) in the transmission buffer */
	unsigned transmission_buffer_transmit_pos;			/**< position of transmission (from the interrupt code) in the transmission buffer */
	char transmission_buffer[SERIAL_IO_BUFFERS_SIZE];	/**< transmission buffer */
} Serial_IO_State;

// Functions, doc in the .c

void serial_io_init(Serial_IO_State* state, int uart_id, unsigned long baud_rate, bool hardware_flow_control, int priority);


bool serial_io_is_data(Serial_IO_State* state);

char serial_io_get_char(Serial_IO_State* state);

unsigned serial_io_get_unsigned(Serial_IO_State* state);

int serial_io_get_int(Serial_IO_State* state);

void serial_io_get_buffer(Serial_IO_State* state, char* buffer, unsigned length);

unsigned serial_io_get_hex(Serial_IO_State* state);


void serial_io_send_char(Serial_IO_State* state, char c);

void serial_io_send_string(Serial_IO_State* state, const char* string);

void serial_io_send_buffer(Serial_IO_State* state, const char* buffer, unsigned length);

void serial_io_send_unsigned(Serial_IO_State* state, unsigned value, int alignment);

void serial_io_send_int(Serial_IO_State* state, int value, int alignment);

void serial_io_send_hex(Serial_IO_State* state, unsigned int value, int alignment);


void serial_io_clear_screen(Serial_IO_State* state);

void serial_io_clear_line(Serial_IO_State* state);

void serial_io_move_cursor(Serial_IO_State* state, unsigned row, unsigned col);

/*@}*/

#endif
