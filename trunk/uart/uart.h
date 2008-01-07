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

#ifndef _MOLOLE_UART_H
#define _MOLOLE_UART_H

#include "../types/types.h"

/** \addtogroup uart */
/*@{*/

/** \file
	UART wrapper definitions
*/

/** UART callback when a byte is received */
typedef void (*uart_byte_received)(int uart_id, unsigned char data);

/** UART callback when a byte has been transmitted
	Return true if a new one should be sent, false otherwise */
typedef bool (*uart_byte_transmitted)(int uart_id, unsigned char* data);

// Functions, doc in the .c

void uart_1_init(
	unsigned long baud_rate,
	bool hardware_flow_control,
	uart_byte_received byte_received_callback,
	uart_byte_transmitted byte_transmitted_callback,
	int priority
);

bool uart_1_transmit_byte(unsigned char data);

void uart_2_init(
	unsigned long baud_rate,
	bool hardware_flow_control,
	uart_byte_received byte_received_callback,
	uart_byte_transmitted byte_transmitted_callback,
	int priority
);

bool uart_2_transmit_byte(unsigned char data);


/*@}*/

#endif
