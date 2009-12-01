/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	
	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>,
	Philippe Retornaz <philippe dot retornaz at epfl dot ch>
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

#ifndef _MOLOLE_CAN_H
#define _MOLOLE_CAN_H

#include "../types/types.h"
#include "../gpio/gpio.h"

/** \addtogroup can */
/*@{*/

/** \file
	CAN wrapper definitions
*/


/** Errors CAN module can throw */
enum can_errors
{
	CAN_ERROR_BASE = 0x0A00,
	CAN_UNKNOWN_SPEED,
	CAN_UNKNOWN_CPU_CLOCK,
};
// Defines

/*! the data that physically go on the CAN bus; used to communicate with the CAN data layer */
typedef struct
{
	unsigned char data[8] __attribute__((aligned(sizeof(int)))); /**< data payload */
	unsigned id:11; /**< CAN identifier */
	unsigned len:4; /**< amount of bytes used in data */
} can_frame;


/*! User-specified function to call when a new CAN frame is available. */
typedef void (*can_frame_received_callback)(const can_frame* frame);

/*! User-specified function to call when the CAN frame was sent successfully. */
typedef void (*can_frame_sent_callback)(void);

// Functions, doc in the .c
void can_init(can_frame_received_callback frame_received_callback, can_frame_sent_callback frame_sent_callback, int dma_rx_channel, int dma_tx_channel, gpio trans_pin, unsigned int kbaud_rate, int priority);

bool can_send_frame(const can_frame *frame);

bool can_is_frame_room(void);

void can_enable(void);
void can_disable(void);


#endif
