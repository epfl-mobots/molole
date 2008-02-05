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

/** \addtogroup can */
/*@{*/

/** \file
	CAN wrapper definitions
*/

// Defines

/*! the data that physically go on the CAN bus; used to communicate with the CAN data layer */
typedef struct
{
	uint8 data[8]; /**< data payload */
	unsigned id:11; /**< CAN identifier */
	unsigned used:1; /**< when frame is in a circular buffer, tell if it frame is used */
	unsigned len:4; /**< amount of bytes used in data */
} can_frame;

/*! User-specified function to call when a new CAN frame is available. */
typedef void (*can_frame_received_callback)(const can_frame* frame);

/*! User-specified function to call when the CAN frame was sent successfully. */
typedef void (*can_frame_sent_callback());


// Functions, doc in the .c

void can_init(can_frame_received_callback frame_received_callback, can_frame_sent_callback frame_sent_callback, int priority);

void can_send_frame(const can_frame *frame);

int can_is_frame_room();


// Defines