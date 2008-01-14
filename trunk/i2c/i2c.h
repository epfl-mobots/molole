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

#ifndef _MOLOLE_I2C_H
#define _MOLOLE_I2C_H

#include "../types/types.h"

/** \addtogroup i2c */
/*@{*/

/** \file
	I2C slave and master wrappers definitions
*/

/** I2C callback for status change */
typedef void(*i2c_status_callback)();

/** I2C callback when a data is available.
	Return true if end of message, false otherwise. */
typedef bool (*i2c_set_data_callback)(unsigned char data);

/** I2C callback when a data must be returned.
	Return true if end of message, false otherwise. */
typedef bool (*i2c_get_data_callback)(unsigned char* data);

// Functions, doc in the .c

void i2c_init(unsigned char address);

void i2c_init_slave(
	i2c_status_callback message_from_master_callback,
	i2c_status_callback message_to_master_callback,
	i2c_set_data_callback data_from_master_callback,
	i2c_get_data_callback data_to_master_callback,
	int priority
);

void i2c_slave_return_to_idle(void);

void i2c_init_master(int priority);

/*@}*/

#endif
