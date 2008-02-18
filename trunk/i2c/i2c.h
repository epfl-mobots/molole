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

/** Errors I2C can throw */
enum i2c_errors
{
	I2C_ERROR_BASE = 0x0A00,
	I2C_INVALID_CLOCK,					/**< The specified Clock is not available */
	I2C_INTERNAL_ERROR,					/**< A internal error has occured, either this is memory corruption or this is a driver bug */
	I2C_ERROR_GOT_NACK_TO_ADDRESS,		/**< The bus got an NACK after the Addressing phase */
	I2C_ERROR_GOT_NACK_AFTER_REGISTER,	/**< The bus got an NACK after sending the register byte */
	I2C_ERROR_GOT_NACK_AFTER_DATA,		/**< The bus got an NACK after sending data */
	I2C_OPERATION_FINISHED,				/**< The operation (read,write) finished successfully */
};

/** \addtogroup i2c */
/*@{*/

/** \file
	I2C slave and master wrappers definitions
*/

// Defines

/** I2C callback for status change */
typedef void(*i2c_status_callback)();

/** I2C callback when a data is available.
	Return true if end of message, false otherwise. */
typedef bool (*i2c_set_data_callback)(unsigned char data);

/** I2C callback when a data must be returned.
	Return true if end of message, false otherwise. */
typedef bool (*i2c_get_data_callback)(unsigned char* data);


/** I2C callback when an error condition is detected on the bus */
typedef void (*i2c_error_callback)(int error_type);

// Functions, doc in the .c

void i2c_init(void);

void i2c_init_slave(
	unsigned char address,
	i2c_status_callback message_from_master_callback,
	i2c_status_callback message_to_master_callback,
	i2c_set_data_callback data_from_master_callback,
	i2c_get_data_callback data_to_master_callback,
	int priority
);

void i2c_slave_return_to_idle(void);

void i2c_init_master(long speed, int priority);
int i2c_master_start_send(i2c_error_callback error_callback, unsigned char mod_addr, unsigned char mod_reg, unsigned char *data, unsigned int size);
int i2c_master_start_read(i2c_error_callback error_callback, unsigned char mod_addr, unsigned char mod_reg, unsigned char * data, unsigned int size);
int i2c_master_is_busy(void);


bool i2c_read(i2c_error_callback error_callback, unsigned char device_add, unsigned char reg, unsigned char *data, unsigned int size);
bool i2c_write(i2c_error_callback error_callback, unsigned char device_add, unsigned char reg, unsigned char *data, unsigned int size);

/*@}*/

#endif
