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

#ifndef _MOLOLE_I2C_H
#define _MOLOLE_I2C_H

#include "../types/types.h"
#include "../error/error.h"

/** Errors I2C can throw */
enum i2c_errors
{
	I2C_ERROR_BASE = 0x0A00,
	I2C_ERROR_INVALID_ID,				/**< The specified I2C does not exists. */
	I2C_INVALID_CLOCK,					/**< The specified Clock is not available. */
	I2C_ERROR_MASTER_BUSY,				/**< A master start was called while the master was busy. */
	I2C_INVALID_OPERATION,				/**< The operation asked is invalid */
	I2C_PROTOCOL_INTERNAL_ERROR,		/**< The state machine is confused. Please report a such bug it should never happens */
};

/** \addtogroup i2c */
/*@{*/

/** \file
	I2C slave and master wrappers definitions
*/

// Defines

/** Identifiers of available I2Cs. */
enum i2c_identifiers
{
	I2C_1 = 0,			/**< first I2C */
	I2C_2,				/**< second I2C */
	I2C_3				/**< third I2C */
};

/** I2C callback for status change */
typedef void (*i2c_status_callback)(int i2c_id);

/** I2C callback when a data is available.
	Return true if end of message, false otherwise. */
typedef bool (*i2c_set_data_callback)(int i2c_id, unsigned char data);

/** I2C callback when a data must be returned.
	Return true if end of message, false otherwise. */
typedef bool (*i2c_get_data_callback)(int i2c_id, unsigned char* data);


/** I2C callback when an error condition is detected on the bus */
typedef void (*i2c_error_callback)(int i2c_id, int error_type);

// Functions, doc in the .c

void i2c_init(int i2c_id);

void i2c_init_slave(
	int i2c_id,
	unsigned char address,
	i2c_status_callback message_from_master_callback,
	i2c_status_callback message_to_master_callback,
	i2c_set_data_callback data_from_master_callback,
	i2c_get_data_callback data_to_master_callback,
	int priority
);

void i2c_disable_slave(int i2c_id);

void i2c_slave_return_to_idle(int i2c_id);

/** I2C master operations the protocol layer can do */
enum i2c_master_operation
{
	I2C_MASTER_NONE,		/** No operation, used when no previous operation */
	I2C_MASTER_READ,	/** Read byte operation */
	I2C_MASTER_WRITE,	/** Write byte operation */
	I2C_MASTER_RESTART,	/** Restart bit operation */
	I2C_MASTER_ACK,		/** Ack bit operation */
	I2C_MASTER_NACK,	/** NAck bit operation */
	I2C_MASTER_STOP,	/** Stop bit operation */
	I2C_MASTER_DONE,	/** No more operation, but reset the state machine */
	I2C_MASTER_QUIT		/** No more operation, do _NOT_ reset the state machine, you have to call \ref i2c_master_reset manually */
};

/** I2C callback when a transfert (read/write) is completed.
	Return next action to do, \ref I2C_MASTER_DONE if no more action must be done.
	- If this function returns \ref I2C_MASTER_READ, data points to the destination of read
	- If this function returns \ref I2C_MASTER_WRITER, data points to the data to write
*/
typedef int (*i2c_master_operation_completed_callback)(int i2c_id, unsigned char** data, void* user_data, bool nack);

// Functions, doc in the .c

void i2c_init_master(int i2c_id, long speed, int priority);

void i2c_master_start_operations(int i2c_id, i2c_master_operation_completed_callback operation_completed_callback, void* user_data);

void i2c_master_reset(int i2c_id);

bool i2c_master_is_busy(int i2c_id);

// high level helpers for I2C master

/** Results of I2C high level protocol operations */
enum i2c_protocol_results
{
	I2C_OPERATION_FINISHED = 0,			/**< The operation (read,write) finished successfully. */
	I2C_ERROR_GOT_NACK_TO_ADDRESS,		/**< The bus got an NACK after the Addressing phase. */
	I2C_ERROR_GOT_NACK_AFTER_REGISTER,	/**< The bus got an NACK after sending the register byte. */
	I2C_ERROR_GOT_NACK_AFTER_DATA,		/**< The bus got an NACK after sending data. */
	I2C_INTERNAL_ERROR,					/**< A internal error has occured, either this is memory corruption or this is a driver bug. */
};

/** I2C callback when async master transfert has finished operations, result is true if successfull, false otherwise */
typedef void (*i2c_master_transfert_result_callback)(int i2c_id, bool result);

// Functions, doc in the .c

void i2c_master_transfert_async(int i2c_id, unsigned char addr, unsigned char* write_data, unsigned write_count, unsigned char* read_data, unsigned read_count, i2c_master_transfert_result_callback result_callback);

int i2c_master_transfert_block(int i2c_id, unsigned char addr, unsigned char* write_data, unsigned write_count, unsigned char* read_data, unsigned read_count);

bool __attribute((deprecated)) i2c_read(int i2c_id, unsigned char device_add, unsigned char reg, unsigned char *data, unsigned int size);
bool __attribute((deprecated)) i2c_write(int i2c_id, unsigned char device_add, unsigned char reg, unsigned char *data, unsigned int size);

/*@}*/

#endif
