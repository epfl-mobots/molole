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

//--------------------
// Usage documentation
//--------------------

/** \addtogroup i2c */
/*@{*/

/** \file
	Implementation of the I2C master protocols.
*/

//------------
// Definitions
//------------

#include <p33fxxxx.h>
#include <string.h> //memcpy
#include "i2c.h"
#include "../error/error.h"
#include "../clock/clock.h"

//-----------------------
// Structures definitions
//-----------------------



// TODO: doc
enum I2C_master_transfert_states
{
	I2C_IDLE = 0,
	I2C_START_DONE,
	I2C_ADDRESS_DONE,
	I2C_WRITE_IN_PROGRESS,
	I2C_RESTART_DONE,
	I2C_READ_IN_PROGRESS,
	I2C_ACK_DONE,
	I2C_NACK_DONE,
	I2C_STOP_DONE,
};

// TODO: doc
typedef struct {
	int state;
	bool result;	/** true if last transfert was successful, false otherwise; false during transfert */
	unsigned char address; /** address to write on the bus */
	unsigned char * write_data; /** ptr to the data waiting to be written on the bus */
	unsigned int write_data_size; /** byte still to be written on the bus */
	unsigned char * read_data; /** ptr where to put data to read */
	unsigned int read_data_size; /** byte still to be read on the bus */
	i2c_master_transfert_result_callback result_callback;

} I2C_Master_Protocol_Data;

/** data for the master I2C 1 and 2 high level protocol */
static I2C_Master_Protocol_Data I2C_master_transfert_datas[2];

//-------------------
// Internal callbacks
//-------------------

/** callback from low-level I2C layer on I2C interrupt */
static int i2c_master_transfert_op(int i2c_id, unsigned char** data, void* user_data, bool nack)
{
	int ret = I2C_MASTER_NONE;
	switch (I2C_master_transfert_datas[i2c_id].state)
	{
		case I2C_START_DONE:
			ret = I2C_MASTER_WRITE;

			if(I2C_master_transfert_datas[i2c_id].write_data_size) 
				I2C_master_transfert_datas[i2c_id].address &= 0xFFFE;
			else
				I2C_master_transfert_datas[i2c_id].address |= 0x1;

			*data = &(I2C_master_transfert_datas[i2c_id].address);
			I2C_master_transfert_datas[i2c_id].state = I2C_ADDRESS_DONE;
			
		break;

		case I2C_ADDRESS_DONE:
			if(nack) {
				ret = I2C_MASTER_STOP;
				I2C_master_transfert_datas[i2c_id].result = false;
				I2C_master_transfert_datas[i2c_id].state = I2C_STOP_DONE;
				break;
			}
			if(I2C_master_transfert_datas[i2c_id].write_data_size) {
				*data = I2C_master_transfert_datas[i2c_id].write_data++;
				I2C_master_transfert_datas[i2c_id].write_data_size--;
				ret = I2C_MASTER_WRITE;
				I2C_master_transfert_datas[i2c_id].state = I2C_WRITE_IN_PROGRESS;
			} else if(I2C_master_transfert_datas[i2c_id].read_data_size) {
				*data = I2C_master_transfert_datas[i2c_id].read_data++;
				I2C_master_transfert_datas[i2c_id].read_data_size--;
				ret = I2C_MASTER_READ;
				I2C_master_transfert_datas[i2c_id].state = I2C_READ_IN_PROGRESS;
			} else {
				ret = I2C_MASTER_STOP;
				I2C_master_transfert_datas[i2c_id].result = true;
				I2C_master_transfert_datas[i2c_id].state = I2C_STOP_DONE;
			}
			break;

		case I2C_WRITE_IN_PROGRESS:
			if(nack) {
				ret = I2C_MASTER_STOP;
				I2C_master_transfert_datas[i2c_id].result = false;
				I2C_master_transfert_datas[i2c_id].state = I2C_STOP_DONE;
				break;
			}
			if(I2C_master_transfert_datas[i2c_id].write_data_size) {
				*data = I2C_master_transfert_datas[i2c_id].write_data++;
				I2C_master_transfert_datas[i2c_id].write_data_size--;
				ret = I2C_MASTER_WRITE;
				I2C_master_transfert_datas[i2c_id].state = I2C_WRITE_IN_PROGRESS;
			} else if(I2C_master_transfert_datas[i2c_id].read_data_size) {
				ret = I2C_MASTER_RESTART;
				I2C_master_transfert_datas[i2c_id].state = I2C_START_DONE;
			} else {
				ret = I2C_MASTER_STOP;
				I2C_master_transfert_datas[i2c_id].result = true;
				I2C_master_transfert_datas[i2c_id].state = I2C_STOP_DONE;
			}
			break;
	
		case I2C_READ_IN_PROGRESS:
			if(I2C_master_transfert_datas[i2c_id].read_data_size) {
				ret = I2C_MASTER_ACK;
				I2C_master_transfert_datas[i2c_id].state = I2C_ACK_DONE;
			} else {
				ret = I2C_MASTER_NACK;
				I2C_master_transfert_datas[i2c_id].state = I2C_NACK_DONE;
				I2C_master_transfert_datas[i2c_id].result = true;
			}
			break;

		case I2C_NACK_DONE:
			/* Nack is only generated when we stop reading so we can directly generate a stop */
			ret = I2C_MASTER_STOP;
			I2C_master_transfert_datas[i2c_id].state = I2C_STOP_DONE;
			break;
		case I2C_ACK_DONE:
			/* We generate an ack only when we want more data, so we can directly reschedule a transfert */
			*data = I2C_master_transfert_datas[i2c_id].read_data++;
			I2C_master_transfert_datas[i2c_id].read_data_size--;
			I2C_master_transfert_datas[i2c_id].state = I2C_READ_IN_PROGRESS;
			ret = I2C_MASTER_READ;
			break;

		case I2C_STOP_DONE:
			ret = I2C_MASTER_DONE;
			I2C_master_transfert_datas[i2c_id].state = I2C_IDLE;
			if(I2C_master_transfert_datas[i2c_id].result_callback)
				I2C_master_transfert_datas[i2c_id].result_callback(i2c_id, I2C_master_transfert_datas[i2c_id].result);
			break;

		case I2C_IDLE:
		default:
			ERROR(I2C_PROTOCOL_INTERNAL_ERROR, &I2C_master_transfert_datas[i2c_id].state);
		break;
	}

	return ret;
}


//-------------------
// Exported functions
//-------------------

/**
	Start an asynchronous I2C master transfert, consisting of a combined write/read cycle.

	Both write_count and read_count may be zero, for write/read only cycle.

	\param	i2c_id
			identifier of the I2C, \ref I2C_1 or \ref I2C_2
	\param	addr
			I2C address (7 bits, unshifted)
	\param	write_data
			pointer to data to write to device
	\param	write_count
			amount of data to write to device
	\param	read_data
			pointer where data read from the device will be written
	\param	read_count
			amount of data to read from device
	\param	result_callback
			user-defined function to call when transfert is completed or aborted because of an errors
*/
void i2c_master_transfert_async(int i2c_id, unsigned char addr, unsigned char* write_data, unsigned write_count, unsigned char* read_data, unsigned read_count, i2c_master_transfert_result_callback result_callback)
{
	ERROR_CHECK_RANGE(i2c_id, I2C_1, I2C_2, I2C_ERROR_INVALID_ID);
	if (I2C_master_transfert_datas[i2c_id].state != I2C_IDLE)
		ERROR(I2C_ERROR_MASTER_BUSY, &(I2C_master_transfert_datas[i2c_id].state));
 
	I2C_master_transfert_datas[i2c_id].result = true;
	I2C_master_transfert_datas[i2c_id].result_callback = result_callback;
	I2C_master_transfert_datas[i2c_id].state = I2C_START_DONE;
	I2C_master_transfert_datas[i2c_id].address = addr << 1;
	I2C_master_transfert_datas[i2c_id].write_data = write_data;
	I2C_master_transfert_datas[i2c_id].write_data_size = write_count;
	I2C_master_transfert_datas[i2c_id].read_data = read_data;
	I2C_master_transfert_datas[i2c_id].read_data_size = read_count;
	i2c_master_start_operations(i2c_id, i2c_master_transfert_op, NULL);
}

/**
	Start a blocking I2C master transfert, consisting of a combined write/read cycle.

	Both write_count and read_count may be zero, for write/read only cycle.

	\param	i2c_id
			identifier of the I2C, \ref I2C_1 or \ref I2C_2
	\param	addr
			I2C address (7 bits, unshifted)
	\param	write_data
			pointer to data to write to device
	\param	write_count
			amount of data to write to device
	\param	read_data
			pointer where data read from the device will be written
	\param	read_count
			amount of data to read from device

	\return	true if transfert was successful, false otherwise
*/
bool i2c_master_transfert_block(int i2c_id, unsigned char addr, unsigned char* write_data, unsigned write_count, unsigned char* read_data, unsigned read_count)
{
	i2c_master_transfert_async(i2c_id, addr, write_data, write_count, read_data, read_count, NULL);
	while (i2c_master_is_busy(i2c_id))	barrier(); // Cannot use Idle since we may miss the interrupt
	return I2C_master_transfert_datas[i2c_id].result;
}

/**
	Read data from a device using the register/value protocol.

	\param	i2c_id
			identifier of the I2C, \ref I2C_1 or \ref I2C_2
	\param	device_add
			I2C address (7 bits, unshifted)
	\param	reg
			register inside the device
	\param	data
			pointer to read data from
	\param	size
			amount of data to read

	\return	true if transfert was successful, false otherwise
*/
bool i2c_read(int i2c_id, unsigned char device_add, unsigned char reg, unsigned char *data, unsigned int size)
{
	return i2c_master_transfert_block(i2c_id, device_add, &reg, 1, data, size);
}

/**
	Write data to a device using the register/value protocol.

	\param	i2c_id
			identifier of the I2C, \ref I2C_1 or \ref I2C_2
	\param	device_add
			I2C address (7 bits, unshifted)
	\param	reg
			register inside the device
	\param	data
			pointer where to write data
	\param	size
			amount of data to write

	\return	true if transfert was successful, false otherwise
*/
bool i2c_write(int i2c_id, unsigned char device_add, unsigned char reg, unsigned char *data, unsigned int size)
{
	#define MIN_STACK_LEFT	64
	if (size + 1 + MIN_STACK_LEFT < get_stack_space())
	{
		unsigned char temp_data[size + 1];
		temp_data[0] = reg;
		memcpy(temp_data+1, data, size);
		return i2c_master_transfert_block(i2c_id, device_add, temp_data, size+1, 0, 0);
	}
	ERROR(GENERIC_ERROR_STACK_SPACE_EXHAUSTED, (void *) (-(get_stack_space() - MIN_STACK_LEFT - size - 1)));
}

/*@}*/
