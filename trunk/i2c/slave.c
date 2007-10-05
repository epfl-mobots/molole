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


//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "i2c.h"
#include "../error/error.h"

//-----------------------
// Structures definitions
//-----------------------

/** I2C wrapper data */
static struct
{
	i2c_status_callback message_from_master_callback; /**< function to call upon new write message */
	i2c_status_callback message_to_master_callback; /**< function to call upon new read message */
	i2c_set_data_callback data_from_master_callback; /**< function to call with data from master */
	i2c_get_data_callback data_to_master_callback; /**< function to call with data to master */
} IC2_Data = { 0, 0, 0, 0 };

void i2c_init_slave(
	i2c_status_callback message_from_master_callback,
	i2c_status_callback message_to_master_callback,
	i2c_set_data_callback data_from_master_callback,
	i2c_get_data_callback data_to_master_callback,
	int priority
)
{
	// Store callback functions
	IC2_Data.message_from_master_callback = message_from_master_callback;
	IC2_Data.message_to_master_callback = message_to_master_callback;
	IC2_Data.data_from_master_callback = data_from_master_callback;
	IC2_Data.data_to_master_callback = data_to_master_callback;
	
	_SI2C1IF = 0;				// clear the slave interrupt
	_SI2C1IP = priority;   		// set the slave interrupt priority
	_SI2C1IE = 1;				// enable the slave interruptt
}

// Interrupt service routine
void _ISR _SI2C1Interrupt(void)
{
	if (I2C1STATbits.D_A)
	{
		// last transfer was data
		if (I2C1STATbits.R_W)
		{
			// to master cycle, check stat bit
			if (!I2C1STATbits.ACKSTAT)
			{
				I2C1TRN = IC2_Data.data_to_master_callback(); // Write data
				I2C1CONbits.SCLREL = 1;				// Release clock
			}
		}
		else
		{
			// from master cycle, check if data is present in the buffer
			if (I2C1STATbits.RBF)
			{
				unsigned char data = I2C1RCV;		// Read data
				I2C1CONbits.SCLREL = 1;				// Release clock
				IC2_Data.data_from_master_callback(data);
			}
		}
	}
	else
	{
		// last transfer was address
		if (I2C1STATbits.R_W)
		{
			// to master cycle
			IC2_Data.message_to_master_callback();
			I2C1TRN = IC2_Data.data_to_master_callback(); // Write data
			I2C1CONbits.SCLREL = 1;				// Release clock
		}
		else
		{
			// from master cycle
			IC2_Data.message_from_master_callback();
		}
	}
	
	_SI2C1IF = 0;				// Clear Slave interrupt flag
}