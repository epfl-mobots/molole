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
	Implementation of the wrapper around I2C slave.
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

/** State of the transmission */
enum I2C_State
{
	I2C_IDLE,
	I2C_TO_MASTER,
	I2C_FROM_MASTER,
	I2C_END_TO_MASTER
};	

/** I2C slave wrapper data */
typedef struct
{
	i2c_status_callback message_from_master_callback; /**< function to call upon new write message */
	i2c_status_callback message_to_master_callback; /**< function to call upon new read message */
	i2c_set_data_callback data_from_master_callback; /**< function to call with data from master */
	i2c_get_data_callback data_to_master_callback; /**< function to call with data to master */
	int state; /**< transmission direction */
} I2C_Slave_Data;

/** data for the slave I2C 1 wrapper */
static I2C_Slave_Data I2C_1_Slave_Data;

/** data for the slave I2C 2 wrapper */
static I2C_Slave_Data I2C_2_Slave_Data;

//-------------------
// Exported functions
//-------------------

/**
	Init I2C slave subsystem.
	
	\param	i2c_id
			identifier of the I2C, \ref I2C_1 or \ref I2C_2
	\param  address 
			Slave address
	\param	message_from_master_callback
			function to call upon new write message
	\param	message_to_master_callback
			function to call upon new read message
	\param	data_from_master_callback
			function to call with data from master
	\param	data_to_master_callback
			function to call with data to master
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 6 (highest normal priority)
*/
void i2c_init_slave(
	int i2c_id,
	unsigned char address,
	i2c_status_callback message_from_master_callback,
	i2c_status_callback message_to_master_callback,
	i2c_set_data_callback data_from_master_callback,
	i2c_get_data_callback data_to_master_callback,
	int priority
)
{
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	
	if (i2c_id == I2C_1)
	{
		// Store callback functions
		I2C_1_Slave_Data.message_from_master_callback = message_from_master_callback;
		I2C_1_Slave_Data.message_to_master_callback = message_to_master_callback;
		I2C_1_Slave_Data.data_from_master_callback = data_from_master_callback;
		I2C_1_Slave_Data.data_to_master_callback = data_to_master_callback;
		
		I2C_1_Slave_Data.state = I2C_IDLE;
		
		I2C1ADD = address;			// Set the module address 
		_SI2C1IF = 0;				// clear the slave interrupt
		_SI2C1IP = priority;   		// set the slave interrupt priority
		_SI2C1IE = 1;				// enable the slave interrupt
	}
	else if (i2c_id == I2C_2)
	{
		// Store callback functions
		I2C_2_Slave_Data.message_from_master_callback = message_from_master_callback;
		I2C_2_Slave_Data.message_to_master_callback = message_to_master_callback;
		I2C_2_Slave_Data.data_from_master_callback = data_from_master_callback;
		I2C_2_Slave_Data.data_to_master_callback = data_to_master_callback;
		
		I2C_2_Slave_Data.state = I2C_IDLE;
		
		I2C2ADD = address;			// Set the module address 
		_SI2C2IF = 0;				// clear the slave interrupt
		_SI2C2IP = priority;   		// set the slave interrupt priority
		_SI2C2IE = 1;				// enable the slave interrupt
	}
	else
	{
		ERROR(I2C_ERROR_INVALID_ID, &i2c_id);
	}
}

/**
	Force I2C internal state machine to return to IDLE,
	This can be useful, for instance after an electric problem
	on the bus.
	
	\param	i2c_id
			identifier of the I2C, \ref I2C_1 or \ref I2C_2
*/
void i2c_slave_return_to_idle(int i2c_id)
{
	if (i2c_id == I2C_1)
		I2C_1_Slave_Data.state = I2C_IDLE;
	else if (i2c_id == I2C_2)
		I2C_2_Slave_Data.state = I2C_IDLE;
	else
		ERROR(I2C_ERROR_INVALID_ID, &i2c_id);
}



//--------------------------
// Interrupt service routine
//--------------------------

/**
	I2C 1 Interrupt Service Routine.
 
	Call the state-specific user-defined function.
*/
void _ISR _SI2C1Interrupt(void)
{
	unsigned char data;
	
	// no interrupt is generated at the end of cycle,
	// nor all way to detect beginning of cycle are buggy
	// and do not behave as the doc predicts
	if (I2C_1_Slave_Data.state == I2C_IDLE)
	{
		if (I2C1STATbits.R_W)
		{
			I2C_1_Slave_Data.state = I2C_TO_MASTER;
			I2C_1_Slave_Data.message_to_master_callback(I2C_1);
		}
		else
		{
			I2C_1_Slave_Data.state = I2C_FROM_MASTER;
			I2C_1_Slave_Data.message_from_master_callback(I2C_1);
			_SI2C1IF = 0;										// Clear Slave interrupt flag
			return;
		}
	}
	
	switch (I2C_1_Slave_Data.state)
	{
		case I2C_TO_MASTER:
		{
			if (I2C_1_Slave_Data.data_to_master_callback(I2C_1, &data))
				I2C_1_Slave_Data.state = I2C_END_TO_MASTER;
			I2C1TRN = data; 									// Write data
			I2C1CONbits.SCLREL = 1;								// Release clock
		}					
		break;
		
		case I2C_FROM_MASTER:
		{
			data = I2C1RCV;										// Read data
			if (I2C_1_Slave_Data.data_from_master_callback(I2C_1, data))
				I2C_1_Slave_Data.state = I2C_IDLE;
		}
		break;
		
		case I2C_END_TO_MASTER:
			I2C_1_Slave_Data.state = I2C_IDLE;
			I2C1CONbits.SCLREL = 1;									// Release clock
		break;
		
	}
	
	_SI2C1IF = 0;				// Clear Slave interrupt flag
}


/**
	I2C 2 Interrupt Service Routine.
 
	Call the state-specific user-defined function.
*/
void _ISR _SI2C2Interrupt(void)
{
	unsigned char data;
	
	// no interrupt is generated at the end of cycle,
	// nor all way to detect beginning of cycle are buggy
	// and do not behave as the doc predicts
	if (I2C_2_Slave_Data.state == I2C_IDLE)
	{
		if (I2C2STATbits.R_W)
		{
			I2C_2_Slave_Data.state = I2C_TO_MASTER;
			I2C_2_Slave_Data.message_to_master_callback(I2C_2);
		}
		else
		{
			I2C_2_Slave_Data.state = I2C_FROM_MASTER;
			I2C_2_Slave_Data.message_from_master_callback(I2C_2);
			_SI2C2IF = 0;										// Clear Slave interrupt flag
			return;
		}
	}
	
	switch (I2C_2_Slave_Data.state)
	{
		case I2C_TO_MASTER:
		{
			if (I2C_2_Slave_Data.data_to_master_callback(I2C_2, &data))
				I2C_2_Slave_Data.state = I2C_END_TO_MASTER;
			I2C2TRN = data; 									// Write data
			I2C2CONbits.SCLREL = 1;								// Release clock
		}
		break;
		
		case I2C_FROM_MASTER:
		{
			data = I2C2RCV;										// Read data
			if (I2C_2_Slave_Data.data_from_master_callback(I2C_2, data))
				I2C_2_Slave_Data.state = I2C_IDLE;
		}
		break;
		
		case I2C_END_TO_MASTER:
			I2C_2_Slave_Data.state = I2C_IDLE;
			I2C2CONbits.SCLREL = 1;									// Release clock
		break;
		
	}
	
	_SI2C2IF = 0;				// Clear Slave interrupt flag
}

/*@}*/
