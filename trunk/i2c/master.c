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

#include <p33fxxxx.h>
#include "i2c.h"
#include "../error/error.h"
#include "../clock/clock.h"

// TODO: major cleanup/refactoring needed!

/** \file
 *  \brief
 *  dsPIC33 I2C Master implementation.
 * 
 *  This file implement the master module.
 *	Compiles into an object file, part of libmarxbot.a. See the file i2c.h for more details.
 *
 *	<h2>Resources needed</h2>
 *	<ul>
 *		<li>SCL,SDA ports</li>
 *	</ul>
 *	
 *	<h2>Revision history</h2>
 *	<ol type="i">
 *  	<li>January 2007  :  Florian Vaussard  ;  First version</li>
 *  	<li>March 2007  :  Florian Vaussard  ;  Doxygen documentation</li>
 * 		<li>February 2008 : Philippe R�tornaz ; Rework API</li>
 *	</ol> 	
 */

/**
 *	\brief	Describe the current operation.
 *
 *	Describe the current operation.
 */
enum master_operation{
	nothing,	/**< The module is in idle state */
	sending,	/**< The module is sending data to another module */
	reading		/**< The module is reading data from another module */
};

/**
 *	\brief	Describe the current state of the master's internal state machine
 *
 *	Describe the current state of the master's internal state machine. This enumeration makes easier to unterstand the state machine
 *	of the master interrupt service routine.
 */
enum master_state{
	idle,				/**< Idle state */
	send_mod_addr,		/**< The master will send the address of the module */
	send_mod_reg,		/**< The master will send the address of the register */
	send_byte,			/**< The master will send the first byte */
	send_another_byte,	/**< The master will send another byte (for a multiple bytes tray) */
	read_repstart_gen,	/**< The master will generate a repeated start */
	read_mod_addr,		/**< The master will send again the address of the module, with the read bit set */
	read_data,			/**< The master will receive a byte */
	read_data_noack,	/**< The master will receive a byte and will not check the ACK bit */
	data_received,		/**< The master will has received a byte */
	stop_ack,			/**< The master will generate a stop condition and check for the ack bit */
	stop_noack,			/**< The master will generate a stop condition but will not check for the ack bit*/
	completed,			/**< The master will finish the operation */
	failed,				/**< The master will finish the operation, the cb will be called with the error */
};	

typedef struct 
{
	int operation;				/**< Store the current operation, as defined by the enum #master_operation */
	int state;					/**< Store the current state, as defined by the enum #master_state */
	unsigned char address;		/**< Address of the distant slave module */
	unsigned char reg;			/**< Address of the distant register on the slave module */
	unsigned char* data_tray;	/**< Tray (Multiple bytes) to send */
	int data_counter;			/**< Number of bytes of #I2C_MasterDataTray */
	i2c_error_callback error_cb;
	bool last_oper_failed;
} I2C_Master_Data;

/** data for the master I2C 1 and 2 wrappers */
static I2C_Master_Data I2C_Master_Datas[2];

// static enum master_operation		I2C_MasterOperation;	/**< Store the current operation, as defined by the enum #master_operation */
// volatile static enum master_state	I2C_MasterState;		/**< Store the current state, as defined by the enum #master_state */
// static unsigned char				I2C_MasterAddress;		/**< Address of the distant slave module */
// static unsigned char				I2C_MasterRegister;		/**< Address of the distant register on the slave module */
// static unsigned char*				I2C_MasterDataTray;		/**< Tray (Multiple bytes) to send */
// static int							I2C_MasterDataCounter;	/**< Number of bytes of #I2C_MasterDataTray */
// static i2c_error_callback error_cb;
// volatile static bool last_oper_failed = false;

static void error_wrapper(int i2c_id, int error)
{
	if ((i2c_id < I2C_1) || (i2c_id > I2C_2))
		ERROR(I2C_ERROR_INVALID_ID, &i2c_id);
	
	if (error != I2C_OPERATION_FINISHED)
		if (I2C_Master_Datas[i2c_id].error_cb)
			I2C_Master_Datas[i2c_id].error_cb(i2c_id, error);
}

bool i2c_last_op_failed(int i2c_id)
{
	if ((i2c_id < I2C_1) || (i2c_id > I2C_2))
		ERROR(I2C_ERROR_INVALID_ID, &i2c_id);
	
	return I2C_Master_Datas[i2c_id].last_oper_failed;
}


bool i2c_read(int i2c_id, i2c_error_callback error_callback, unsigned char device_add, unsigned char reg, unsigned char *data, unsigned int size) {

	while(i2c_master_start_read(i2c_id, error_wrapper, device_add, reg, data, size))
		Idle();

	while(i2c_master_is_busy(i2c_id))
		Idle();
	
	return i2c_last_op_failed(i2c_id);
}

bool i2c_write(int i2c_id, i2c_error_callback error_callback, unsigned char device_add, unsigned char reg, unsigned char *data, unsigned int size) {

	while(i2c_master_start_send(i2c_id, error_wrapper, device_add, reg, data, size))
		Idle();

	while(i2c_master_is_busy(i2c_id))
		Idle();
	
	return i2c_last_op_failed(i2c_id);
}

int i2c_master_start_send(int i2c_id, i2c_error_callback error_callback, unsigned char mod_addr, unsigned char mod_reg, unsigned char *data, unsigned int size)
{
	if ((i2c_id < I2C_1) || (i2c_id > I2C_2))
		ERROR(I2C_ERROR_INVALID_ID, &i2c_id);
		
	if (I2C_Master_Datas[i2c_id].state != idle)
	{
		return -1;
	}
	
	I2C_Master_Datas[i2c_id].error_cb = error_callback;
	// ok ! Now we are writing data
	I2C_Master_Datas[i2c_id].operation = sending;
	
	// Set the values to send
	I2C_Master_Datas[i2c_id].address = mod_addr;
	I2C_Master_Datas[i2c_id].reg = mod_reg;
	I2C_Master_Datas[i2c_id].data_tray = data;
	I2C_Master_Datas[i2c_id].data_counter = size;
	
	// Initiate a start condition to begin the operation and set the next state
	I2C_Master_Datas[i2c_id].state = send_mod_addr;
	
	if (i2c_id == I2C_1)
		I2C1CONbits.SEN = 1;
	else
		I2C2CONbits.SEN = 1;
	
	return 0;	
}

int i2c_master_start_read(int i2c_id, i2c_error_callback error_callback, unsigned char mod_addr, unsigned char mod_reg, unsigned char * data, unsigned int size)
{
	if ((i2c_id < I2C_1) || (i2c_id > I2C_2))
		ERROR(I2C_ERROR_INVALID_ID, &i2c_id);
		
	if(I2C_Master_Datas[i2c_id].state != idle)
	{
		return -1;
	}
	
	I2C_Master_Datas[i2c_id].error_cb = error_callback;
	// ok ! Now we are reading data
	I2C_Master_Datas[i2c_id].operation = reading;
	
	// Set the values to send
	I2C_Master_Datas[i2c_id].address = mod_addr;
	I2C_Master_Datas[i2c_id].reg = mod_reg;;
	I2C_Master_Datas[i2c_id].data_tray = data;
	I2C_Master_Datas[i2c_id].data_counter = size;
	
	// Initiate a start condition to begin the operation and set the next state
	I2C_Master_Datas[i2c_id].state = send_mod_addr;
	
	if (i2c_id == I2C_1)
		I2C1CONbits.SEN = 1;
	else
		I2C2CONbits.SEN = 1;
	
	return 0;
}


int i2c_master_is_busy(int i2c_id)
{
	if ((i2c_id < I2C_1) || (i2c_id > I2C_2))
		ERROR(I2C_ERROR_INVALID_ID, &i2c_id);
		
	if(I2C_Master_Datas[i2c_id].state != idle)
		return 1;
	else
		return 0;
}


void i2c_init_master(int i2c_id, long speed, int priority)
{
	unsigned long brg;
	
	if ((i2c_id < I2C_1) || (i2c_id > I2C_2))
		ERROR(I2C_ERROR_INVALID_ID, &i2c_id);
	
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	brg = (clock_get_cycle_frequency()/speed - clock_get_cycle_frequency()/1111111) - 1;
	ERROR_CHECK_RANGE(brg, 1, 65535, I2C_INVALID_CLOCK);
	
	if (i2c_id == I2C_1)
	{
		I2C1BRG = (unsigned int) brg;
		_MI2C1IF = 0;					// clear the master interrupt
		_MI2C1IP = priority;			// set the master interrupt priority
	
		_MI2C1IE = 1;					// enable the master interrupt*/
	}
	else
	{
		I2C2BRG = (unsigned int) brg;
		_MI2C2IF = 0;					// clear the master interrupt
		_MI2C2IP = priority;			// set the master interrupt priority
	
		_MI2C2IE = 1;					// enable the master interrupt*/
	}
}

void _ISR _MI2C1Interrupt(void)
{
	switch(I2C_Master_Datas[I2C_1].state)
	{
		case idle:
			// should never arrive !!!!! Error !!!!
			ERROR(I2C_INTERNAL_ERROR, (unsigned int *)&I2C_Master_Datas[I2C_1].state);
			break;
		case send_mod_addr:
			// we must send the module address
			I2C1TRN = ((I2C_Master_Datas[I2C_1].address << 1) + 0);		// write
			I2C_Master_Datas[I2C_1].state = send_mod_reg;

			break;
		case send_mod_reg:
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1){

				I2C_Master_Datas[I2C_1].operation = I2C_ERROR_GOT_NACK_TO_ADDRESS;
				I2C1CONbits.PEN = 1;
				I2C_Master_Datas[I2C_1].state = failed;
				break;
			}
				
			// we must send the module register
			I2C1TRN = I2C_Master_Datas[I2C_1].reg;
			
			// ok ! what next ?
			if(I2C_Master_Datas[I2C_1].operation == sending){
				I2C_Master_Datas[I2C_1].state = send_byte;
			}else if(I2C_Master_Datas[I2C_1].operation == reading){
				I2C_Master_Datas[I2C_1].state = read_repstart_gen;
			}else{
				ERROR(I2C_INTERNAL_ERROR, &I2C_Master_Datas[I2C_1].operation);
			}
			break;
		case send_byte:			
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1){

				I2C_Master_Datas[I2C_1].operation = I2C_ERROR_GOT_NACK_AFTER_REGISTER;
			
				/* Abort */
				I2C1CONbits.PEN = 1;
				I2C_Master_Datas[I2C_1].state = failed;
				break;
			}
				
			I2C1TRN = *I2C_Master_Datas[I2C_1].data_tray;
				
			I2C_Master_Datas[I2C_1].data_counter--;
			
			if(I2C_Master_Datas[I2C_1].data_counter)	// other bytes to send
				I2C_Master_Datas[I2C_1].state = send_another_byte;
			else
				I2C_Master_Datas[I2C_1].state = stop_ack;	// ok !
				
			break;
		case send_another_byte:
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1){
				I2C_Master_Datas[I2C_1].operation = I2C_ERROR_GOT_NACK_AFTER_DATA;
			
				/* Abort */
				I2C1CONbits.PEN = 1;
				I2C_Master_Datas[I2C_1].state = failed;
				break;
			}

			// send the next byte
			I2C1TRN = *(++I2C_Master_Datas[I2C_1].data_tray);
			I2C_Master_Datas[I2C_1].data_counter--;

			// finished ?
			if(I2C_Master_Datas[I2C_1].data_counter == 0)
				I2C_Master_Datas[I2C_1].state = stop_ack;
			
			break;
		case read_repstart_gen:
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1){
				I2C_Master_Datas[I2C_1].operation = I2C_ERROR_GOT_NACK_AFTER_REGISTER;
			
				/* Abort */
				I2C1CONbits.PEN = 1;
				I2C_Master_Datas[I2C_1].state = failed;
				break;
			}

			// generate a repeated start event
			I2C1CONbits.RSEN = 1;
			
			I2C_Master_Datas[I2C_1].state = read_mod_addr;
			break;
		case read_mod_addr:
			// send again the module address, with Read bit on
			I2C1TRN = ((I2C_Master_Datas[I2C_1].address << 1) + 1);		// read
			I2C_Master_Datas[I2C_1].state = read_data;

			break;
		case read_data:
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1){
				I2C_Master_Datas[I2C_1].operation = I2C_ERROR_GOT_NACK_TO_ADDRESS;
			
				/* Abort */
				I2C1CONbits.PEN = 1;
				I2C_Master_Datas[I2C_1].state = failed;
			}
		case read_data_noack:
			// we must read the data, enable the receive mode
			I2C1CONbits.RCEN = 1;

			I2C_Master_Datas[I2C_1].state = data_received;
			break;
		case data_received:
			/* No ACK checking ... */
			
			// read the value
			*I2C_Master_Datas[I2C_1].data_tray = I2C1RCV;
			I2C_Master_Datas[I2C_1].data_tray++;
			
			I2C_Master_Datas[I2C_1].data_counter--;

			// finished ?
			if(I2C_Master_Datas[I2C_1].data_counter == 0) {		
				// generate a non acknowledgment
				I2C1CONbits.ACKDT = 1;
				I2C1CONbits.ACKEN = 1;
				I2C_Master_Datas[I2C_1].state = stop_noack;
			} else {
				// generate an acknowledgment
				I2C1CONbits.ACKDT = 0;
				I2C1CONbits.ACKEN = 1;
				I2C_Master_Datas[I2C_1].state = read_data_noack;
			}

			break;
		case stop_ack:
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1) {
				I2C_Master_Datas[I2C_1].operation = I2C_ERROR_GOT_NACK_AFTER_DATA;
				I2C1CONbits.PEN = 1;
				I2C_Master_Datas[I2C_1].state = failed;
				break;
			}

		case stop_noack:
			// generate stop condition
			I2C1CONbits.PEN = 1;
			
			// finished !
			I2C_Master_Datas[I2C_1].state = completed;
			break;			

		case completed:
			// We set idle/nothing before calling so the cb can immediatly restart a transfert
			I2C_Master_Datas[I2C_1].state = idle;
			I2C_Master_Datas[I2C_1].last_oper_failed = false;
			if (I2C_Master_Datas[I2C_1].error_cb)
				I2C_Master_Datas[I2C_1].error_cb(I2C_1, I2C_OPERATION_FINISHED);
			break;

		case failed:
			I2C_Master_Datas[I2C_1].state = idle;
			I2C_Master_Datas[I2C_1].last_oper_failed = true;
			if (I2C_Master_Datas[I2C_1].error_cb)
				I2C_Master_Datas[I2C_1].error_cb(I2C_1, I2C_Master_Datas[I2C_1].operation);
			break;
	}
	
	_MI2C1IF = 0;			// clear master interrupt flag
}

void _ISR _MI2C2Interrupt(void)
{
	switch(I2C_Master_Datas[I2C_2].state)
	{
		case idle:
			// should never arrive !!!!! Error !!!!
			ERROR(I2C_INTERNAL_ERROR, (unsigned int *)&I2C_Master_Datas[I2C_2].state);
			break;
		case send_mod_addr:
			// we must send the module address
			I2C2TRN = ((I2C_Master_Datas[I2C_2].address << 1) + 0);		// write
			I2C_Master_Datas[I2C_2].state = send_mod_reg;

			break;
		case send_mod_reg:
			// ack or not ?
			if(I2C2STATbits.ACKSTAT == 1){

				I2C_Master_Datas[I2C_2].operation = I2C_ERROR_GOT_NACK_TO_ADDRESS;
				I2C2CONbits.PEN = 1;
				I2C_Master_Datas[I2C_2].state = failed;
				break;
			}
				
			// we must send the module register
			I2C2TRN = I2C_Master_Datas[I2C_2].reg;
			
			// ok ! what next ?
			if(I2C_Master_Datas[I2C_2].operation == sending){
				I2C_Master_Datas[I2C_2].state = send_byte;
			}else if(I2C_Master_Datas[I2C_2].operation == reading){
				I2C_Master_Datas[I2C_2].state = read_repstart_gen;
			}else{
				ERROR(I2C_INTERNAL_ERROR, &I2C_Master_Datas[I2C_2].operation);
			}
			break;
		case send_byte:			
			// ack or not ?
			if(I2C2STATbits.ACKSTAT == 1){

				I2C_Master_Datas[I2C_2].operation = I2C_ERROR_GOT_NACK_AFTER_REGISTER;
			
				/* Abort */
				I2C2CONbits.PEN = 1;
				I2C_Master_Datas[I2C_2].state = failed;
				break;
			}
				
			I2C2TRN = *I2C_Master_Datas[I2C_2].data_tray;
				
			I2C_Master_Datas[I2C_2].data_counter--;
			
			if(I2C_Master_Datas[I2C_2].data_counter)	// other bytes to send
				I2C_Master_Datas[I2C_2].state = send_another_byte;
			else
				I2C_Master_Datas[I2C_2].state = stop_ack;	// ok !
				
			break;
		case send_another_byte:
			// ack or not ?
			if(I2C2STATbits.ACKSTAT == 1){
				I2C_Master_Datas[I2C_2].operation = I2C_ERROR_GOT_NACK_AFTER_DATA;
			
				/* Abort */
				I2C2CONbits.PEN = 1;
				I2C_Master_Datas[I2C_2].state = failed;
				break;
			}

			// send the next byte
			I2C2TRN = *(++I2C_Master_Datas[I2C_2].data_tray);
			I2C_Master_Datas[I2C_2].data_counter--;

			// finished ?
			if(I2C_Master_Datas[I2C_2].data_counter == 0)
				I2C_Master_Datas[I2C_2].state = stop_ack;
			
			break;
		case read_repstart_gen:
			// ack or not ?
			if(I2C2STATbits.ACKSTAT == 1){
				I2C_Master_Datas[I2C_2].operation = I2C_ERROR_GOT_NACK_AFTER_REGISTER;
			
				/* Abort */
				I2C2CONbits.PEN = 1;
				I2C_Master_Datas[I2C_2].state = failed;
				break;
			}

			// generate a repeated start event
			I2C2CONbits.RSEN = 1;
			
			I2C_Master_Datas[I2C_2].state = read_mod_addr;
			break;
		case read_mod_addr:
			// send again the module address, with Read bit on
			I2C2TRN = ((I2C_Master_Datas[I2C_2].address << 1) + 1);		// read
			I2C_Master_Datas[I2C_2].state = read_data;

			break;
		case read_data:
			// ack or not ?
			if(I2C2STATbits.ACKSTAT == 1){
				I2C_Master_Datas[I2C_2].operation = I2C_ERROR_GOT_NACK_TO_ADDRESS;
			
				/* Abort */
				I2C2CONbits.PEN = 1;
				I2C_Master_Datas[I2C_2].state = failed;
			}
		case read_data_noack:
			// we must read the data, enable the receive mode
			I2C2CONbits.RCEN = 1;

			I2C_Master_Datas[I2C_2].state = data_received;
			break;
		case data_received:
			/* No ACK checking ... */
			
			// read the value
			*I2C_Master_Datas[I2C_2].data_tray = I2C2RCV;
			I2C_Master_Datas[I2C_2].data_tray++;
			
			I2C_Master_Datas[I2C_2].data_counter--;

			// finished ?
			if(I2C_Master_Datas[I2C_2].data_counter == 0) {		
				// generate a non acknowledgment
				I2C2CONbits.ACKDT = 1;
				I2C2CONbits.ACKEN = 1;
				I2C_Master_Datas[I2C_2].state = stop_noack;
			} else {
				// generate an acknowledgment
				I2C2CONbits.ACKDT = 0;
				I2C2CONbits.ACKEN = 1;
				I2C_Master_Datas[I2C_2].state = read_data_noack;
			}

			break;
		case stop_ack:
			// ack or not ?
			if(I2C2STATbits.ACKSTAT == 1) {
				I2C_Master_Datas[I2C_2].operation = I2C_ERROR_GOT_NACK_AFTER_DATA;
				I2C2CONbits.PEN = 1;
				I2C_Master_Datas[I2C_2].state = failed;
				break;
			}

		case stop_noack:
			// generate stop condition
			I2C2CONbits.PEN = 1;
			
			// finished !
			I2C_Master_Datas[I2C_2].state = completed;
			break;			

		case completed:
			// We set idle/nothing before calling so the cb can immediatly restart a transfert
			I2C_Master_Datas[I2C_2].state = idle;
			I2C_Master_Datas[I2C_2].last_oper_failed = false;
			if (I2C_Master_Datas[I2C_2].error_cb)
				I2C_Master_Datas[I2C_2].error_cb(I2C_2, I2C_OPERATION_FINISHED);
			break;

		case failed:
			I2C_Master_Datas[I2C_2].state = idle;
			I2C_Master_Datas[I2C_2].last_oper_failed = true;
			if (I2C_Master_Datas[I2C_2].error_cb)
				I2C_Master_Datas[I2C_2].error_cb(I2C_2, I2C_Master_Datas[I2C_2].operation);
			break;
	}
	
	_MI2C2IF = 0;			// clear master interrupt flag
}

