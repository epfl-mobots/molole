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
 * 		<li>February 2008 : Philippe Rétornaz ; Rework API</li>
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

static enum master_operation		I2C_MasterOperation;	/**< Store the current operation, as defined by the enum #master_operation */
volatile static enum master_state	I2C_MasterState;		/**< Store the current state, as defined by the enum #master_state */
static unsigned char				I2C_MasterAddress;		/**< Address of the distant slave module */
static unsigned char				I2C_MasterRegister;		/**< Address of the distant register on the slave module */
static unsigned char*				I2C_MasterDataTray;		/**< Tray (Multiple bytes) to send */
static int							I2C_MasterDataCounter;	/**< Number of bytes of #I2C_MasterDataTray */
static i2c_error_callback error_cb;
volatile static bool last_oper_failed = false;

static void error_wrapper(int error) {
	if(error != I2C_OPERATION_FINISHED)
		if(error_cb)
			error_cb(error);
}



bool i2c_read(i2c_error_callback error_callback, unsigned char device_add, unsigned char reg, unsigned char *data, unsigned int size) {

	while(i2c_master_start_read(error_wrapper, device_add, reg, data, size)) Idle();

	while(i2c_master_is_busy()) Idle();
	return last_oper_failed;
}

bool i2c_write(i2c_error_callback error_callback, unsigned char device_add, unsigned char reg, unsigned char *data, unsigned int size) {

	while(i2c_master_start_send(error_wrapper, device_add, reg, data, size)) Idle();

	while(i2c_master_is_busy()) Idle();
	return last_oper_failed;
}

int i2c_master_start_send(i2c_error_callback error_callback, unsigned char mod_addr, unsigned char mod_reg, unsigned char *data, unsigned int size)
{
	if(I2C_MasterState != idle)
	{
		return -1;
	}
	
	error_cb = error_callback;
	// ok ! Now we are writing data
	I2C_MasterOperation = sending;
	
	// Set the values to send
	I2C_MasterAddress = mod_addr;
	I2C_MasterRegister = mod_reg;
	I2C_MasterDataTray = data;
	I2C_MasterDataCounter = size;
	
	// Initiate a start condition to begin the operation and set the next state
	I2C_MasterState = send_mod_addr;
	I2C1CONbits.SEN = 1;
	
	return 0;	
}

int i2c_master_start_read(i2c_error_callback error_callback, unsigned char mod_addr, unsigned char mod_reg, unsigned char * data, unsigned int size)
{
	if(I2C_MasterState != idle)
	{
		return -1;
	}
	
	error_cb = error_callback;
	// ok ! Now we are reading data
	I2C_MasterOperation = reading;
	
	// Set the values to send
	I2C_MasterAddress = mod_addr;
	I2C_MasterRegister = mod_reg;;
	I2C_MasterDataTray = data;
	I2C_MasterDataCounter = size;
	
	// Initiate a start condition to begin the operation and set the next state
	I2C_MasterState = send_mod_addr;
	I2C1CONbits.SEN = 1;
	
	return 0;
}


int i2c_master_is_busy(void)
{
	if(I2C_MasterState != idle)
		return 1;
	else
		return 0;
}


void i2c_init_master(long speed, int priority)
{
	unsigned long brg;
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	brg = (clock_get_cycle_frequency()/speed - clock_get_cycle_frequency()/1111111) - 1;
	ERROR_CHECK_RANGE(brg, 1, 65535, I2C_INVALID_CLOCK);
	I2C1BRG = (unsigned int) brg;
	_MI2C1IF = 0;					// clear the master interrupt
	_MI2C1IP = priority;			// set the master interrupt priority

	_MI2C1IE = 1;					// enable the master interrupt*/
}



void _ISR _MI2C1Interrupt(void)
{
	switch(I2C_MasterState)
	{
		case idle:
			// should never arrive !!!!! Error !!!!
			ERROR(I2C_INTERNAL_ERROR, (unsigned int *)&I2C_MasterState);
			break;
		case send_mod_addr:
			// we must send the module address
			I2C1TRN = ((I2C_MasterAddress << 1) + 0);		// write
			I2C_MasterState = send_mod_reg;

			break;
		case send_mod_reg:
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1){

				I2C_MasterOperation = I2C_ERROR_GOT_NACK_TO_ADDRESS;
				I2C1CONbits.PEN = 1;
				I2C_MasterState = failed;
				break;
			}
				
			// we must send the module register
			I2C1TRN = I2C_MasterRegister;
			
			// ok ! what next ?
			if(I2C_MasterOperation == sending){
				I2C_MasterState = send_byte;
			}else if(I2C_MasterOperation == reading){
				I2C_MasterState = read_repstart_gen;
			}else{
				ERROR(I2C_INTERNAL_ERROR, &I2C_MasterOperation);
			}
			break;
		case send_byte:			
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1){

				I2C_MasterOperation = I2C_ERROR_GOT_NACK_AFTER_REGISTER;
			
				/* Abort */
				I2C1CONbits.PEN = 1;
				I2C_MasterState = failed;
				break;
			}
				
			I2C1TRN = *I2C_MasterDataTray;
				
			I2C_MasterDataCounter--;
			
			if(I2C_MasterDataCounter)	// other bytes to send
				I2C_MasterState = send_another_byte;
			else
				I2C_MasterState = stop_ack;	// ok !
				
			break;
		case send_another_byte:
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1){
				I2C_MasterOperation = I2C_ERROR_GOT_NACK_AFTER_DATA;
			
				/* Abort */
				I2C1CONbits.PEN = 1;
				I2C_MasterState = failed;
				break;
			}

			// send the next byte
			I2C1TRN = *(++I2C_MasterDataTray);
			I2C_MasterDataCounter--;

			// finished ?
			if(I2C_MasterDataCounter == 0)
				I2C_MasterState = stop_ack;
			
			break;
		case read_repstart_gen:
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1){
				I2C_MasterOperation = I2C_ERROR_GOT_NACK_AFTER_REGISTER;
			
				/* Abort */
				I2C1CONbits.PEN = 1;
				I2C_MasterState = failed;
				break;
			}

			// generate a repeated start event
			I2C1CONbits.RSEN = 1;
			
			I2C_MasterState = read_mod_addr;
			break;
		case read_mod_addr:
			// send again the module address, with Read bit on
			I2C1TRN = ((I2C_MasterAddress << 1) + 1);		// read
			I2C_MasterState = read_data;

			break;
		case read_data:
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1){
				I2C_MasterOperation = I2C_ERROR_GOT_NACK_TO_ADDRESS;
			
				/* Abort */
				I2C1CONbits.PEN = 1;
				I2C_MasterState = failed;
			}
		case read_data_noack:
			// we must read the data, enable the receive mode
			I2C1CONbits.RCEN = 1;

			I2C_MasterState = data_received;
			break;
		case data_received:
			/* No ACK checking ... */
			
			// read the value
			*I2C_MasterDataTray = I2C1RCV;
			I2C_MasterDataTray++;
			
			I2C_MasterDataCounter--;

			// finished ?
			if(I2C_MasterDataCounter == 0) {		
				// generate a non acknowledgment
				I2C1CONbits.ACKDT = 1;
				I2C1CONbits.ACKEN = 1;
				I2C_MasterState = stop_noack;
			} else {
				// generate an acknowledgment
				I2C1CONbits.ACKDT = 0;
				I2C1CONbits.ACKEN = 1;
				I2C_MasterState = read_data_noack;
			}

			break;
		case stop_ack:
			// ack or not ?
			if(I2C1STATbits.ACKSTAT == 1) {
				I2C_MasterOperation = I2C_ERROR_GOT_NACK_AFTER_DATA;
				I2C1CONbits.PEN = 1;
				I2C_MasterState = failed;
				break;
			}

		case stop_noack:
			// generate stop condition
			I2C1CONbits.PEN = 1;
			
			// finished !
			I2C_MasterState = completed;
			break;			

		case completed:
			// We set idle/nothing before calling so the cb can immediatly restart a transfert
			I2C_MasterState = idle;
			last_oper_failed = false;
			if(error_cb)
				error_cb(I2C_OPERATION_FINISHED);
			break;

		case failed:
			I2C_MasterState = idle;
			last_oper_failed = true;
			if(error_cb)
				error_cb(I2C_MasterOperation);
			break;
	}
	
	IFS1bits.MI2C1IF=0;			// clear master interrupt flag
}
