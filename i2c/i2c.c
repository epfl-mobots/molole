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

void i2c_init(unsigned char address)
{
	I2C1ADD = address;				// Set the module address defined in the program
	I2C1CONbits.I2CEN = 1;			// Enables the I2C module and configures the SDA and SCL pins as serial port pins
	I2C1CONbits.I2CSIDL = 0;		// Continue module operation in idle mode
	I2C1CONbits.SCLREL = 0;			// Release SCLx clock
	I2C1CONbits.IPMIEN = 0;			// Only acknowledge own address
	I2C1CONbits.A10M = 0;			// 7bit slave address
	I2C1CONbits.DISSLW = 1;			// Slew rate control disabled (enable for 400kHz operation!)
	I2C1CONbits.SMEN = 0;			// Disable SMBus Input thresholds (set for 3.3V operation!)
	I2C1CONbits.GCEN = 0;			// General call address disabled
	I2C1CONbits.STREN = 0;			// Disable software or receive clock stretching
	I2C1CONbits.ACKDT = 0;			// Send ACK during acknowledge
	I2C1CONbits.ACKEN = 0;			// Acknowledge sequence not in progress
	I2C1CONbits.RCEN = 0;			// Receive sequence not in progress
	I2C1CONbits.PEN = 0;			// STOP condition not in progress
	I2C1CONbits.RSEN = 0;			// Repeated START condition not in progress
	I2C1CONbits.SEN = 0;			// START condition not in progress

	I2C1STATbits.I2COV = 0;			// clear Overflow flag
}


