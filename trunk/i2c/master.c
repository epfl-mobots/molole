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

#include "i2c.h"

/**
	I2C Master yet to implement
*/
void i2c_init_master(int priority)
{
/*	I2C1BRG = ((int)((FCY/FSCLI2CBUS)-1.0));	// Set the baud rate for master mode
	_MI2C1IF = 0;					// clear the master interrupt
	_MI2C1IP = priority;			// set the master interrupt priority
	_MI2C1IE = 1;					// enable the master interrupt*/
}
