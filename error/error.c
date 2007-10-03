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

//--------------------
// Usage documentation
//--------------------

/**
	\defgroup error
	
	A simple error management library for callback-based assertions.
*/
/*@{*/

/** \file
	\brief Implementation of the error management library for callback-based assertions
*/


//------------
// Definitions
//------------

#include "error.h"

//-----------------------
// Structures definitions
//-----------------------

static void error_default_handler(const char * file, int line, int id, void* arg);

/** error management library data */
static struct
{
	error_callback callback;  /**< function to call when an error occurs */
} Error_Data = { &error_default_handler };


//-------------------
// Exported functions
//-------------------

static void error_default_handler(const char * file, int line, int id, void* arg)
{
	// do nothing
}

void error_report(const char * file, int line, int id, void* arg)
{
	Error_Data.callback(file, line, id, arg);
}

void error_register_callback(error_callback callback)
{
	Error_Data.callback = callback;
}

/*@}*/