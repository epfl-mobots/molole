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

#ifndef _MOLOLE_ERROR_H
#define _MOLOLE_ERROR_H

/** \addtogroup error */
/*@{*/

/** \file
	\brief An error management library for callback-based assertions
*/

/** Callback when an error occurs */
typedef void(*error_callback)(const char * file, int line, int id, void* arg);

// Macros, to get file and line

/** Report an error, alongside the file name and line number and return */
#define ERROR(id, arg) { error_report(__FILE__, __LINE__, id, arg); return; }

// Functions, doc in the .c

void error_report(const char * file, int line, int id, void*arg);

void error_register_callback(error_callback callback);

/*@}*/

#endif
