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

#ifndef _MOLOLE_ERROR_H
#define _MOLOLE_ERROR_H

/** \addtogroup error */
/*@{*/

/** \file
	\brief An error management library for callback-based assertions
*/

/** Generic errors anyone can throw */
enum generic_errors
{
	GENERIC_ERROR_BASE = 0x0000,
	GENERIC_ERROR_NOT_IMPLEMENTED,				/**< An not yet implemented code was called. */
	GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY,	/**< A requested interrupt priority was not between 1 and 7 */
	GENERIC_ERROR_STACK_SPACE_EXHAUSTED			/**< No more room in stack for requested operation */
};

// Every molole file which has interrupt include error.h so redefine _ISR here 
#ifdef _ISR
#undef _ISR
#endif
#define _ISR __attribute__((interrupt,auto_psv))


/** Callback when an error occurs */
typedef void  __attribute__((noreturn)) (*error_callback)(const char * file, int line, int id, void* arg);

// Macros, to get file and line

/** Report an error, alongside the file name and line number and return */
#define ERROR(id, arg) { error_report(__FILE__, __LINE__, (id), (arg)); }

/** Report an error, alongside the file name and line number and return 0 */
#define ERROR_RET_0(id, arg) { error_report(__FILE__, __LINE__, (id), (arg)); }

/** Report an error, alongside the file name and line number, if a variable is outside the bounds of a specific range */
#define ERROR_CHECK_RANGE(var, min, max, id) if ((var) < (min) || (var) > (max)) { error_report(__FILE__, __LINE__, (id), &(var));}


// Functions, doc in the .c

void __attribute__((noreturn)) error_report(const char * file, int line, int id, void*arg);

void  error_register_callback(error_callback callback);

/*@}*/

#endif
