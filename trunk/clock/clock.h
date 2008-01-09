/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	Copyright (C) 2006 - 2007 Florian Vaussard <Florian dot Vaussard at a3 dot epfl dot ch>
	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>
	
	Copyright (C) 2004-2008 Mobots group http://mobots.epfl.ch
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

#ifndef _MOLOLE_CLOCK_H
#define _MOLOLE_CLOCK_H

/** \addtogroup clock */
/*@{*/

/** \file
	\brief Clock configuration
*/


// Functions, doc in the .c

void clock_init_internal_rc_from_n1_m_n2(unsigned n1, unsigned m, unsigned n2);

void clock_init_internal_rc_30();

void clock_init_internal_rc_40();

unsigned long clock_get_cycle_duration();

unsigned long clock_get_cycle_frequency();

/*@}*/

#endif
