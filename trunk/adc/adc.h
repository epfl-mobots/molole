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

/** \file
	\brief A wrapper around dsPIC33 ADC.
*/

#ifndef _MOLOLE_ADC_H
#define _MOLOLE_ADC_H

#include "../types/types.h"

/** ADC callback when conversion is completed */
typedef void(*adc_simple_callback)(int channel, int value);


// Functions, doc in the .c

void adc1_init_simple(adc_simple_callback callback, int priority, unsigned long inputs, int sample_time);

void adc1_start_simple_conversion(int channel);

#endif
