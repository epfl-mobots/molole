/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	
	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>,
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

#ifndef _MOLOLE_ADC_H
#define _MOLOLE_ADC_H

#include "../types/types.h"
#include "../dma/dma.h"

/** \addtogroup adc */
/*@{*/

/** \file
	\brief A wrapper around dsPIC33 ADC.
*/

// Defines

/** Errors ADC can throw */
enum adc_errors
{
	ADC_ERROR_BASE = 0x0200,
	ADC_ERROR_CONVERSION_IN_PROGRESS,	/**< A conversion is already in progress */
	ADC_ERROR_INVALID_SAMPLE_TIME,		/**< The specified sample time is not valid (outside 0 .. 31) */
	ADC_ERROR_INVALID_START_CONVERSION_EVENT,	/**< A specified stop sampling and start conversion was not one of \ref adc_start_conversion_event */
	ADC_ERROR_INVALID_BUFFER_BUILD_MODE,	/**< A specified DMA buffer build mode was not one of \ref adc_dma_buffer_build_mode */
	ADC_ERROR_INVALID_BUFFER_SIZE_FOR_SCATTER_GATHER,	/**< A specified DMA buffer size was invalid for Scatter/Gather mode. */
	ADC_ERROR_TOO_MANY_INPUTS_FOR_SCATTER_GATHER,		/**< The Scatter/Gather mode is limited to 16 inputs */
};

/** Which event stop sampling and start conversion */
enum adc_start_conversion_event
{
	ADC_START_CONVERSION_MANUAL_CLEAR_SAMPLE_BIT = 0,	/**< Manual start*/
	ADC_START_CONVERSION_EXTERNAL_INT = 1,				/**< Active transition on INTx pin */
	ADC_START_CONVERSION_TIMER_COMPARE = 2,				/**< GP timer (Timer3 for ADC1, Timer5 for ADC2) compare */
	ADC_START_CONVERSION_MC_PWM = 3,					/**< Motor Control PWM */
	ADC_START_CONVERSION_FROM_INTERNAL_COUNTER = 7,		/**< Internal counter */
};

/** DMA Buffer Build Mode */
enum adc_dma_buffer_build_mode
{
	ADC_DMA_SCATTER_GATHER = 0,				/**< DMA buffers are written in Scatter/Gather mode. */
	ADC_DMA_CONVERSION_ORDER = 1,			/**< DMA buffers are written in the order of conversion. */
	ADC_DMA_CONVERSION_ORDER_ONESHOT = 2,	/**< DMA buffers are written in the order of conversion but only once. */
};

/** ADC callback when conversion is completed */
typedef void(*adc_simple_callback)(int channel, int value);


// Functions, doc in the .c

void adc1_init_simple(adc_simple_callback callback, int priority, unsigned long inputs, int sample_time);

void adc1_start_simple_conversion(int channel);

void adc1_init_scan_dma(unsigned long inputs, int start_conversion_event, int sample_time, int dma_channel, void * a, void * b, unsigned buffers_size, int buffer_build_mode, dma_callback callback);

void adc1_enable();

void adc2_disable();

void adc2_init_simple(adc_simple_callback callback, int priority, unsigned int inputs, int sample_time);

void adc2_start_simple_conversion(int channel);

void adc2_init_scan_dma(unsigned int inputs, int start_conversion_event, int sample_time, int dma_channel, void * a, void * b, unsigned buffers_size, int buffer_build_mode, dma_callback callback);

void adc2_enable();

void adc2_disable();

/*@}*/

#endif
