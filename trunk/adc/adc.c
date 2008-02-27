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

//--------------------
// Usage documentation
//--------------------

/**
	\defgroup adc ADC
	
	This very simple wrapper ease the usage of the ADC converter.
*/
/*@{*/

/** \file
	Implementation of the wrapper around dsPIC33 ADC.
*/

//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "adc.h"
#include "../error/error.h"

//-----------------------
// Structures definitions
//-----------------------

/** ADC wrapper data */
static struct
{
	adc_simple_callback callback; /**< function to call upon conversion complete interrupt, 0 if none */
	int simple_channel; /**< channel on which last simple conversion was performed */
} ADC_Data[2] = { {0, -1},{0, -1} };


//-------------------
// Exported functions
//-------------------

/**
	Enable ADC 1.
	
	The ADC must be previously initialized by calling adc1_init_simple() or adc1_init_scan_dma().
	The user should call this function only if she hase previously called adc1_disable() manually.
*/
void adc1_enable()
{
	int i;
	
	// Turn on ADC Module
	AD1CON1bits.ADON = 1;
	
	// Wait 20 us, at 40 MIPS == 800 instructions
	i = 800/3;
	while (--i)
		Nop();
}

/** Disable ADC 1. */
void adc1_disable()
{
	// Turn off ADC Module
	AD1CON1bits.ADON = 0;
}

/**
	Initialize and enable ADC1 for simple input conversion.
	
	The converter is put in 12 bits / single conversion mode and callback is called when conversion is completed.
	This function does not start any conversion. Call adc1_start_simple_conversion() to start a conversion.
	
	\param	callback
			Pointer to a function that will be called when conversion is completed.
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 7 (highest priority)
	\param	inputs
			Bitfield that specify which physical input to use (AN0..AN31).
			1 put pins in analogic, 0 in digital.
	\param	sample_time
			Sample time, from 0 to 31, in number of ADC Internal RC Clock cycle

*/
void adc1_init_simple(adc_simple_callback callback, int priority, unsigned long inputs, int sample_time)
{
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	ERROR_CHECK_RANGE(sample_time, 0, 31, ADC_ERROR_INVALID_SAMPLE_TIME);
	
	adc1_disable();
	
	// Setup callback
	ADC_Data[0].callback = callback;
	_AD1IP = priority;
	
	// configure I/O pins in digital or analogic
#ifdef _CSS16
	AD1PCFGH = ~((unsigned short)(inputs >> 16));
#endif
	AD1PCFGL = ~((unsigned short)(inputs));
	// No input to scan
#ifdef _CSS16
	AD1CSSH = 0x0000;
#endif
	AD1CSSL = 0x0000;
	
	AD1CON1bits.ADSIDL = 0;		// Continue module operation in Idle mode
	AD1CON1bits.AD12B = 1;		// 12-bit, 1-channel ADC operation
	AD1CON1bits.FORM = 0;		// Integer (DOUT = 0000 dddd dddd dddd)
	AD1CON1bits.SSRC = 7;		// Internal counter ends sampling and starts conversion (auto-convert)
	AD1CON1bits.ASAM = 0;		// Sampling begins when SAMP bit is set
	
	AD1CON2bits.VCFG = 0;		// Avdd, Avss Converter Voltage References
	AD1CON2bits.CSCNA = 0;		// Do not scan inputs
	AD1CON2bits.BUFM = 0;		// Always starts filling the buffer from the start address.
	AD1CON2bits.ALTS = 0;		// Always uses channel input selects for Sample A
	
	AD1CON3bits.ADRC = 1;		// ADC Internal RC Clock: TAD == ADC Internal RC Clock cycle
	AD1CON3bits.SAMC = sample_time;	// Auto Sample Time bits sample_time*TAD
	AD1CON3bits.ADCS = 0;		// ADC Convertion Clock selection bits (ADCS+1)*TCY=TAD
	
	AD1CHS0bits.CH0NA = 0;		// Channel 0 negative input is VREFL
	AD1CHS0bits.CH0SA = 0;		// Channel 0 positive input is AN0
	
	// Reset ADC 1 interrupt flag
	_AD1IF = 0;
	// Enable ADC 1 Interrupt
	_AD1IE = 1;
	
	adc1_enable();
}

/**
	Request a conversion on a specific channel.
	
	The user must call adc1_init_simple prior to this function.
	
	\param	channel
			physical input to convert (AN0..AN31)
*/
void adc1_start_simple_conversion(int channel)
{
	// If a conversion is already in progress, throw an error
	if (ADC_Data[0].simple_channel >= 0)
		ERROR(ADC_ERROR_CONVERSION_IN_PROGRESS, &ADC_Data[0].simple_channel)
	
	// Select channel
	AD1CHS0bits.CH0SA = channel;
	ADC_Data[0].simple_channel = channel;
	
	// Start sampling
	AD1CON1bits.SAMP = 1;
}

/** Support function that returns the amount of bits at one in its argument */
unsigned amount_of_bits_at_one(unsigned long inputs)
{
	unsigned counter = 0;
	while (inputs)
	{
		if ((inputs & 1) == 1)
			counter++;
		inputs >>= 1;
	}
	return counter;
}

/** Support function that returns the log_2 of its argument for use in DMABL, with invalid values producing an error */
unsigned log_2(unsigned value)
{
	switch (value)
	{
		case 1: return 0;
		case 2: return 1;
		case 4: return 2;
		case 8: return 3;
		case 16: return 4;
		case 32: return 5;
		case 64: return 6;
		case 128: return 7;
		default: ERROR_RET_0(ADC_ERROR_INVALID_BUFFER_SIZE_FOR_SCATTER_GATHER, &value); return 0;
	}
}

/**
	Initialize and enable ADC1 for input scanning conversion using DMA.
	
	The converter is put in 12 bits.
	
	\param	inputs
			Bitfield that specify which physical input to use (AN0..AN31).
			1 put pins in analogic, 0 in digital.
	\param	start_conversion_event
			Which event triggers stop of sampling and start of conversion.
			Muse be one of \ref adc_start_conversion_event
	\param	sample_time
			If start_conversion_event is \ref ADC_START_CONVERSION_FROM_INTERNAL_COUNTER : Sample time, from 0 to 31, in number of ADC Internal RC Clock cycle. Otherwise ignored.
	\param	dma_channel
			DMA channel, from \ref DMA_CHANNEL_0 to \ref DMA_CHANNEL_7 .
	\param	a
			Buffer A inside the DMA memory.
	\param	b
			Buffer B inside the DMA memory.
			If 0, Ping-pong mode is not enabled
	\param	buffers_size
			Size of memory buffers (in amount of int, i.e. 2 bytes)
	\param	buffer_build_mode
			DMA Buffer Build Mode, must be one of \ref adc_dma_buffer_build_mode .
	\param	callback
			User-specified function to call when a buffer filled. If 0, DMA interrupt is disabled
*/
void adc1_init_scan_dma(unsigned long inputs, int start_conversion_event, int sample_time, int dma_channel, void * a, void * b, unsigned buffers_size, int buffer_build_mode, dma_callback callback)
{
	ERROR_CHECK_RANGE(sample_time, 0, 31, ADC_ERROR_INVALID_SAMPLE_TIME);
	if (start_conversion_event != ADC_START_CONVERSION_MANUAL_CLEAR_SAMPLE_BIT &&
		start_conversion_event != ADC_START_CONVERSION_EXTERNAL_INT &&
		start_conversion_event != ADC_START_CONVERSION_TIMER_COMPARE  &&
		start_conversion_event != ADC_START_CONVERSION_MC_PWM &&
		start_conversion_event != ADC_START_CONVERSION_FROM_INTERNAL_COUNTER
	)
		ERROR(ADC_ERROR_INVALID_START_CONVERSION_EVENT, &start_conversion_event);
	
	adc1_disable();
	
	// configure I/O pins in digital or analogic
#ifdef _CSS16
	AD1PCFGH = ~((unsigned short)(inputs >> 16));
#endif
	AD1PCFGL = ~((unsigned short)(inputs));
	// configure scanning on enabled inputs
#ifdef _CSS16
	AD1CSSH = (unsigned short)(inputs >> 16);
#endif
	AD1CSSL = (unsigned short)(inputs);
	
	AD1CON1bits.ADSIDL = 0;		// Continue module operation in Idle mode
	AD1CON1bits.AD12B = 1;		// 12-bit, 1-channel ADC operation
	AD1CON1bits.FORM = 0;		// Integer (DOUT = 0000 dddd dddd dddd)
	AD1CON1bits.ASAM = 1;		// Sampling begins immediately after last conversion and SAMP bit is auto-set
	AD1CON1bits.SSRC = start_conversion_event;	// select event which starts conversion
	
	if (buffer_build_mode == ADC_DMA_SCATTER_GATHER)
	{
		unsigned adc_input_count = amount_of_bits_at_one(inputs);
		unsigned per_input_buffer_size = buffers_size / adc_input_count;
		if (per_input_buffer_size * adc_input_count != buffers_size)
			ERROR(ADC_ERROR_INVALID_BUFFER_SIZE_FOR_SCATTER_GATHER, &buffers_size);
		if (adc_input_count > 16)
			ERROR(ADC_ERROR_TOO_MANY_INPUTS_FOR_SCATTER_GATHER, &adc_input_count);
		dma_init_channel(
			dma_channel,
			DMA_INTERRUPT_SOURCE_ADC_1,
			DMA_SIZE_WORD,
			DMA_DIR_FROM_PERIPHERAL_TO_RAM,
			DMA_INTERRUPT_AT_FULL,
			DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL,
			DMA_ADDRESSING_PERIPHERAL_INDIRECT,
			b ? DMA_OPERATING_CONTINUOUS_PING_PONG : DMA_OPERATING_CONTINUOUS,
			a,
			b,
			(void*)&ADC1BUF0,
			buffers_size,
			callback
		);
		
		AD1CON4bits.DMABL = log_2(per_input_buffer_size);
		AD1CON2bits.SMPI = adc_input_count - 1;		// Increments the DMA address after completion of adc_input_count sample/conversion operation
	}
	else if (buffer_build_mode == ADC_DMA_CONVERSION_ORDER)
	{
		unsigned adc_input_count = amount_of_bits_at_one(inputs);
		/*	unsigned per_input_buffer_size = buffers_size / adc_input_count;

		 It's perfectly "Legal" to do it, but it's non-sens. It mean the 
		   ADC won't scan all the imputs ... 
		if (per_input_buffer_size * adc_input_count != buffers_size) 
			ERROR(ADC_ERROR_INVALID_BUFFER_SIZE_FOR_SCATTER_GATHER, &buffers_size);
		*/
		dma_init_channel(
			dma_channel,
			DMA_INTERRUPT_SOURCE_ADC_1,
			DMA_SIZE_WORD,
			DMA_DIR_FROM_PERIPHERAL_TO_RAM,
			DMA_INTERRUPT_AT_FULL,
			DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL,
			DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT,
			b ? DMA_OPERATING_CONTINUOUS_PING_PONG : DMA_OPERATING_CONTINUOUS,
			a,
			b,
			(void*)&ADC1BUF0,
			buffers_size,
			callback
		);
		
		AD1CON2bits.SMPI = adc_input_count - 1;		// Increments the DMA address after completion of all input scanning operation
	}
	else
	{
		ERROR(ADC_ERROR_INVALID_BUFFER_BUILD_MODE, &buffer_build_mode);
	}
	AD1CON1bits.ADDMABM = buffer_build_mode;
	
	AD1CON2bits.VCFG = 0;		// Avdd, Avss Converter Voltage References
	AD1CON2bits.CSCNA = 1;		// Scan inputs for CH0+ during Sample A bit
	AD1CON2bits.BUFM = 0;		// Always starts filling the buffer from the start address.
	AD1CON2bits.ALTS = 0;		// Always uses channel input selects for Sample A
	
	AD1CON3bits.ADRC = 1;		// ADC Internal RC Clock: TAD == ADC Internal RC Clock cycle
	AD1CON3bits.SAMC = sample_time;	// Auto Sample Time bits sample_time * TAD
	AD1CON3bits.ADCS = 0;		// ADC Convertion Clock selection bits (ADCS+1)*TCY=TAD
	
	AD1CON4bits.DMABL = 0;		// Allocates 1 word of DMA buffer to each analog input
	
	AD1CHS0bits.CH0NA = 0;		// Channel 0 negative input is VREFL
	AD1CHS0bits.CH0SA = 0;		// Channel 0 positive input is defaulted to AN0
	
	// Reset ADC 1 interrupt flag
	_AD1IF = 0;
	// Disable ADC 1 Interrupt, because we use DMA
	_AD1IE = 0;
	
	// Enable DMA channel
	dma_enable_channel(dma_channel);
	
	adc1_enable();
}


/**
	Enable ADC 2.
	
	The ADC must be previously initialized by calling adc1_init_simple() or adc1_init_scan_dma().
	The user should call this function only if she hase previously called adc1_disable() manually.
*/
void adc2_enable()
{
	int i;
	
	// Turn on ADC Module
	AD2CON1bits.ADON = 1;
	
	// Wait 20 us, at 40 MIPS == 800 instructions
	i = 800/3;
	while (--i)
		Nop();
}

/** Disable ADC 2. */
void adc2_disable()
{
	// Turn off ADC Module
	AD2CON1bits.ADON = 0;
}

/**
	Initialize and enable ADC2 for simple input conversion.
	
	The converter is put in 12 bits / single conversion mode and callback is called when conversion is completed.
	This function does not start any conversion. Call adc2_start_simple_conversion() to start a conversion.
	
	\param	callback
			Pointer to a function that will be called when conversion is completed.
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 7 (highest priority)
	\param	inputs
			Bitfield that specify which physical input to use (AN0..AN16).
			1 put pins in analogic, 0 in digital.
	\param	sample_time
			Sample time, from 0 to 31, in number of ADC Internal RC Clock cycle

*/
void adc2_init_simple(adc_simple_callback callback, int priority, unsigned int inputs, int sample_time)
{
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	ERROR_CHECK_RANGE(sample_time, 0, 31, ADC_ERROR_INVALID_SAMPLE_TIME);
	
	adc2_disable();
	
	// Setup callback
	ADC_Data[1].callback = callback;
	_AD2IP = priority;
	
	// configure I/O pins in digital or analogic
	AD2PCFGL = ~(inputs);
	// No input to scan
	AD2CSSL = 0x0000;
	
	AD2CON1bits.ADSIDL = 0;		// Continue module operation in Idle mode
	AD2CON1bits.AD12B = 1;		// 12-bit, 1-channel ADC operation
	AD2CON1bits.FORM = 0;		// Integer (DOUT = 0000 dddd dddd dddd)
	AD2CON1bits.SSRC = 7;		// Internal counter ends sampling and starts conversion (auto-convert)
	AD2CON1bits.ASAM = 0;		// Sampling begins when SAMP bit is set
	
	AD2CON2bits.VCFG = 0;		// Avdd, Avss Converter Voltage References
	AD2CON2bits.CSCNA = 0;		// Do not scan inputs
	AD2CON2bits.BUFM = 0;		// Always starts filling the buffer from the start address.
	AD2CON2bits.ALTS = 0;		// Always uses channel input selects for Sample A
	
	AD2CON3bits.ADRC = 1;		// ADC Internal RC Clock: TAD == ADC Internal RC Clock cycle
	AD2CON3bits.SAMC = sample_time;	// Auto Sample Time bits sample_time*TAD
	AD2CON3bits.ADCS = 0;		// ADC Convertion Clock selection bits (ADCS+1)*TCY=TAD
	
	AD2CHS0bits.CH0NA = 0;		// Channel 0 negative input is VREFL
	AD2CHS0bits.CH0SA = 0;		// Channel 0 positive input is AN0
	
	// Reset ADC 1 interrupt flag
	_AD2IF = 0;
	// Enable ADC 1 Interrupt
	_AD2IE = 1;
	
	adc2_enable();
}

/**
	Request a conversion on a specific channel.
	
	The user must call adc2_init_simple prior to this function.
	
	\param	channel
			physical input to convert (AN0..AN16)
*/
void adc2_start_simple_conversion(int channel)
{
	// If a conversion is already in progress, throw an error
	if (ADC_Data[1].simple_channel >= 0)
		ERROR(ADC_ERROR_CONVERSION_IN_PROGRESS, &ADC_Data[1].simple_channel)
	
	// Select channel
	AD2CHS0bits.CH0SA = channel;
	ADC_Data[1].simple_channel = channel;
	
	// Start sampling
	AD2CON1bits.SAMP = 1;
}

/**
	Initialize and enable ADC2 for input scanning conversion using DMA.
	
	The converter is put in 12 bits.
	
	\param	inputs
			Bitfield that specify which physical input to use (AN0..AN16).
			1 put pins in analogic, 0 in digital.
	\param	start_conversion_event
			Which event triggers stop of sampling and start of conversion.
			Muse be one of \ref adc_start_conversion_event
	\param	sample_time
			If start_conversion_event is \ref ADC_START_CONVERSION_FROM_INTERNAL_COUNTER : Sample time, from 0 to 31, in number of ADC Internal RC Clock cycle. Otherwise ignored.
	\param	dma_channel
			DMA channel, from \ref DMA_CHANNEL_0 to \ref DMA_CHANNEL_7 .
	\param	a
			Buffer A inside the DMA memory.
	\param	b
			Buffer B inside the DMA memory.
			If 0, Ping-pong mode is not enabled
	\param	buffers_size
			Size of memory buffers (in amount of int, i.e. 2 bytes)
	\param	buffer_build_mode
			DMA Buffer Build Mode, must be one of \ref adc_dma_buffer_build_mode .
	\param	callback
			User-specified function to call when a buffer filled. If 0, DMA interrupt is disabled
*/
void adc2_init_scan_dma(unsigned int inputs, int start_conversion_event, int sample_time, int dma_channel, void * a, void * b, unsigned buffers_size, int buffer_build_mode, dma_callback callback)
{
	ERROR_CHECK_RANGE(sample_time, 0, 31, ADC_ERROR_INVALID_SAMPLE_TIME);
	if (start_conversion_event != ADC_START_CONVERSION_MANUAL_CLEAR_SAMPLE_BIT &&
		start_conversion_event != ADC_START_CONVERSION_EXTERNAL_INT &&
		start_conversion_event != ADC_START_CONVERSION_TIMER_COMPARE  &&
		start_conversion_event != ADC_START_CONVERSION_MC_PWM &&
		start_conversion_event != ADC_START_CONVERSION_FROM_INTERNAL_COUNTER
	)
		ERROR(ADC_ERROR_INVALID_START_CONVERSION_EVENT, &start_conversion_event);
	
	adc2_disable();
	
	// configure I/O pins in digital or analogic
	AD2PCFGL = ~(inputs);
	AD2CSSL = inputs;
	
	AD2CON1bits.ADSIDL = 0;		// Continue module operation in Idle mode
	AD2CON1bits.AD12B = 1;		// 12-bit, 1-channel ADC operation
	AD2CON1bits.FORM = 0;		// Integer (DOUT = 0000 dddd dddd dddd)
	AD2CON1bits.ASAM = 1;		// Sampling begins immediately after last conversion and SAMP bit is auto-set
	AD2CON1bits.SSRC = start_conversion_event;	// select event which starts conversion
	
	if (buffer_build_mode == ADC_DMA_SCATTER_GATHER)
	{
		unsigned adc_input_count = amount_of_bits_at_one(inputs);
		unsigned per_input_buffer_size = buffers_size / adc_input_count;
		if (per_input_buffer_size * adc_input_count != buffers_size)
			ERROR(ADC_ERROR_INVALID_BUFFER_SIZE_FOR_SCATTER_GATHER, &buffers_size);
		if (adc_input_count > 16)
			ERROR(ADC_ERROR_TOO_MANY_INPUTS_FOR_SCATTER_GATHER, &adc_input_count);
		dma_init_channel(
			dma_channel,
			DMA_INTERRUPT_SOURCE_ADC_2,
			DMA_SIZE_WORD,
			DMA_DIR_FROM_PERIPHERAL_TO_RAM,
			DMA_INTERRUPT_AT_FULL,
			DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL,
			DMA_ADDRESSING_PERIPHERAL_INDIRECT,
			b ? DMA_OPERATING_CONTINUOUS_PING_PONG : DMA_OPERATING_CONTINUOUS,
			a,
			b,
			(void*)&ADC2BUF0,
			buffers_size,
			callback
		);
		
		AD2CON4bits.DMABL = log_2(per_input_buffer_size);
		AD2CON2bits.SMPI = adc_input_count - 1;		// Increments the DMA address after completion of adc_input_count sample/conversion operation
	}
	else if (buffer_build_mode == ADC_DMA_CONVERSION_ORDER)
	{
		unsigned adc_input_count = amount_of_bits_at_one(inputs);

		dma_init_channel(
			dma_channel,
			DMA_INTERRUPT_SOURCE_ADC_2,
			DMA_SIZE_WORD,
			DMA_DIR_FROM_PERIPHERAL_TO_RAM,
			DMA_INTERRUPT_AT_FULL,
			DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL,
			DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT,
			b ? DMA_OPERATING_CONTINUOUS_PING_PONG : DMA_OPERATING_CONTINUOUS,
			a,
			b,
			(void*)&ADC2BUF0,
			buffers_size,
			callback
		);
		
		AD2CON2bits.SMPI = adc_input_count - 1;		// Increments the DMA address after completion of all input scanning operation
	}
	else
	{
		ERROR(ADC_ERROR_INVALID_BUFFER_BUILD_MODE, &buffer_build_mode);
	}
	AD2CON1bits.ADDMABM = buffer_build_mode;
	
	AD2CON2bits.VCFG = 0;		// Avdd, Avss Converter Voltage References
	AD2CON2bits.CSCNA = 1;		// Scan inputs for CH0+ during Sample A bit
	AD2CON2bits.BUFM = 0;		// Always starts filling the buffer from the start address.
	AD2CON2bits.ALTS = 0;		// Always uses channel input selects for Sample A
	
	AD2CON3bits.ADRC = 1;		// ADC Internal RC Clock: TAD == ADC Internal RC Clock cycle
	AD2CON3bits.SAMC = sample_time;	// Auto Sample Time bits sample_time * TAD
	AD2CON3bits.ADCS = 0;		// ADC Convertion Clock selection bits (ADCS+1)*TCY=TAD
	
	AD2CON4bits.DMABL = 0;		// Allocates 1 word of DMA buffer to each analog input
	
	AD2CHS0bits.CH0NA = 0;		// Channel 0 negative input is VREFL
	AD2CHS0bits.CH0SA = 0;		// Channel 0 positive input is defaulted to AN0
	
	// Reset ADC 2 interrupt flag
	_AD2IF = 0;
	// Disable ADC 2 Interrupt, because we use DMA
	_AD2IE = 0;
	
	// Enable DMA channel
	dma_enable_channel(dma_channel);
	
	adc2_enable();
}
//--------------------------
// Interrupt service routine
//--------------------------

/**
	ADC 1 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR  _ADC1Interrupt(void)
{
	// Conversion is completed, mark it as such
	int channel = ADC_Data[0].simple_channel;
	ADC_Data[0].simple_channel = -1;
	
	// Call user-defined function with result of conversion
	ADC_Data[0].callback(channel, ADC1BUF0);
	
	// Clear ADC 1 interrupt flag
	_AD1IF = 0;
}

/**
	ADC 2 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR  _ADC2Interrupt(void)
{
	// Conversion is completed, mark it as such
	int channel = ADC_Data[1].simple_channel;
	ADC_Data[1].simple_channel = -1;
	
	// Call user-defined function with result of conversion
	ADC_Data[1].callback(channel, ADC2BUF0);
	
	// Clear ADC 2 interrupt flag
	_AD2IF = 0;
}

/*@}*/
