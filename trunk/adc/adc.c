/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	Copyright (C) 2006 - 2007 Florian Vaussard <Florian dot Vaussard at a3 dot epfl dot ch>
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
	\brief Implementation of the wrapper around dsPIC33 ADC.
*/

#include <p33fxxxx.h>

#include "adc.h"

static struct
{
	adc_simple_callback callback; /**< function to call upon conversion complete interrupt, 0 if none */
} ADC_Data = { 0 };


void adc1_init_simple(adc_simple_callback callback, unsigned long ports)
{
	// Turn off ADC Module
	AD1CON1bits.ADON = 0;
	
	// Setup callback
	ADC_Data.callback = callback;
	
	// configure I/O pins in digital or analogic
	AD1PCFGH = ~((unsigned short)(ports >> 16));
	AD1PCFGL = ~((unsigned short)(ports));
	// ADC is off
	// Discontinue module operation when device enters Idle mode
	// DMA conf bits ignored
	// 12-bit, 1-channel ADC operation
	// Integer (DOUT = 0000 dddd dddd dddd)
	// Internal counter ends sampling and starts conversion (auto-convert)
	// Sampling begins when SAMP bit is set
	AD1CON1 = 0x24E0;
	// Avdd, Avss Converter Voltage References
	// Do not scan inputs
	// Converts CH0
	// Always uses channel input selects for Sample A
	// Always starts filling the buffer from the start address.
	// DMA conf bits ignored
	AD1CON2 = 0x0000;
	// Clock Derived From System Clock
	// Auto Sample Time bits 0 TAD
	// ADC Conversion Clock Select bits 1 · TCY = TAD
	AD1CON3 = 0x0000;
	// No DMA, AD1CON4 ignored
	// No input to scan yet
	AD1CSSH = 0x0000;
	AD1CSSL = 0x0000;
	// Sample B bit ignored
	// Channel 0 negative input is VREFL
	// Channel 0 positive input is AN0
	AD1CHS0 = 0x0000;
	
	// Reset ADC 1 interrupt flag
	_AD1IF = 0;
	// Enable ADC 1 Interrupt
	_AD1IE = 1;
	
	// Turn on ADC Module
	AD1CON1bits.ADON = 1;
}

void adc1_start_simple_conversion(int channel)
{
	// Select channel
	AD1CHS0 = channel;
	// Start sampling
	AD1CON1bits.ASAM = 1;
}

void _ISR  _ADC1Interrupt(void)
{
	// Call user-defined function with result of conversion
	ADC_Data.callback(ADC1BUF0);
	// Clear ADC 1 interrupt flag
	_AD1IF = 0;
} 

