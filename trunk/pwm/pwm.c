/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>
	Copyright (C) 2007 Florian Vaussard <Florian dot Vaussard at a3 dot epfl dot ch>
	Copyright (C) 2004 Daniel Baer
	
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

//--------------------
// Usage documentation
//--------------------

/**
	\defgroup pwm PWM
	
	Wrapper around PWM, with a callback oriented interface.
*/
/*@{*/

/** \file
	Implementation of the wrapper around PWM.
*/


//------------
// Definitions
//------------

#include <p33fxxxx.h>

#include "pwm.h"
#include "../error/error.h"

// prescaler 1:1, 1:4, 1:16, 1:64
void pwm_init(int prescaler, unsigned period, int mode)
{
	if (mode > 3)
		ERROR(PWM_ERROR_INVALIDE_MODE, &mode);
	
	if (prescaler > 3)
		ERROR(PWM_ERROR_INVALIDE_PRESCALER, &prescaler);
	
	PTPER = period;						// PWM period
	PTCONbits.PTCKPS = prescaler;		// PWM Time Base Input Clock Prescale
	PTCONbits.PTMOD = mode;				// PWM Time Base operates in continuous up/down counting mode
	PTCONbits.PTSIDL = 0;				// PWM time base runs in CPU Idle mode
	PTCONbits.PTEN = 1;					// Enable PWM Time Base Timer
}

// TODO: implement timer interface
// 	// Trig the ADC with the special event
// 	SEVTCMP = PTPER;			// Conversion in the middle of the high (or low) side of the PWM (for the Up/Down counting mode)
// 	PWMCON2bits.SEVOPS = 0xF;	// Special Event Trigger Postscaler = 16 -> don't useful to trig the ADC too fast !


/*
void pwm_enable_interrupt(pwm_callback callback, int postscaler, int priority)
{
	// TODO: check invalid postscaler
	PTCONbits.PTOPS = postscaler;		// PWM Time Base Output Postscale
	// TODO: implement
}

void pwm_disable_interrupt()
{
	// TODO: implement
// 	_PWMIP = PWM_IP;			// Set Interrupt Priority
// 	_PWMIF = 0;					// Clear PWM Interrupt flag	
// 	_PWMIE = 0;					// Disable PWM interrupts.
}
*/

void pwm_enable(int pwm_id)
{
	// L and H pins are set to PWM in complimentary output
	switch (pwm_id)
	{
		case 0: PWMCON1bits.PEN1L = 1; PWMCON1bits.PEN1H = 1; PWMCON1bits.PMOD1 = 0; break;
		case 1: PWMCON1bits.PEN2L = 1; PWMCON1bits.PEN2H = 1; PWMCON1bits.PMOD2 = 0; break;
		case 2: PWMCON1bits.PEN3L = 1; PWMCON1bits.PEN3H = 1; PWMCON1bits.PMOD3 = 0; break;
		case 3: PWMCON1bits.PEN4L = 1; PWMCON1bits.PEN4H = 1; PWMCON1bits.PMOD4 = 0; break;
		default: ERROR(PWM_ERROR_INVALIDE_PWM_ID, &pwm_id);
	}
}

void pwm_disable(int pwm_id)
{
	// L and H pins are to GPIO
	switch (pwm_id)
	{
		case 0: PWMCON1bits.PEN1L = 0; PWMCON1bits.PEN1H = 0; break;
		case 1: PWMCON1bits.PEN2L = 0; PWMCON1bits.PEN2H = 0; break;
		case 2: PWMCON1bits.PEN3L = 0; PWMCON1bits.PEN3H = 0; break;
		case 3: PWMCON1bits.PEN4L = 0; PWMCON1bits.PEN4H = 0; break;
		default: ERROR(PWM_ERROR_INVALIDE_PWM_ID, &pwm_id);
	}
}

void pwm_set_duty(int pwm_id, unsigned duty)
{
	switch (pwm_id)
	{
		case 0: PDC1 = duty; break;
		case 1: PDC2 = duty; break;
		case 2: PDC3 = duty; break;
		case 3: PDC4 = duty; break;
		default: ERROR(PWM_ERROR_INVALIDE_PWM_ID, &pwm_id);
	}
}

/*@}*/