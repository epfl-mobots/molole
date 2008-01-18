/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs
	
	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>,
	Mobots group (http://mobots.epfl.ch), Robotics system laboratory (http://lsro.epfl.ch)
	EPFL Ecole polytechnique federale de Lausanne (http://www.epfl.ch)
	
	Copyright (C) 2007 Florian Vaussard <Florian dot Vaussard at a3 dot epfl dot ch>
	
	Copyright (C) 2004 Daniel Baer, Autonomous Systems Lab, EPFL
	
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

/** PWM wrapper data */
static struct
{
	pwm_callback interrupt_callback; /**< function to call upon PWM interrupt */
} PWM_Data = { 0 };

/**
	Init the PWM subsystem.
	
	\param	prescaler
			Prescaler of FCY. Must be one of \ref pwm_prescaler_values.
	\param	period
			PWM period (0..32767)
	\param	mode
			PWM time base modes. Must be one of \ref pwm_time_base_modes.
*/
void pwm_init(int prescaler, unsigned period, int mode)
{
	ERROR_CHECK_RANGE(prescaler, 0, 3, PWM_ERROR_INVALID_PRESCALER);
	ERROR_CHECK_RANGE(period, 0, 32767, PWM_ERROR_INVALID_RANGE);
	ERROR_CHECK_RANGE(mode, 0, 3, PWM_ERROR_INVALID_MODE);
	
	PTPER = period;						// PWM period
	PTCONbits.PTCKPS = prescaler;		// PWM Time Base Input Clock Prescale
	PTCONbits.PTMOD = mode;				// PWM Time Base operates in continuous up/down counting mode
	PTCONbits.PTSIDL = 0;				// PWM time base runs in CPU Idle mode
	PTCONbits.PTEN = 1;					// Enable PWM Time Base Timer
}

/**
	Enable the PWM interrupt
	
	\param	postscale
			The conversion is started each (postscale+1) (parameter is 0..15, corresponding to a 1:1 to 1:16 postscale)
	\param	callback
			Pointer to a function that will be called upon interrupt
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 7 (highest priority)
*/
void pwm_enable_interrupt(int postscaler, pwm_callback callback, int priority)
{
	ERROR_CHECK_RANGE(postscaler, 0, 15, PWM_ERROR_INVALID_POSTSCALER);
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	
	PTCONbits.PTOPS = postscaler;		// PWM Time Base Output Postscale
	
	PWM_Data.interrupt_callback = callback;
	_PWMIF = 0;							// clear the PWM interrupt
	_PWMIP = priority;   				// set the PWM interrupt priority
	_PWMIE = 1;							// enable the PWM interrupt
}

/**
	Disable the PWM interrupt
*/
void pwm_disable_interrupt()
{
	_PWMIE = 0;							// disable the PWM interrupt
	_PWMIF = 0;							// clear the PWM interrupt
}

/**
	Enables a PWM.
	
	\param	pwm_id
			Identifier of the PWM, from \ref PWM_1 to \ref PWM_4.
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
		default: ERROR(PWM_ERROR_INVALID_PWM_ID, &pwm_id);
	}
}

/**
	Disables a PWM.
	
	\param	pwm_id
			Identifier of the PWM, from \ref PWM_1 to \ref PWM_4.
*/
void pwm_disable(int pwm_id)
{
	// L and H pins are to GPIO
	switch (pwm_id)
	{
		case 0: PWMCON1bits.PEN1L = 0; PWMCON1bits.PEN1H = 0; break;
		case 1: PWMCON1bits.PEN2L = 0; PWMCON1bits.PEN2H = 0; break;
		case 2: PWMCON1bits.PEN3L = 0; PWMCON1bits.PEN3H = 0; break;
		case 3: PWMCON1bits.PEN4L = 0; PWMCON1bits.PEN4H = 0; break;
		default: ERROR(PWM_ERROR_INVALID_PWM_ID, &pwm_id);
	}
}

/**
	Set the duty of a PWM.
	
	\param	pwm_id
			Identifier of the PWM, from \ref PWM_1 to \ref PWM_4.
	\param	duty
			Duty cycle (0..32767)
*/

void pwm_set_duty(int pwm_id, unsigned duty)
{
	if (duty > 32676)
		ERROR(PWM_ERROR_INVALID_RANGE, &duty);
		
	switch (pwm_id)
	{
		case 0: PDC1 = duty; break;
		case 1: PDC2 = duty; break;
		case 2: PDC3 = duty; break;
		case 3: PDC4 = duty; break;
		default: ERROR(PWM_ERROR_INVALID_PWM_ID, &pwm_id);
	}
}

/**
	Setup analog-to-digital conversion (adc) synchronization with PWM.
	
	When PWM reaches a specific value and direction, a counter is incremented.
	When this counter reaches (postscale+1), the adc starts.
	
	This function does not perform adc setup.
	
	\param	direction
			On which direction of counting to compare, \ref PWM_SEV_UP or \ref PWM_SEV_DOWN
	\param	postscale
			The conversion is started each (postscale+1) (parameter is 0..15, corresponding to a 1:1 to 1:16 postscale)
	\param	value
			Value to compare (0.. 32767)
*/
void pwm_set_special_event_trigger(int direction, int postscale, unsigned value)
{
	ERROR_CHECK_RANGE(direction, 0, 1, PWM_ERROR_INVALID_SEV_DIRECTION);
	ERROR_CHECK_RANGE(postscale, 0, 15, PWM_ERROR_INVALID_SEV_POSTSCALE);
	ERROR_CHECK_RANGE(value, 0, 32767, PWM_ERROR_INVALID_RANGE);
	
	SEVTCMPbits.SEVTDIR = direction;
	SEVTCMPbits.SEVTCMP = value;
	PWMCON2bits.SEVOPS = postscale;
	
	// TODO: set or check ADC, for instance: AD1CON1bits.SSRC
}


//--------------------------
// Interrupt service routine
//--------------------------

/**
	PWM Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _PWMInterrupt(void)
{
	PWM_Data.interrupt_callback();
	
	_PWMIF = 0;
}

// TODO: In Florient's code, this interrupt used the _ISRFAST flag

/*@}*/
