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

#ifndef _MOLOLE_PWM_H
#define _MOLOLE_PWM_H

/** \addtogroup pwm */
/*@{*/

/** \file
	PWM wrapper definitions
*/

/** Errors PWM can throw */
enum pwm_errors
{
	PWM_ERROR_BASE = 0x0300,
	PWM_ERROR_INVALIDE_PWM_ID,			/**< The desired PWM does not exists. */
	PWM_ERROR_INVALIDE_PRESCALER,		/**< The specified prescaler value is invalid. */
	PWM_ERROR_INVALIDE_MODE,			/**< The specified time base mode is invalid. */
};

/** Valid prescaler values */
enum pwm_prescaler_values
{
	PWM_PRESCALER_1 = 0,
	PWM_PRESCALER_4,
	PWM_PRESCALER_16,
	PWM_PRESCALER_64,
};

/** Available PWM time base modes */
enum pwm_time_base_modes
{
	PWM_MODE_FREE_RUNNING = 0,			/**< PWM time base operates in a continuous Up/Down Counting mode */
	PWM_MODE_SINGLE_EVENT,				/**< PWM time base operates in Single Event mode */
	PWM_CONTINUOUS_UP_DOWN, 			/**< PWM time base operates in Free Running mode */
	PWM_CONTINUOUS_UP_DOWN_DOUBLE		/**< PWM time base operates in a continuous Up/Down mode with interrupts for double PWM updates */
};


// Functions, doc in the .c

void pwm_init(int prescaler, unsigned period, int mode);
//void pwm_enable_interrupt(pwm_callback callback, int postscaler, int priority);
//void pwm_disable_interrupt();
void pwm_enable(int pwm_id);
void pwm_disable(int pwm_id);
void pwm_set_duty(int pwm_id, unsigned duty);


/*@}*/

#endif

