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

#ifndef _MOLOLE_OC_H
#define _MOLOLE_OC_H

/** \addtogroup oc */
/*@{*/

/** \file
	Output Compare wrapper definitions
*/

// Defines

/** Errors Output Compare can throw */
enum oc_errors
{
	OC_ERROR_BASE = 0x0400,
	OC_ERROR_INVALID_OC_ID,				/**< The desired Output Compare does not exists, must be one of \ref oc_identifiers. */
	OC_ERROR_INVALID_TIMER_SOURCE,		/**< The specified timer source is invalid, must be \ref TIMER_2 or \ref TIMER_3. */
	OC_ERROR_INVALID_MODE,				/**< The specified mode is invalid, must be one of \ref oc_modes excepted \ref OC_DISABLED. */
};

/** Identifiers of available Output Compares. */
enum oc_identifiers
{
	OC_1 = 0,			/**< Output Compare 1 */
	OC_2,				/**< Output Compare 2 */
	OC_3,				/**< Output Compare 3 */
	OC_4,				/**< Output Compare 4 */
	OC_5,				/**< Output Compare 5 */
	OC_6,				/**< Output Compare 6 */
	OC_7,				/**< Output Compare 7 */
	OC_8,				/**< Output Compare 8 */
};


/** Available Output Compare modes */
enum oc_modes
{
	OC_DISABLED = 0,					/**< Module Disabled; Output Compare module is disabled; This is not a valid mode to set, call oc_disable() instead. */
	OC_ACTIVE_LOW_ONE_SHOT,				/**< Active Low One-Shot mode; Initialize OCx pin low, compare event forces OCx pin high. */
	OC_ACTIVE_HIGH_ONE_SHOT,			/**< Active High One-Shot mode; Initialize OCx pin high, compare event forces OCx pin low. */
	OC_TOGGLE,							/**< Toggle mode; Compare event toggles OCx pin. */
	OC_DELAYED_ONE_SHOT,				/**< Delayed One-Shot mode; Initialize OCx pin low, generate single output pulse on OCx pin. */
	OC_CONTINUOUS_PULSE,				/**< Continuous Pulse mode; Initialize OCx pin low, generate continuous output pulses on OCx pin. */
	OC_PWM_NO_FAULT_PROTECTION,			/**< PWM mode without fault protection; PWM mode on OCx, Fault pin is disabled. */
	OC_PWM_WITH_FAULT_PROTECTION		/**< PWM mode with fault protection; PWM mode on OCx, Fault pin is enabled */
};


typedef void (*oc_irq_cb)(int oc_id);


// Functions, doc in the .c

void oc_enable(int oc_id, int timer, int mode);

void oc_disable(int oc_id);

void oc_set_value(int oc_id, unsigned primary, unsigned secondary);

void oc_set_value_pwm(int oc_id, unsigned duty);

void oc_enable_interrupt(int oc_id, oc_irq_cb cb, int priority);

void oc_disable_interrupt(int oc_id);

/*@}*/

#endif

