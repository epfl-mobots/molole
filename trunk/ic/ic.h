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

#ifndef _MOLOLE_IC_H
#define _MOLOLE_IC_H

/** \addtogroup ic */
/*@{*/

/** \file
	Input Capture wrapper definitions
*/

// Defines

/** Errors Input Capture can throw */
enum ic_errors
{
	IC_ERROR_BASE = 0x0700,
	IC_ERROR_INVALID_IC_ID,				/**< The desired Input Capture does not exists, must be one of \ref ic_identifiers. */
	IC_ERROR_INVALID_TIMER_SOURCE,		/**< The specified timer source is invalid, must be one of \ref ic_timer_source. */
	IC_ERROR_INVALID_MODE,				/**< The specified mode is invalid, must be one of \ref ic_modes excepted \ref IC_DISABLED. */
};

/** Identifiers of available Input Capture. */
enum ic_identifiers
{
	IC_1 = 0,							/**< Input Capture 1 */
	IC_2,								/**< Input Capture 2 */
	IC_3,								/**< Input Capture 3 */
	IC_4,								/**< Input Capture 4 */
	IC_5,								/**< Input Capture 5 */
	IC_6,								/**< Input Capture 6 */
	IC_7,								/**< Input Capture 7 */
	IC_8,								/**< Input Capture 8 */
};

/** Possible Input Capture timer sources */
enum ic_timer_source
{
	IC_TIMER3 = 0,						/**< Input Capture timer source is timer 3 */
	IC_TIMER2							/**< Input Capture timer source is timer 2 */
};

/** Available Input Capture modes */
enum ic_modes
{
	IC_DISABLED = 0,					/**< The specified Input Capture is disabled. This is not a valid mode to set, call ic_disable() instead.  */
	IC_EDGE_CAPTURE,					/**< Capture on both rising and falling edge of the capture input signal. */
	IC_FALLING_EDGE,					/**< Capture on the falling edge of the capture input signal. */
	IC_RISING_EDGE,						/**< Capture on the rising edge of the capture input signal. */
	IC_EACH_4_RISING_EDGE,				/**< Capture on every 4th rising edge of the capture input signal. */
	IC_EACH_16_RISING_EDGE,				/**< Capture on every 16th rising edge of the capture input signal. */
	IC_WAKEUP = 7						/**< No capture event, only wake-up processor from sleep/idle. */
};

/** Input Capture callback on interrupt, with the value of the timer at that moment */
typedef void (*ic_callback)(int ic_id, unsigned int value, void* user_data);

// Functions, doc in the .c

void ic_enable(int ic_id, int source, int mode, ic_callback callback, int priority, void* user_data);

void ic_disable(int ic_id);

/*@}*/

#endif

