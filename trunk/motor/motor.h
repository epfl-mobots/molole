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

#ifndef _MOLOLE_MOTOR_H
#define _MOLOLE_MOTOR_H

#include "../types/types.h"

/** \addtogroup motor */
/*@{*/

/** \file
	\brief A generic motor module.
*/

// Defines

/** Type of constraint violation that can happen */
enum motor_constraint_violation_type
{
	MOTOR_CONSTRAINT_OVERRUN = 0,		/**< the actual value of a constrained variable is over the specified threshold */
	MOTOR_CONSTRAINT_UNDERRUN = 1,		/**< the actual value of a constrained variable is below the specified threshold */
};

/** Motor callback on constraint violation, argument is one of motor_constraint_violation_type and the current output; this function must return the new output. */
typedef int (*motor_constraint_violation_callback)(int violation_type, int output);

// Structures definitions

/** Data associated with a motor controller. */
typedef struct
{
	int* setpoint;						//!< pointer to a variable containing the setpoint
	int setpoint_limit_low;				//!< minimum acceptable value for setpoint
	int setpoint_limit_high;			//!< maximum acceptable value for setpoint
	int* measure;						//!< pointer to a variable containing the latest measure
	
	int* constraint;					//!< pointer to a variable containing a constraint; if 0, constraint is disabled
	int constraint_limit_low;			//!< minimum acceptable value for constraint
	int constraint_limit_high;			//!< maximum acceptable value for constraint
	motor_constraint_violation_callback constraint_callback; //!< user function to call on constraint violation
	
	int output_shift_factor;			//!< factor by which the sum of P, I, and D terms are shifted to produce output
	int output_limit_low;				//!< minimum acceptable value for output
	int output_limit_high;				//!< maximum acceptable value for output
	int output;							//!< last output value computed on motor_step(), 0 on init
	
	long kp;							//!< PID proportional gain
	long ki;							//!< PID integral gain
	long kd;							//!< PID derivative gain
	
	long last_error;					//!< last error
	long last_integral_term;			//!< last integral term
	long last_derivative_term;			//!< last derivative term
	
	unsigned int forgetness_counter;	//!< Amount of step since last forgetness action
	unsigned int forgetness;			//!< amount of step to decrease one unity of integral term. 0 mean disabled. (high-pass filter)
} Motor_Controller_Data;

// Functions, doc in the .c

void motor_init(Motor_Controller_Data* module);

void motor_step(Motor_Controller_Data* module);

/*@}*/

#endif
