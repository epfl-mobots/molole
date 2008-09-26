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
	\defgroup motor Motor
	
	Generic motor module.
	
	To use this module, you must declare a struct of type Motor_Controller_Data
	and initialize it with calling motor_init().
	Then, you have to setup some fields by hand. The mandatory fields are:
	- setpoint
	- measure
	- kp
	Then, you have to repeateadely call motor_step() to update the value of output,
	which you can use to drive a physical output.
*/
/*@{*/

/** \file
	Implementation of the generic motor module.
*/


//------------
// Definitions
//------------

#include <p33fxxxx.h>
#include <string.h>
#include <limits.h>
#include "motor.h"


//-------------------
// Exported functions
//-------------------

/**
	Initialize a user-provided motor module.
	
	All values are initialized to zero, excepted limits that are initialized to maximum integer range.
*/
void motor_init(Motor_Controller_Data* module)
{
	memset(module, 0, sizeof(Motor_Controller_Data));
	
	module->setpoint_limit_low = INT_MIN;
	module->setpoint_limit_high = INT_MAX;
	
	module->constraint_limit_low = INT_MIN;
	module->constraint_limit_high = INT_MAX;
	
	module->output_limit_low = INT_MIN;
	module->output_limit_high = INT_MAX;
}

void motor_init_32bits(Motor_Controller_Data* module)
{
	memset(module, 0, sizeof(Motor_Controller_Data));
	
	module->setpoint_limit_low = LONG_MIN;
	module->setpoint_limit_high = LONG_MAX;
	
	module->constraint_limit_low = INT_MIN;
	module->constraint_limit_high = INT_MAX;
	
	module->output_limit_low = INT_MIN;
	module->output_limit_high = INT_MAX;
	
	module->is_32bits = true;
		
}

/**
	Do a step of motor control.
	
	This consists of a PID controller and an external constraint check.
*/
void motor_step(Motor_Controller_Data* module)
{
	// check setpoint limit
	long setpoint;
	long measure;

	if(module->is_32bits) {
		measure = *((long *) module->measure);
		setpoint = *((long *)module->setpoint);
	} else {
		setpoint = *((int *)module->setpoint);
		measure = *((int *) module->measure);
	}
	
	if (setpoint > module->setpoint_limit_high)
	{
		setpoint = module->setpoint_limit_high;
	}
	else if (setpoint < module->setpoint_limit_low)
	{
		setpoint = module->setpoint_limit_low;
	}
	
	// compute terms
	long error = setpoint - measure;
	long proportional_term = module->kp * error;
	long integral_term = module->ki * error + module->last_integral_term;
	long derivative_term = module->kd * (error - module->last_error);
	long output = (proportional_term + integral_term  + derivative_term) >> (long)module->output_shift_factor;
	
	// antireset windup
	if (output > module->output_limit_high)
	{
		// recompute integral term
		if(module->ki)
			integral_term = (module->output_limit_high << (long)module->output_shift_factor) - proportional_term - derivative_term;
		
		// crop output
		output = module->output_limit_high;
	}
	else if (output < module->output_limit_low)
	{
		// recompute integral term
		if(module->ki)
			integral_term = (module->output_limit_low << (long)module->output_shift_factor) - proportional_term - derivative_term;
		
		// crop output
		output = module->output_limit_low;
	}
	
	// store terms for next iteration
	module->output = output;
	module->last_error = error;
	module->last_integral_term = integral_term;
	module->last_derivative_term = derivative_term;
	
	// check external constraint
	if (module->constraint)
	{
		if (*(module->constraint) > module->constraint_limit_high)
		{
			module->output = module->constraint_callback(MOTOR_CONSTRAINT_OVERRUN, module->output);
		}
		else if (*(module->constraint) < module->constraint_limit_low)
		{
			module->output = module->constraint_callback(MOTOR_CONSTRAINT_UNDERRUN, module->output);
		}
	}
	
	// Poor's man high-pass filter on integral term
	if (module->forgetness)
	{
		module->forgetness_counter++;
		if(module->forgetness_counter == module->forgetness)
		{
			module->forgetness_counter = 0;
			if(module->last_integral_term > 0)
				module->last_integral_term--;
			else if(module->last_integral_term < 0) 
				module->last_integral_term++;
		}
	}
}

/*@}*/
