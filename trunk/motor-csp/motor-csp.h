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

#ifndef _MOLOLE_MOTOR_CSP_H
#define _MOLOLE_MOTOR_CSP_H

#include "../types/types.h"

/** \addtogroup motor_csp */ 
/*@{*/

/** \file
        \brief A nested current, speed and position motor module.
*/

/** External callback to be called before the position or speed controller execute */
typedef void (*motor_csp_enc_cb)(void);

#define MOTOR_CSP_OVERCURRENT_ACTIVE 0
#define MOTOR_CSP_OVERCURRENT_CLEARED 1	
/** External callback to be called when overcurrent status change */
typedef void (*motor_csp_overcurrent) (int status);

typedef struct {
// Current PI part
	int *current_m; 				//! Current mesure
	int current_t;					//! Current target, automatically set if enable_s is true
	int kp_i;						//! KP value for current, must be >= 0
	int ki_i;						//! KI value for current, must be >= 0
	int scaler_i;					//! Scale factor for pwm output. 0 mean scaling disabled, must be >= 0
	long integral_i;				//! Integral term for current, internal use only.
	int pwm_min;					//! Minimum PWM value
	int pwm_max;					//! Maximum PWM value
	int pwm_output;					//! PWM output value
	int current_max;				//! Maximum current for speed PID output and current PI input
	int current_min;				//! Minimum current for speed PID output and current PI input
	unsigned char time_cst;			//! Motor winding time constant for heat dissipation in 128*current period
	int current_nominal;			//! Max DC current for the motor
	unsigned long square_c_iir;		//! Square current IIR filter
	unsigned long iir_sum;			//! Sum for the mean
	unsigned char _iir_counter;		//! IIR counter for mean
	unsigned char _over_status;		//! overcurrent internal status
	
	unsigned int prescaler_period;	//! Prescaler value for speed and position controller
	unsigned int prescaler_c;		//! Counter for prescaler, internal use only.
	
// 	Speed PID part
	int *speed_m;					//! Speed mesure
	int speed_t;					//! Speed target, automatically set if enable_p is true
	int kp_s;						//! KP value for speed, must be >= 0
	int ki_s;						//! KI value for speed, must be >= 0
	int kd_s;						//! KD value for speed, must be >= 0
	int scaler_s;					//! Scale factor for current output.  0 mean scaling disabled, must be >= 0
	long integral_s;				//! Integral value for speed, internal use only
	bool enable_s;					//! Enable speed PID
	int last_error_s;				//! Last speed error (for D term)
	
//	Position part
	void *position_m;				//! Position mesure
	void *position_t;				//! Position target
	int kp_p;						//! KP value for position, must be >= 0
	int kd_p;						//! KD value for position, must be >= 0
	int scaler_p;					//! Scale factor for speed output. 0 mean scaling disabled, must be >= 0
	bool enable_p;					//! Enable position PID
	int speed_max;					//! Maximum speed for position PD output
	int speed_min;					//! Minimum speed for position PD output
	long last_error_p;				//! Last position error (for D term)
	bool is_32bits;					//! True if the position is 32bits, false if it's 16bits
	
	motor_csp_enc_cb enc_up;		//! Encoder update callback pointer
	motor_csp_overcurrent ov_up;	//! Motor overcurrent callback pointer
	
	int sat_status;					//! 2bit-field of current controller saturation status, internal use only
} motor_csp_data;

// init 32bit, init 16bits

void motor_csp_step(motor_csp_data * d);
void motor_csp_init_32(motor_csp_data *d);
void motor_csp_init_16(motor_csp_data *d);

#endif

