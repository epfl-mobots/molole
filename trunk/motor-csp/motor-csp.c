#include "motor-csp.h"
#include <string.h>
// use (lont)  __builtin_mulss(a,b) to perform int*int => long

// use (int) __builtin_divsd(a,b) to perfrom long/int => int
// CHECK OV bit after the division to know if an overflow occured



static void __attribute__((always_inline)) s_control(motor_csp_data *d) {
	int error;
	int error_d;
	long temp;
	int output;
	
	error = d->speed_t - *d->speed_m;
	error_d = error - d->last_error_s;
	d->last_error_s = error;
	
	d->integral_s += error;
	
	temp = __builtin_mulss(d->kp_s, error);
	temp += __builtin_mulss(d->kd_s, error_d);
	temp += d->integral_s * d->ki_s; // long * long is about 7-8 cycle, so it's OK
	
	if(d->scaler_s) {
		output = __builtin_divsd(temp, d->scaler_s);
		if(SR & 0x4) {
			// Overflows flag. Mean that 16 bits is not enough
			if(output > 0) 
				output = d->current_max;
			else
				output = d->current_min;
		}
	} else {
		output = (int) temp;
	}
	
	if(output >= d->current_max) {
		output = d->current_max;
		
		if(d->ki_s) {
			if(d->scaler_s)
				d->integral_s = __builtin_mulss(d->current_max, d->scaler_s) / d->ki_s - __builtin_mulss(d->kp_s, error);
			else
				d->integral_s =  d->current_max / d->ki_s - __builtin_mulss(d->kp_s, error);
		}
		
			
	} else if(output <= d->current_min) {
		output = d->current_min;
		if(d->ki_s) {
			if(d->scaler_s) 
				d->integral_s = __builtin_mulss(d->pwm_min, d->scaler_i) / d->ki_s - __builtin_mulss(d->kp_s, error);
			else
				d->integral_s = d->current_min / d->ki_s  - __builtin_mulss(d->kp_s, error);
		}
		
	} else if(d->sat_status & 0x1) {
		// Ok, the current controller is getting a too high value, stop incrementing accumulator and don't put a higher value
		if(output > d->current_t) {
			output = d->current_t;
			if(error > 0)
				d->integral_s -= error;
		}
	} else if(d->sat_status & 0x2) {
		// The current controller is getting a too low value, stop decrementing the accumulator and don't put a lower value
		if(output < d->current_t) {
			output = d->current_t;
			if(error < 0)
				d->integral_s -= error;
		}
	}
	
	d->current_t = output;
}

static void __attribute__((always_inline)) p_control_32(motor_csp_data *d) {
	long error;
	long error_d;
	long temp;
	int output;
	
	error = *((long *) d->position_t) - *((long *) d->position_m);
	error_d = error - d->last_error_p;
	d->last_error_p = error;
	
	temp = error * d->kp_p + error_d * d->kd_p;
	
	if(d->scaler_p) {
		output = __builtin_divsd(temp, d->scaler_p);
		if(SR & 0x4) {
			// Overflow flag. Mean that 16 bits is not enough
			if(output > 0) 
				output = d->speed_max;
			else
				output = d->speed_min;
		}
	} else {
		output = (int) temp;
	}
	
	if(output > d->speed_max)
		output = d->speed_max;
	if(output < d->speed_min)
		output = d->speed_min;
		
	d->speed_t = output;
}

static void __attribute__((always_inline)) p_control_16(motor_csp_data *d) {
	int error;
	int error_d;
	long temp;
	int output;
	
	error = *((int *) d->position_t) - *((int *) d->position_m);
	error_d = error -  ((int) d->last_error_p);
	d->last_error_p = (long) error;
	
	temp = __builtin_mulss(d->kp_p, error);
	temp += __builtin_mulss(d->kd_p, error_d);
	
	if(d->scaler_p) {
		output = __builtin_divsd(temp, d->scaler_p);
		if(SR & 0x4) {
			// Overflow flag. Mean that 16 bits is not enough
			if(output > 0) 
				output = d->speed_max+1;
			else
				output = d->speed_min-1;
		}
	} else {
		output = (int) temp;
	}
	
	if(output > d->speed_max)
		output = d->speed_max;
	if(output < d->speed_min)
		output = d->speed_min;
		
	d->speed_t = output;
}

/**
        Do a step of motor control.

        Execute the position PD, then speed PID, then current PI.
        The position and speed controllers are executed only if they are enabled and if the prescaler hit the period.
*/

void motor_csp_step(motor_csp_data * d) {
	int error;
	long temp;
	int output;
	if(++d->prescaler_c == d->prescaler_period) {
		d->prescaler_c = 0;
		
		d->enc_up();
		
		if(d->enable_p) {
			if(d->is_32bits)
				p_control_32(d);
			else
				p_control_16(d);
		}
		if(d->enable_s)
			s_control(d);
	}
	
	error = d->current_t - *d->current_m;
	
	d->integral_i += error;
	
	temp = __builtin_mulss(d->kp_i, error);
	
	temp += d->integral_i * d->ki_i; 
	
	if(d->scaler_i) {
		output = __builtin_divsd(temp, d->scaler_i);
		if(SR & 0x4) {
			// Overflow flag. Mean that 16 bits is not enough
			if(output > 0) 
				output = d->pwm_max;
			else
				output = d->pwm_min;
		}
	} else {
		output = (int) temp;
	}
	
	// ARW
	if(output >= d->pwm_max) {
		output = d->pwm_max;
		
		if(d->ki_i) {
			if(d->scaler_i) 
				d->integral_i = __builtin_mulss(d->pwm_max, d->scaler_i) / d->ki_i - __builtin_mulss(d->kp_i, error);
			else
				d->integral_i = (d->pwm_max / d->ki_i) - __builtin_mulss(d->kp_i, error);
		}
		
		d->sat_status |= 0x1;
		
	} else if(output <= d->pwm_min) {
		output = d->pwm_min;
		if(d->ki_i) {
			if(d->scaler_i) 
				d->integral_i = __builtin_mulss(d->pwm_min, d->scaler_i) / d->ki_i - __builtin_mulss(d->kp_i, error);
			else
				d->integral_i =  (d->pwm_min / d->ki_i) - __builtin_mulss(d->kp_i, error);
		}
		d->sat_status |= 0x2;
	} else
		d->sat_status &= ~0x3;
	
	d->pwm_output = output;

}


/**
        Initialize an user-provided motor module. Setup 32bits position controller.
*/


void motor_csp_init_32(motor_csp_data *d) {
	memset(d, 0, sizeof(motor_csp_data));
	d->is_32bits = 1;
}


/**
        Initialize an user-provided motor module. Setup 16bits position controller.
*/


void motor_csp_init_16(motor_csp_data *d) {
	memset(d, 0, sizeof(motor_csp_data));
}
