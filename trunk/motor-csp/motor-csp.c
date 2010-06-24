#include "motor-csp.h"
#include <string.h>
// use (long)  __builtin_mulss(a,b) to perform int*int => long

// use (int) __builtin_divsd(a,b) to perfrom long/int => int
// CHECK OV bit after the division to know if an overflow occured


// 32bits / 16 bits => 32 bits implemented with two 32/16 => 16
unsigned long div32by16u(unsigned long a, unsigned int b) {
	// Take about 60 cycles
	unsigned int q1, rem1, q2;
	
	q1 = __builtin_divmodud(a>>16, b, &rem1);
	q2 = __builtin_divud((a & 0xffff) | (((unsigned long) rem1) << 16),b);
	return (((unsigned long) q1) << 16)| q2;
}

long div32by16s(long a, int b) {
	unsigned long ret;
	unsigned long aa;
	unsigned int ab;
	int sgn;
	
	if(a < 0) {
		sgn = 1;
		aa = -a;
	} else {
		sgn = -1;
		aa = a;
	}
		
	if(b < 0) {
		sgn += 1;
		ab = -b;
	} else {
		sgn -= 1;
		ab = b;
	}		
	
	ret = div32by16u(aa, ab);
	
	if(!sgn)
		return - ((long) ret);
	else
		return ret;
}


static void __attribute__((always_inline)) s_control(motor_csp_data *d) {
	int error;
	int error_d;
	long temp;
	int output;
	int do_arw = 0;
	
	if(d->speed_max || d->speed_min) {
		if(d->speed_t > d->speed_max)
			d->speed_t = d->speed_max;
		if(d->speed_t < d->speed_min)
			d->speed_t = d->speed_min;
	}
	
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
			if(temp > 0) 
				output = d->current_max;
			else
				output = d->current_min;
		}
	} else {
		if(temp > 32767)
			output = d->current_max;
		else if(temp < -32768)
			output = d->current_min;
		else
			output = (int) temp;
	}
	if(d->_over_status) {
		if(output > d->current_nominal) {
			output = d->current_nominal;
			do_arw = 1;
		} 
		if(output < -d->current_nominal) {
			output = -d->current_nominal;
			do_arw = 1;
		}
	} else {
		if(output > d->current_max) {
			output = d->current_max;
			do_arw = 1;
		} 
		if(output < d->current_min) {
			output = d->current_min;
			do_arw = 1;
		}
	}
	
	if(do_arw && d->ki_s) {
		if(d->scaler_s)
			d->integral_s = div32by16s(__builtin_mulss(output, d->scaler_s)  - __builtin_mulss(d->kp_s, error) - __builtin_mulss(d->kd_s, error_d), d->ki_s);
		else
			d->integral_s =  div32by16s(output - __builtin_mulss(d->kp_s, error) - __builtin_mulss(d->kd_s, error_d), d->ki_s);
	} else if(d->sat_status & 0x1) {
		// Ok, the current controller is getting a too high value, stop incrementing accumulator and don't put a higher value
		if(output > d->current_t) {
			output = d->current_t;
			d->integral_s -= error;
		}
	} else if(d->sat_status & 0x2) {
		// The current controller is getting a too low value, stop decrementing the accumulator and don't put a lower value
		if(output < d->current_t) {
			output = d->current_t;
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
			if(temp > 0) 
				output = d->speed_max;
			else
				output = d->speed_min;
		}
	} else {
		if(temp > 32767)
			output = d->speed_max;
		else if(temp < -32768)
			output = d->speed_min;
		else 
			output = (int) temp;
	}
		
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
			if(temp > 0) 
				output = d->speed_max;
			else
				output = d->speed_min;
		}
	} else {
		if (temp > 32767)
			output = d->speed_max;
		else if(temp < -32768)
			output = d->speed_min;
		else
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
        The speed control take about 600 cycles worst-case (mean when ARW code is executing).
        The position control should add a ~100 cycles.
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
	
	if(d->_over_status) {
		if(d->current_t > d->current_nominal)
			d->current_t = d->current_nominal;
			
		if(d->current_t < -d->current_nominal)
			d->current_t = -d->current_nominal;
	} else {
		if(d->current_t > d->current_max)
			d->current_t = d->current_max;
	
		if(d->current_t < d->current_min)
			d->current_t = d->current_min;
	}
	
	if(d->current_nominal && d->time_cst) {
		if(d->_iir_counter++ == 127) {
			d->_iir_counter = 0;
			d->iir_sum >>= 7;
			
			// It cannot overflow.
			d->square_c_iir = div32by16u(d->square_c_iir * d->time_cst + d->iir_sum, d->time_cst + 1);
			if(d->_over_status) {
				int tp_c_n = d->current_nominal - (d->current_nominal >> 3);
				if(d->square_c_iir < (__builtin_mulss(tp_c_n, tp_c_n) >> 2)) {
					d->_over_status = 0;
					if(d->ov_up) 
						d->ov_up(MOTOR_CSP_OVERCURRENT_CLEARED);
				}
			} else {
				if(d->square_c_iir > (__builtin_mulss(d->current_nominal, d->current_nominal) >> 2)) {
					// overcurrent on the motor
					d->_over_status = 1;
					if(d->ov_up) 
						d->ov_up(MOTOR_CSP_OVERCURRENT_ACTIVE);
				}
			}
			d->iir_sum = 0;
		} else 
			d->iir_sum += __builtin_mulss(*d->current_m, *d->current_m) >> 2; // To avoid overflow
		
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
				d->integral_i = div32by16s(__builtin_mulss(d->pwm_max, d->scaler_i)  - __builtin_mulss(d->kp_i, error), d->ki_i);
			else
				d->integral_i = div32by16s(d->pwm_max - __builtin_mulss(d->kp_i, error), d->ki_i);
		}
		
		d->sat_status = 0x1;
		
	} else if(output <= d->pwm_min) {
		output = d->pwm_min;
		if(d->ki_i) {
			if(d->scaler_i) 
				d->integral_i = div32by16s(__builtin_mulss(d->pwm_min, d->scaler_i) - __builtin_mulss(d->kp_i, error), d->ki_i);
			else
				d->integral_i =  div32by16s(d->pwm_min  - __builtin_mulss(d->kp_i, error), d->ki_i);
		}
		d->sat_status = 0x2;
	} else
		d->sat_status = 0;
	
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
