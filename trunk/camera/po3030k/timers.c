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

/*! \file
 * \brief Manage camera's interrupts
 * \author Philippe Rétornaz
 * \verbinclude interrupt.s
 */

#include <p33fxxxx.h>
#include "po3030k.h"
#include "../../ic/ic.h"
#include "../../timer/timer.h"
#include "../../i2c/i2c.h"
#include "../../error/error.h"


/*! The buffer to write to */
char * _po3030k_buffer;

/*! Internal flag to tell that the pixel clock is half speed */
int _po3030k_slow_path; 

/*! The flag to tell, the image is ready or not
 * Zero mean capture is in progress, non-zero mean capture done.
 * \sa po3030k_is_img_ready
 */
int _po3030k_img_ready;

static int blank_row_betw;
static int timer_priority;
static int timer_id;
static int ic_id;

int _po3030k_current_row;
int _po3030k_row;
unsigned char * _po3030k_port;


char _po3030k_line_conf[330];

/* Yes, it's an ISR ... we goto() to it from the timer irq. */
void _ISR _po3030k_disable_hsync(void) {
	timer_disable(timer_id);
}

/*! \brief The VSYNC callback.
 * This callback is called every time the Vertical sync signal is asserted
 * This mean that the picture is coming from the camera ( we will have the first line soon )
 */
static void vsync_cb(int id, unsigned int __attribute__((unused)) value, void * __attribute__((unused)) data) {
	/* let's enable Hsync */
	timer_enable(timer_id);
	/* single shot */
	ic_disable(id);
}

static void init_hsync(void) {
	timer_set_clock_source(timer_id, TIMER_CLOCK_EXTERNAL);			
	timer_set_value(timer_id, blank_row_betw);
	timer_set_period(timer_id, blank_row_betw + 1, -1);	
	timer_enable_interrupt(timer_id, 0, timer_priority);
}


/*! Launch a capture in the \a buf buffer
 * \param buf The buffer to write to
 * \sa po3030k_config_cam and po3030k_is_img_ready
 */
void po3030k_launch_capture(char * buf) {
	_po3030k_current_row = 0;
	_po3030k_buffer = buf;
	_po3030k_img_ready = 0;
	init_hsync();
	/* VSync must always be initialised as the last one */
	ic_enable(ic_id, IC_TIMER3 /* DON'T CARE*/, IC_RISING_EDGE, vsync_cb, timer_priority -1, 0);
}

/*! Modify the interrupt configuration
 * \warning This is an internal function, use \a po3030k_config_cam
 * \param pixel_row The number of row to take
 * \param pixel_col The number of pixel to take each \a pixel_row
 * \param bpp The number of byte per pixel
 * \param pbp The number of pixel to ignore between each pixel
 * \param bbl The number of row to ignore between each line
 * \return Zero if OK, non-zero if the mode exceed internal data representation
 * \sa po3030k_get_bytes_per_pixel and po3030k_config_cam
 */
int po3030k_apply_timer_config(int pixel_row, int pixel_col, int bpp, int pbp, int bbl) {
	int i;
	int pos = 0;
	if(pixel_col * bpp * (1+pbp) + 1 > sizeof(_po3030k_line_conf))
		return -1;

	for(i = 0; i < pixel_col; i++) {
		int j;
		for(j = 0; j < bpp; j++) {
			_po3030k_line_conf[pos++] = 1;
		}
		for(j = 0; j < pbp*bpp; j++)  {
			_po3030k_line_conf[pos++] = 0;
		}
	}
	_po3030k_line_conf[pos] = 2;  /* flag to tell "line end here" */
	blank_row_betw = bbl;
	_po3030k_row = pixel_row;

	return 0;
}

/*! Check if the current capture is finished
 * \return Zero if the current capture is in progress, non-zero if the capture is done.
 * \sa po3030k_launch_capture
 */
int po3030k_is_img_ready(void) {
	return _po3030k_img_ready;
}

/*! Initialize the camera, must be called before any other function
 */
void po3030k_init_cam(unsigned char * port, gpio cam_reset, int timer, int ic, int priority) {
	int i;
	unsigned char r[2];

	gpio_write(cam_reset, false);
	gpio_set_dir(cam_reset, GPIO_OUTPUT);
	for(i=1000;i;i--) Nop();

	/* Test the IC So the error is generated at init time, not at capture time */
	ic_enable(ic, IC_TIMER3 /* DON'T CARE*/, IC_RISING_EDGE, vsync_cb, priority, 0);
	ic_disable(ic);

	gpio_write(cam_reset, true);
	for(i=1000;i;i--) Nop();

	/* Test if this is really a po3030k CAM */
	if(i2c_read(I2C_1, PO3030K_DEVICE_ID, 0x0, r, 2) || r[0] != 0x30 || r[1] != 0x30) {
		ERROR(PO3030K_IO_ERROR,r);
	}
	timer_init(timer, 1, -1);

	_po3030k_port = port + 2 /* The TRIS is given, get the PORT */;
	timer_priority = priority;
	timer_id = timer;
	ic_id = ic;
}
