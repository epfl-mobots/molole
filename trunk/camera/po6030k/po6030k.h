/*! \file
 * \brief PO6030k library header
 * \author Philippe R�tornaz
 */



#ifndef __PO6030K_H__
#define __PO6030K_H__


#include "../../gpio/gpio.h"

#define PO6030K_DEVICE_ID  	0x6E 


#define	ARRAY_WIDTH			640		
#define	ARRAY_HEIGHT		480

#define GREY_SCALE_MODE		0
#define RGB_565_MODE		1
#define YUV_MODE			2

#define MODE_VGA 			0x20
#define MODE_QVGA 			0x40
#define MODE_QQVGA 			0x80

#define BAYER_CLOCK_1 0x10
#define BAYER_CLOCK_2 0x50
#define BAYER_CLOCK_4 0x90
#define BAYER_CLOCK_8 0xB0

#define BANK_A 0x0
#define BANK_B 0x1
#define BANK_C 0x2
#define BANK_D 0x3

#define SPEED_1 BAYER_CLOCK_1
#define SPEED_2 BAYER_CLOCK_2
#define SPEED_4 BAYER_CLOCK_4
#define SPEED_8 BAYER_CLOCK_8


/** Errors po3030k can throw */
enum po6030k_errors
{
	PO6030K_ERROR_BASE = 0x0F00,
	PO6030K_IO_ERROR,
};

int po6030k_config_cam(unsigned int sensor_x1,unsigned int sensor_y1,
			 unsigned int sensor_width,unsigned int sensor_height,
			 unsigned int zoom_fact_width,unsigned int zoom_fact_height,  
			 int color_mode);

int  po6030k_get_bytes_per_pixel(int color_mode);

void po6030k_set_color_mode(unsigned char format);

void po6030k_set_bank(unsigned char bank);
void po6030k_write_register(unsigned char bank, unsigned char reg, unsigned char value);
#define po6030k_set_speed(div) po6030k_set_bayer_clkdiv(div)
void po6030k_set_bayer_clkdiv(unsigned char div);

void po6030k_set_pclkdiv(unsigned char div);
int po6030k_set_mode(unsigned char format, unsigned char sampl_mode);


void po6030k_init_cam(unsigned char * port, gpio cam_reset, int timer, int ic, int priority);

int po6030k_get_bytes_per_pixel(int color_mode);
int po6030k_set_wx(unsigned int start,unsigned int stop);

int po6030k_set_wy(unsigned int start, unsigned int stop);

int po6030k_set_vsync(unsigned int start,unsigned int stop);

int po6030k_is_img_ready(void);
int po6030k_apply_timer_config(int pixel_row, int pixel_col, int bpp, int pbp, int bbl);
void po6030k_launch_capture(char * buf);

#define PO6030K_SKETCH_BW 0
#define PO6030K_SKETCH_COLOR 1
void po6030k_set_sketch_mode(int mode);

#endif
