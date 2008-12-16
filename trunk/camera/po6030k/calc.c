/*! \file
 * \brief Calculate the timing for the camera
 * \author Philippe R�tornaz
 */

#include "po6030k.h"
#include "../../i2c/i2c.h"
#include <p33fxxxx.h>

#define 	ARRAY_ORIGINE_X	80
#define		ARRAY_ORIGINE_Y 8

#define IRQ_PIX_LAT 1

extern int _po6030k_slow_path;

/*! This function setup the internal timing of the camera to match the 
 * zoom, color mode and interest area.
 * \warning If the common deniminator of both zoom factor is 4 or 2, part of the 
 * subsampling is done by the camera ( QQVGA = 4, QVGA = 2 ). This increase the 
 * framerate by respectivly 4 or 2. Moreover greyscale is twice faster than color mode.
 * \param sensor_x1 The X coordinate of the window's corner
 * \param sensor_y1 The Y coordinate of the window's corner
 * \param sensor_width the Width of the interest area, in FULL sampling scale
 * \param sensor_height The Height of the insterest area, in FULL sampling scale
 * \param zoom_fact_width The subsampling to apply for the window's Width
 * \param zoom_fact_height The subsampling to apply for the window's Height
 * \param color_mode The color mode in which the camera should be configured
 */

void po6030k_config_cam(unsigned int sensor_x1,unsigned int sensor_y1,
				unsigned int sensor_width,unsigned int sensor_height,
				unsigned int zoom_fact_width,unsigned int zoom_fact_height,  
				int color_mode) {
	int pbp_h, pbp_w;
	int nb_pixels, nb_lines;
	int real_zoom_h, real_zoom_w;
	int zoom_sample;
	int sampl_mode;

	sensor_x1 += ARRAY_ORIGINE_X - 64;
	sensor_y1 += ARRAY_ORIGINE_Y;
	/* testing if the mode is acceptable */
	if(zoom_fact_height < 1 || zoom_fact_width < 1)
		ERROR(PO6030K_INVALID_ZOOM, NULL);

	/* Check if the area is out of bound */
	if((sensor_x1 + sensor_width) > (ARRAY_ORIGINE_X + ARRAY_WIDTH)) 
		ERROR(PO6030K_ARRAY_OUT_OF_BOUND,sensor_x1 + sensor_width);
	if((sensor_y1 + sensor_height) > (ARRAY_ORIGINE_Y + ARRAY_HEIGHT))
		ERROR(PO6030K_ARRAY_OUT_OF_BOUND, sensor_y1 + sensor_height);
	
	/* Check if Sensor[Width|Height] is a multiple of Zoom */
	if(sensor_width % zoom_fact_width)
		ERROR(PO6030K_NONMULTIPLE_SIZE, sensor_width);
	if(sensor_height % zoom_fact_height)
		ERROR(PO6030K_NONMULTIPLE_SIZE, sensor_height);	

	/* Search the best subsampling aviable */
	if(!(zoom_fact_height%4)) {
		if(!(zoom_fact_width%4)) {
			sampl_mode = MODE_QQVGA;
			zoom_sample = 4;
			real_zoom_h = zoom_fact_height >> 2;
			real_zoom_w = zoom_fact_width >> 2;
			/* this is done because we must have Vsync a real
			 * row before the real picture */
			sensor_y1 -= 4;
			/* We need Hsync some time before the first effective pixel */
			sensor_x1 -= IRQ_PIX_LAT*4;
		} else {
			if(!(zoom_fact_width%2)) {
				sampl_mode = MODE_QVGA;
				zoom_sample = 2;
				real_zoom_h = zoom_fact_height >> 1;
				real_zoom_w = zoom_fact_width >> 1;
				sensor_y1 -= 2;
				sensor_x1 -= 2*IRQ_PIX_LAT;
			} else {
				sampl_mode = MODE_VGA;
				zoom_sample = 1;
				real_zoom_h = zoom_fact_height;
				real_zoom_w = zoom_fact_width;
				sensor_y1--;	
				sensor_x1 -= IRQ_PIX_LAT;
			}
		}
	} else if(!(zoom_fact_height%2)) {
		if(!(zoom_fact_width%2)) {
			sampl_mode = MODE_QVGA;
			zoom_sample = 2;
			real_zoom_h = zoom_fact_height >> 1;
			real_zoom_w = zoom_fact_width >> 1;
			sensor_y1 -= 2;
			sensor_x1 -= 2*IRQ_PIX_LAT;
		} else {
			sampl_mode = MODE_VGA;
			zoom_sample = 1;
			real_zoom_h = zoom_fact_height;
			real_zoom_w = zoom_fact_width;
			sensor_y1--;
			sensor_x1 -= IRQ_PIX_LAT;
		}
	} else {	
		zoom_sample = 1;
		sampl_mode = MODE_VGA;
		real_zoom_w = zoom_fact_width;
		real_zoom_h = zoom_fact_height;
		sensor_y1--;
		sensor_x1 -= IRQ_PIX_LAT;
	}
	pbp_w = real_zoom_w - 1;
	pbp_h = real_zoom_h - 1;
	
	nb_pixels = sensor_width / zoom_fact_width;
	nb_lines = sensor_height/ zoom_fact_height;

	/* set camera configuration */

	if(po6030k_set_wx(sensor_x1 /zoom_sample,(ARRAY_ORIGINE_X + ARRAY_WIDTH + 1)/zoom_sample)) 
		ERROR(PO6030K_INTERNAL_ERROR, 1);

	if(po6030k_set_wy(sensor_y1 /zoom_sample,(ARRAY_ORIGINE_Y + ARRAY_HEIGHT + 1)/zoom_sample))
		ERROR(PO6030K_INTERNAL_ERROR, 2);

	if(po6030k_set_vsync(sensor_y1,ARRAY_ORIGINE_Y + ARRAY_HEIGHT))
		ERROR(PO6030K_INTERNAL_ERROR, 3);

	_po6030k_slow_path = 0;

	if(color_mode == GREY_SCALE_MODE) {
		switch (sampl_mode) {
			case MODE_VGA:
				po6030k_set_speed(SPEED_2);
				break;
			case MODE_QVGA:
				po6030k_set_speed(SPEED_1);
				break;
			case MODE_QQVGA:
				_po6030k_slow_path = 1;
				po6030k_set_speed(SPEED_1);
				break;
		}
	} else {
		switch (sampl_mode) {
			case MODE_VGA:
				po6030k_set_speed(SPEED_4);
				break;
			case MODE_QVGA:
				po6030k_set_speed(SPEED_2);
				break;
			case MODE_QQVGA:
				po6030k_set_speed(SPEED_1);
				break;
			}
	}

	if(po6030k_set_mode(color_mode, sampl_mode))
		ERROR(PO6030K_UNKNOW_COLOR_MODE, color_mode);

	
	/* set timer configuration */
	if(po6030k_apply_timer_config(nb_lines,nb_pixels,po6030k_get_bytes_per_pixel(color_mode),pbp_w,pbp_h))
		ERROR(PO6030K_NOMEM, NULL);

}

/*! Return the number of bytes per pixel in the given color mode
 * \param color_mode The given color mode
 * \return The number of bytes per pixel in the given color mode
 */
int po6030k_get_bytes_per_pixel(int color_mode) {
	switch (color_mode) {
		case GREY_SCALE_MODE:
			return 1;
		case RGB_565_MODE:
			return 2;
		case YUV_MODE:
			return 2; /* YUV mode is not the best mode for subsampling */
	}
	/* UNKNOW ! */
	return 1;
}
	

