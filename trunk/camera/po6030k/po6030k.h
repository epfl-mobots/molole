/*! \file
 * \brief PO6030k library header
 * \author Philippe Rétornaz
 * \defgroup camera_po6030k Po6030k camera drivers
 *
 * This driver expose most of the Po6030k camera interfaces. Some functions
 * are useful, some other not. But since this is not up to the driver to
 * decide if a function is needed, I've exported almost all.
 *
 *
 * \section defset Default settings
 * The camera is, by default, configured with the following settings:
 * - Automatic white balance control
 * - Automatic expousesure control
 * - Automatic flicker detection ( 50Hz and 60Hz )
 * .
 * There are no default setting for the image size and color.
 *
 * \section perfsec Performances
 * The maximum framerate ( without doing anything else than acquiring the picture ) varies
 * with the subsampling and the color mode.
 * Here are some framerates:
 * - Size: 640x480, Subsampling: 16x16, RGB565: 4.3 fps
 * - Size: 16x480, Subsampling: 16x16, RGB565: 4.3 fps
 * - Size: 480x16, Subsampling: 16x16: RGB565: 4.3fps
 * - Size: 64x64, Subsampling: 4x4, RGB565: 4.3 fps
 * - Size: 32x32, Subsampling: 2x2, RGB565: 2.6 fps
 * - Size: 16x16, No Subsampling, RGB565: 1.3 fps
 * - Size: 640x480, Subsampling: 16x16, GREYSCALE: 8.6 fps
 * - Size: 16x480, Subsampling: 16x16, GREYSCALE: 8.6 fps
 * - Size: 480x16, Subsampling: 16x16, GREYSCALE: 8.6 fps
 * - Size: 64x64, Subsampling: 4x4, GREYSCALE: 8.6 fps
 * - Size: 32x32, Subsampling: 2x2, GREYSCALE: 4.3 fps
 * - Size: 16x16, No subsampling, GREYSCALE: 2.2 fps
 *
 * \section IntegrDet Important note
 * This driver is extremly sensitive to interrupt latency, thus it uses interrupt priority to
 * be sure that the latencies are kept low. The Timer4 and Timer5 interrupt priorities are set at
 * level 6 and interrupt nesting is enabled.
 * The Timer4 interrupt uses the "push.s" and "pop.s" instructions. You should not have any
 * code using thoses two instructions when you use the camera. This includes the _ISRFAST C
 * macro. If you use them, some random and really hard to debug behaviors will happen.
 * You have been warned !
 *
 * \section example_sect Examples
 *
 * \subsection ex1_sect Basic example
 *
 * \code
#include "po6030k.h"

char buffer[2*40*40];
int main(void) {

        po6030k_init_cam();
        po6030k_config_cam((ARRAY_WIDTH -160)/2,(ARRAY_HEIGHT-160)/2,
                         160,160,4,4,RGB_565_MODE);

        po6030k_launch_capture(buffer);
        while(!po6030k_is_img_ready());

        // buffer contain a 40*40 RGB picture now
        ( insert useful code here )

        return 0;
}
\endcode

 * This example tells the driver to acquire a 160x160 pixel picture from the camera, with
 * 4x subsampling, thus resulting with a 40x40 pixel. The buffer has a size of
 * 40*40*2 because RGB565 is a two bytes-per-pixel data format.
 *
 * \subsection ex2_sect More advanced example
\code
#include "po6030k.h"

char buffer[160*2];
int main(void) {
        po6030k_init_cam();
        po6030k_config_cam((ARRAY_WIDTH - 320)/2,(ARRAY_HEIGHT - 32)/2,
                        320,8,2,4,GREY_SCALE_MODE);
        po6030k_set_sketch_mode(PO6030K_SKETCH_COLOR);

        po6030k_launch_capture(buffer);
        while(!po6030k_is_img_ready());

        // Here buffer contain a 160x2 greyscale picture

        return 0;
}
\endcode
 * This example configures the camera to acquire a 320x8 pixel picture, but subsampled
 * 2x in width and 4x in height, thus resulting in a 160*2 linear
 * greyscale picture. It "emulates" a linear camera.
 */
/*@{*/


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
	PO6030K_INVALID_ZOOM,
	PO6030K_ARRAY_OUT_OF_BOUND,
	PO6030K_NONMULTIPLE_SIZE,
	PO6030K_UNKNOW_COLOR_MODE,
	PO6030K_INTERNAL_ERROR,
	PO6030K_NOMEM,
	PO6030K_IO_ERROR,
};

// Callback to be called after the image is acquired. 
// DO NOT DO ANY IMAGE PROCESSING INSIDE !
// it must be FAST
typedef void (*po6030k_callback)(void);

void po6030k_config_cam(unsigned int sensor_x1,unsigned int sensor_y1,
			 unsigned int sensor_width,unsigned int sensor_height,
			 unsigned int zoom_fact_width,unsigned int zoom_fact_height,  
			 int color_mode);
			 
void po6030k_init_cam(unsigned char * port, gpio cam_reset, int timer, int ic, int priority);

// Callback may be NULL if not used
void po6030k_launch_capture(char * buf, po6030k_callback cb);

int po6030k_is_img_ready(void);

#define PO6030K_SKETCH_BW 0
#define PO6030K_SKETCH_COLOR 1
void po6030k_set_sketch_mode(int mode);

void po6030k_reset(void);


// The following are internal function, use only if you really know what you are doing
int  po6030k_get_bytes_per_pixel(int color_mode);

void po6030k_set_color_mode(unsigned char format);

void po6030k_set_bank(unsigned char bank);
void po6030k_write_register(unsigned char bank, unsigned char reg, unsigned char value);
#define po6030k_set_speed(div) po6030k_set_bayer_clkdiv(div)
void po6030k_set_bayer_clkdiv(unsigned char div);

void po6030k_set_pclkdiv(unsigned char div);
int po6030k_set_mode(unsigned char format, unsigned char sampl_mode);



int po6030k_get_bytes_per_pixel(int color_mode);
int po6030k_set_wx(unsigned int start,unsigned int stop);

int po6030k_set_wy(unsigned int start, unsigned int stop);

int po6030k_set_vsync(unsigned int start,unsigned int stop);

int po6030k_apply_timer_config(int pixel_row, int pixel_col, int bpp, int pbp, int bbl);


#endif

/*@}*/
