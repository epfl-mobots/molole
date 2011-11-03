/*! \file
 * \ingroup camera1
 * \brief Manage camera's interrupts
 * \author Philippe Rétornaz
 * \verbinclude interrupt.s
 * \addtogroup camera_po6030k 
 */
/*@{*/


#include <p33fxxxx.h>
#include "po6030k.h"
#include "../../ic/ic.h"
#include "../../timer/timer.h"
#include "../../i2c/i2c.h"
#include "../../error/error.h"
#include "../../clock/clock.h"


/*! The buffer to write to */
char * _po6030k_buffer;

/*! Internal flag to tell that the pixel clock is half speed */
int __attribute((near)) _po6030k_slow_path; 

/*! The flag to tell, the image is ready or not
 * Zero mean capture is in progress, non-zero mean capture done.
 * \sa e_po6030k_is_img_ready
 */
int _po6030k_img_ready;

static po6030k_callback done_cb;

static int blank_row_betw;
static int timer_priority;
static int timer_id;
static int ic_id;
static gpio cam_r;

int __attribute((near)) _po6030k_current_row;
int __attribute((near)) _po6030k_row;

unsigned char * _po6030k_port;

char _po6030k_line_conf[320*2+1];

/* Yes, it's an ISR ... we goto() to it from the timer irq. */
void _ISR _po6030k_disable_hsync(void) {
	timer_disable(timer_id);
	if(done_cb)
		done_cb();
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
 * \param cb Pointer to a function that will be called upon capture done
 * \sa po6030k_config_cam and po6030k_is_img_ready
 */
void po6030k_launch_capture(char * buf, po6030k_callback cb) {
	_po6030k_current_row = 0;
	_po6030k_buffer = buf;
	_po6030k_img_ready = 0;
	done_cb = cb;
	init_hsync();
	/* VSync must always be initialised as the last one */
	ic_enable(ic_id, IC_TIMER3 /* DON'T CARE*/, IC_RISING_EDGE, vsync_cb, timer_priority -1, 0);
}


/*! Modify the interrupt configuration
 * \warning This is an internal function, use \a e_po6030k_config_cam
 * \param pixel_row The number of row to take
 * \param pixel_col The number of pixel to take each \a pixel_row
 * \param bpp The number of byte per pixel
 * \param pbp The number of pixel to ignore between each pixel
 * \param bbl The number of row to ignore between each line
 * \return Zero if OK, non-zero if the mode exceed internal data representation
 * \sa e_po6030k_get_bytes_per_pixel and e_po6030k_config_cam
 */
int po6030k_apply_timer_config(int pixel_row, int pixel_col, int bpp, int pbp, int bbl) {
	int i;
	int pos = 0;
	if(pixel_col * bpp * (1+pbp) + 1 > sizeof(_po6030k_line_conf))
		return -1;

	for(i = 0; i < pixel_col; i++) {
		int j;
		for(j = 0; j < bpp; j++) {
			_po6030k_line_conf[pos++] = 1;
		}
		for(j = 0; j < pbp*bpp; j++)  {
			_po6030k_line_conf[pos++] = 0;
		}
	}
	_po6030k_line_conf[pos] = 2;  /* flag to tell "line end here" */
	blank_row_betw = bbl;
	_po6030k_row = pixel_row;

	return 0;
}

/*! Check if the current capture is finished
 * \return Zero if the current capture is in progress, non-zero if the capture is done.
 * \sa e_po6030k_launch_capture
 */
int po6030k_is_img_ready(void) {
	return _po6030k_img_ready;
}

/*! Initialize the camera, must be called before any other function
 */
void po6030k_init_cam(unsigned char * port, gpio cam_reset, int timer, int ic, int priority) {
	int i;
	unsigned char r[2];
	
	cam_r = cam_reset;

	gpio_write(cam_reset, false);
	gpio_set_dir(cam_reset, GPIO_OUTPUT);
	// must be held 16 master clock, mean 32 cycle
	for(i = 0; i < 32; i++) 
		Nop();

	/* Test the IC So the error is generated at init time, not at capture time */
	ic_enable(ic, IC_TIMER3 /* DON'T CARE*/, IC_RISING_EDGE, vsync_cb, priority, 0);
	ic_disable(ic);

	gpio_write(cam_reset, true);
	// wait so the camera finish the reset
	clock_delay_us(1000);

	/* Test if this is really a po6030k CAM */
	if(!i2c_read(I2C_1, PO6030K_DEVICE_ID, 0x0, r, 2) || r[0] != 0x60 || r[1] != 0x30) {
		ERROR(PO6030K_IO_ERROR,r);
	}
	timer_init(timer, 1, -1);

	_po6030k_port = port + 2 /* The TRIS is given, get the PORT */;
	timer_priority = priority;
	timer_id = timer;
	ic_id = ic;
}

void po6030k_reset(void) {
	int i;
	gpio_write(cam_r, false);
	// must be held 16 master clock, mean 32 cycle
	for(i = 0; i < 32; i++) 
		Nop();
		
	gpio_write(cam_r, true);
	// wait so the camera finish the reset
	clock_delay_us(1000);
}	

/*@}*/
