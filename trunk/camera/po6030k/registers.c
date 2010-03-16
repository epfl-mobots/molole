
/*! \file
 * \brief Manage po6030k registers (two timers)
 * \author Philippe Rétornaz
 * \addtogroup camera_po6030k 
 */
/*@{*/



#include "../../i2c/i2c.h"
#include "../../clock/clock.h" 
#include "po6030k.h"

/* these two define should go into a private header but, well, 
 * I won't do it one for only two define ... */
#define 	ARRAY_ORIGINE_X	80
#define		ARRAY_ORIGINE_Y 8

#define DEVICE_ID PO6030K_DEVICE_ID
#define BANK_REGISTER 0x3

void po6030k_set_bank(unsigned char bank) {
	i2c_write(I2C_1,DEVICE_ID,BANK_REGISTER, &bank, 1);
}
void po6030k_write_register(unsigned char bank, unsigned char reg, unsigned char value) {
	po6030k_set_bank(bank);
	i2c_write(I2C_1,DEVICE_ID, reg, &value, 1);
}

unsigned char po6030k_read_register(unsigned char bank, unsigned char reg) {
	unsigned char ret;
	po6030k_set_bank(bank);
	i2c_read(I2C_1,DEVICE_ID,reg,&ret, 1);
	return ret;

}
void po6030k_set_bayer_clkdiv(unsigned char div) {
	po6030k_write_register(BANK_A, 0x91, div);
}

void po6030k_set_pclkdiv(unsigned char div) {
	po6030k_write_register(BANK_B, 0x68, div);
}

static int po6030k_set_sampl_gray(unsigned char sample) {
	switch (sample) {
		case MODE_VGA:
			po6030k_set_pclkdiv(1);
			po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
		case MODE_QVGA:
			po6030k_set_pclkdiv(3);
			po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
		case MODE_QQVGA:
			po6030k_set_pclkdiv(7);
			po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
	} 
	return -1;
}

static int po6030k_set_sampl_color(unsigned char sample) {
	switch (sample) {
		case MODE_VGA:
			po6030k_set_pclkdiv(0);
			po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
		case MODE_QVGA:
			po6030k_set_pclkdiv(1);
			po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
		case MODE_QQVGA:
			po6030k_set_pclkdiv(3);
			po6030k_write_register(BANK_B, 0x80, sample); /* Scale X*/
			po6030k_write_register(BANK_B, 0x81, sample); /* Scale Y*/
			po6030k_write_register(BANK_B, 0x82, 1); /* WTF ?!! */
			return 0;
	} 
	return -1;
}

int po6030k_set_mode(unsigned char format, unsigned char sampl_mode) {
	switch(format) {
		case GREY_SCALE_MODE:	
			po6030k_write_register(BANK_B, 0x38, 0x0D);
			return po6030k_set_sampl_gray(sampl_mode);
		case RGB_565_MODE:
			po6030k_write_register(BANK_B, 0x38, 0x08);
			return po6030k_set_sampl_color(sampl_mode);
		case YUV_MODE:
			po6030k_write_register(BANK_B, 0x38, 0x02);
			return po6030k_set_sampl_color(sampl_mode);
	}		
	return -1;
}

int po6030k_set_wx(unsigned int start,unsigned int stop) {
	unsigned char start_h,start_l,stop_h,stop_l;

	if(start >= stop)
		return -1;
	if(stop > 799)
		return -1;
	start--;
	start_l = (unsigned char) start;
	start_h = (unsigned char) (start >> 8);
	stop_l = (unsigned char) stop;
	stop_h = (unsigned char) (stop >> 8);
	
//	start_l = e_po6030k_read_register(BANK_B, 0x51);
	

	po6030k_write_register(BANK_B, 0x51, start_l);
	po6030k_write_register(BANK_B, 0x50, start_h);
	
//	e_po6030k_write_register(BANK_B, 0x54, stop_h);
//	e_po6030k_write_register(BANK_B, 0x55, stop_l);
	
	return 0;
}

int po6030k_set_wy(unsigned int start, unsigned int stop) {
	unsigned char start_h,start_l,stop_h,stop_l;

	return 0;
	if(start >= stop)
		return -1; 
	if(stop > 499)
		return -1;
	start_l = (unsigned char) start;
	start_h = (unsigned char) (start >> 8);
	stop_l = (unsigned char) stop;
	stop_h = (unsigned char) (stop >> 8);

	po6030k_write_register(BANK_B, 0x52, start_h);
	po6030k_write_register(BANK_B, 0x53, start_l);
	
//	e_po6030k_write_register(BANK_B, 0x56, stop_h);
//	e_po6030k_write_register(BANK_B, 0x57, stop_l);

	return 0;
}

int po6030k_set_vsync(unsigned int start,unsigned int stop) {
	unsigned char start_h,start_l,stop_h,stop_l;

	if(start >= stop)
		return -1; 
	if(stop > 499)
		return -1;

	start_l = (unsigned char) start;
	start_h = (unsigned char) (start >> 8);
	stop_l = (unsigned char) stop;
	stop_h = (unsigned char) (stop >> 8);

	po6030k_write_register(BANK_B, 0x60, start_h);
	po6030k_write_register(BANK_B, 0x61, start_l);
	
	po6030k_write_register(BANK_B, 0x62, stop_h);
	po6030k_write_register(BANK_B, 0x63, stop_l);
	
//	e_po6030k_write_register(BANK_B, 0x64, col_l);
//	e_po6030k_write_register(BANK_B, 0x65, col_h);

	return 0;
}

void po6030k_set_sketch_mode(int mode) {
	po6030k_write_register(BANK_C, 0x5A, 0x01);
	po6030k_write_register(BANK_B, 0x32, 0x41);
	if(mode == PO6030K_SKETCH_BW) {
		po6030k_write_register(BANK_B, 0x88, 0xFF);
		po6030k_write_register(BANK_B, 0x89, 0xFF);
		po6030k_write_register(BANK_B, 0x8A, 0x08);
		po6030k_write_register(BANK_B, 0x8B, 0xFF);
	} else {
		po6030k_write_register(BANK_B, 0x88, 0x20);
		po6030k_write_register(BANK_B, 0x89, 0x80);
		po6030k_write_register(BANK_B, 0x8A, 0x08);
		po6030k_write_register(BANK_B, 0x8B, 0xFF);
	}
}

/*@}*/
