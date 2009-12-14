#ifndef _DAC_H_
#define _DAC_H_

#include <types/types.h>
#include <error/error.h>

/* Warning: DAC clock source is the PLL. External oscillator not supported ! */


enum dac_errors
{
	DAC_ERROR_BASE = 0x1000,
	DAC_ERROR_INVALID_FREQ,		/**< Invalid frequency specification */
	DAC_ERROR_INVALID_FORMAT,	/**< Invalid format provided */
};

enum dac_formats
{
	DAC_FORMAT_UNSIGNED = 0,
	DAC_FORMAT_SIGNED = 1,
};

enum dac_channel
{
	DAC_RIGHT_CHANNEL,
	DAC_LEFT_CHANNEL,
};

/* In this callback, you need to fill buffer (size is equal to buffer_word_size) with new samples
 * Channel is either DAC_RIGHT_CHANNEL or DAC_LEFT_CHANNEL
 */
typedef void (*dac_callback)(int channel, void * buffer);
unsigned long dac_init(unsigned long hz, int format, unsigned int buffer_word_size,
			int r_dma, void * r_dma_bufferA, void * r_dma_bufferB, bool out_m_r,
			int l_dma, void * l_dma_bufferA, void * l_dma_bufferB, bool out_m_l,
			dac_callback cb, int priority);

void dac_set_default_value(unsigned int dflt);

void dac_stop(void);
void dac_start(void);




#endif

