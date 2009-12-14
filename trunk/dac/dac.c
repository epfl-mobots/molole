#include <p33fxxxx.h>
#include "../dma/dma.h"

#include "dac.h"

static int dma_channel_r;
static int dma_channel_l;
static void * dma_bufferR[2];
static void * dma_bufferL[2];
static dac_callback dac_cb;

/**
        \defgroup dac DAC

	DAC driver for molole. Can only be used with FRCPLL oscillator
*/
/*@{*/

static void set_auxclk_divider(int div) {
	switch(div) {
		case 1:
			ACLKCONbits.APSTSCLR = 0b111;
			break;
		case 2:
			ACLKCONbits.APSTSCLR = 0b110;
			break;
		case 4:
			ACLKCONbits.APSTSCLR = 0b101;
			break;
		case 8:
			ACLKCONbits.APSTSCLR = 0b100;
			break;
		case 16:
			ACLKCONbits.APSTSCLR = 0b011;
			break;
		case 32:
			ACLKCONbits.APSTSCLR = 0b010;
			break;
		case 64:
			ACLKCONbits.APSTSCLR = 0b001;
			break;
		case 256:
			ACLKCONbits.APSTSCLR = 0b000;
			break;
	}
}

static void callback_dma(int channel, bool first_buffer) {
	int rl = channel == dma_channel_r ? DAC_RIGHT_CHANNEL : DAC_LEFT_CHANNEL;
	int buf = first_buffer ? 0 : 1;
	
	if(rl == DAC_RIGHT_CHANNEL)
		dac_cb(rl, dma_bufferR[buf]);
	else
		dac_cb(rl, dma_bufferL[buf]);
}


/* We only support the Fvco input clock */
/* The pic MUST be using the FRCPLL CLOCK ! */
/* The value returned is the real sampling frequency in khz */

/**
      Initialize but does not emable the DAC1 module 
      
      The microcontroller MUST be using the PLL as the DAC take it's clock from the PLL
      
      \return	The true sampling speed
      
      \param 	hz
		The desired sampling speed in Hz
      \param 	format
		The binary format of the data. One of DAC_FORMAT_UNSIGNED or DAC_FORMAT_SIGNED.
      \param	buffer_word_size
		The size of each DMA buffer in word (16 bits) size.
      \param	r_dma
		The right DMA channel, from \ref DMA_CHANNEL_0 to \ref DMA_CHANNEL_7 .
		If the right channel is not used set r_dma_bufferA and r_dma_bufferB to NULL.
      \param	r_dma_bufferA
		The right DMA channel buffer A
      \param	r_dma_bufferB
		The right DMA channel buffer B
      \param	out_m_r
		Enable the middle level output for the right channel
      \param	l_dma
		The left DMA channel, from \ref DMA_CHANNEL_0 to \ref DMA_CHANNEL_7 .
		If the left channel is not used set l_dma_bufferA and l_dma_bufferB to NULL.
      \param 	l_dma_bufferA
		The left DMA channel buffer A
      \param	l_dma_bufferB
		The left DMA channel buffer B
      \param	out_m_l
		Enable the middle level output for the left channel
      \param	cb
		The DAC callback.
      \param 	priority
		The DAC callback interrupt priority.
*/
    
unsigned long dac_init(unsigned long hz, int format, unsigned int buffer_word_size,
			int r_dma, void * r_dma_bufferA, void * r_dma_bufferB, bool out_m_r,
			int l_dma, void * l_dma_bufferA, void * l_dma_bufferB, bool out_m_l,
			dac_callback cb, int priority) {
	
	ERROR_CHECK_RANGE(format, 0, 1, DAC_ERROR_INVALID_FORMAT);
	ERROR_CHECK_RANGE(hz, 0, 100000, DAC_ERROR_INVALID_FREQ);
	
	// First compute the clock and get the real clock
	// Yeah, we do it in floating point, but we don't care since it's only done once at the initialisation. */

	long fvco = (7370000L * (_PLLDIV + 2)) / (_PLLPRE + 2);
	int divider;
	long fsamp;
	float fdac = hz * 256.;
	int i;
	
	for(i = 1; i < 200; i = i << 1) {
		if(i == 128) /* @#! */
			i = 256;
		
		divider = (fvco / (fdac * i)) + 0.5;
		
		if(divider == 0)
			divider = 1;
			
		if(divider <= 128)
			break;
	}
	if(i == 512)
		i = 256; // Input clock is too high .... 
	
	fsamp = ((fvco / divider) / i) / 256;
	
	set_auxclk_divider(i);
	DAC1CONbits.DACFDIV = divider - 1;
	
	ACLKCONbits.SELACLK = 0; /* Aux clock is VCO output */
	
	DAC1CONbits.DACSIDL = 0; /* Don't stop in idle mode */
	
	DAC1CONbits.AMPON = 0; /* Disable output amplifier in sleep mode */
	
	DAC1CONbits.FORM = format;
	
	dma_channel_r = -1;
	dma_channel_l = -1;
	
	if(r_dma_bufferA && r_dma_bufferB) {
		/* Right channel enabled */
		DAC1STATbits.RITYPE = 1; /* Interrupt if fifo is empty */
		DAC1STATbits.RMVOEN = out_m_r ? 1 : 0;
		DAC1STATbits.ROEN = 1;

		dma_init_channel(r_dma, DMA_INTERRUPT_SOURCE_DAC1_RC, DMA_SIZE_WORD, 
						DMA_DIR_FROM_RAM_TO_PERIPHERAL, DMA_INTERRUPT_AT_FULL,
						DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL, DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT,
						DMA_OPERATING_CONTINUOUS_PING_PONG, r_dma_bufferA, r_dma_bufferB,
						(void *)&DAC1RDAT, buffer_word_size, callback_dma);
	
		dma_set_priority(r_dma, priority);
		
		dma_channel_r = r_dma;
		dma_bufferR[0] = r_dma_bufferA;
		dma_bufferR[1] = r_dma_bufferB;

	}
	if(l_dma_bufferA && l_dma_bufferB) {
		/* Left channel enabled */
		DAC1STATbits.LITYPE = 1; /* Interrupt if fifo is empty */
		DAC1STATbits.LMVOEN = out_m_l ? 1 : 0;
		DAC1STATbits.LOEN = 1;

		dma_init_channel(l_dma, DMA_INTERRUPT_SOURCE_DAC1_LC, DMA_SIZE_WORD, 
						DMA_DIR_FROM_RAM_TO_PERIPHERAL, DMA_INTERRUPT_AT_FULL,
						DMA_DO_NOT_NULL_WRITE_TO_PERIPHERAL, DMA_ADDRESSING_REGISTER_INDIRECT_POST_INCREMENT,
						DMA_OPERATING_CONTINUOUS_PING_PONG, l_dma_bufferA, l_dma_bufferB,
						(void *)&DAC1LDAT, buffer_word_size, callback_dma);
		dma_set_priority(l_dma, priority);
		
		dma_channel_l = l_dma;
		dma_bufferL[0] = l_dma_bufferA;
		dma_bufferL[1] = l_dma_bufferB;
	}
	
	dac_cb = cb;
	return fsamp;
}


/**
	Set the DAC1 default value
	
	\param	dflt
		The default value in the same format as the DAC configuration 
*/
void dac_set_default_value(unsigned int dflt) {
	DAC1DFLT = dflt;
}

/**
	Stop the DAC1
	The output will be immediatly disabled. Both DMA channel will be reseted
*/
void dac_stop(void) {
	// Stop the DAC, stop the DMA, reset _EVERYTHING_ so the dma channel are in sync (A,B)
	DAC1CONbits.DACEN = 0;
	
	if(dma_channel_l != -1)
		dma_disable_channel(dma_channel_l);
	if(dma_channel_r != -1)
		dma_disable_channel(dma_channel_r);
}

/* You should have filled ALL the DMA buffer before starting the DAC */
/**
	Start the DAC1
	You must have called dac_init() before.
	All the DMA channel must be properly filled. The DAC will start playing channel A and then switch to channel B.
*/
void dac_start(void) {
	if(dma_channel_l != -1)
		dma_enable_channel(dma_channel_l);
	
	if(dma_channel_r != -1) 
		dma_enable_channel(dma_channel_r);

	DAC1CONbits.DACEN = 1;
}

/*@}*/


