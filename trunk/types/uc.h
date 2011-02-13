#ifndef _UC_H_
#define _UC_H_

#if defined(__dsPIC30F__)
#include <p30fxxxx.h>
#define DSP_AVAILABLE
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#define DSP_AVAILABLE
#elif defined(__PIC24F__)
#include <p24Fxxxx.h>
#else
#error Unknown microcontroller familly
#endif

#endif

