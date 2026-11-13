#include "analog.h"
#include <xc.h>
#include <stdint.h>
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000
#endif
#define VREF_MV 4700UL // tensiunea de alimentare a MCU (referinta ADC)
#define SCALE_FACTOR (((VREF_MV*1024UL) + 511UL) / 1023UL)
void adc_init(void) {
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA2 = 1;
}
