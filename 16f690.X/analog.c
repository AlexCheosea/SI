#include "analog.h"
#include <xc.h>
#include <stdint.h>
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000
#endif
#define VREF_MV 4700UL //Tensiune de alimentare MCU
#define SCALE_FACTOR (((VREF_MV*1024UL) + 511UL) / 1023UL)
void adc_init(void) {
    // 1. Set pins as analog
    // ANSELbits.ANS0 = 1;   // RA0 = Analog (Temperature)
    // ANSELbits.ANS1 = 1;   // RA1 = Analog (Light)
    // ANSELbits.ANS2 = 1;   // RA2 = Analog (Potentiometer)

    // 2. Set pins as inputs
    TRISAbits.TRISA0 = 1; 
    TRISAbits.TRISA1 = 1; 
    TRISAbits.TRISA2 = 1; 

    // 3. Configure ADC Module
    // ADCON1bits.ADCS = 0b010; // ADC Clock = Fosc/32 (Safe for 8MHz)
    // ADCON0bits.ADFM = 1;    // Right justified (0-1023)
    // ADCON0bits.VCFG = 0;    // Vref+ = VDD (VREF_MV)
    // ADCON0bits.ADON = 1;    // Turn ADC module ON
}