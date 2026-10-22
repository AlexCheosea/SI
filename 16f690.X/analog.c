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
    ANSELbits.ANS0 = 1;   // RA0 = Analog (Temperature)
    ANSELbits.ANS1 = 1;   // RA1 = Analog (Light)
    ANSELbits.ANS2 = 1;   // RA2 = Analog (Potentiometer)

    // 2. Set pins as inputs
    TRISAbits.TRISA0 = 1; 
    TRISAbits.TRISA1 = 1; 
    TRISAbits.TRISA2 = 1; 

    // 3. Configure ADC Module
    ADCON1bits.ADCS = 0b010; // ADC Clock = Fosc/32 (Safe for 8MHz)
    ADCON0bits.ADFM = 1;    // Right justified (0-1023)
    ADCON0bits.VCFG = 0;    // Vref+ = VDD (VREF_MV)
    ADCON0bits.ADON = 1;    // Turn ADC module ON
}



uint16_t adc_read(uint8_t channel) {
    ADCON0bits.CHS = channel;   // Select ADC channel
    
    // 1. Dummy Read: Flush residual charge from the previous channel
    __delay_us(ACQ_DELAY);             // Acquisition time for the dummy read
    ADCON0bits.GO_nDONE = 1;    // Start dummy conversion
    while(ADCON0bits.GO_nDONE); // Wait until done
    volatile uint16_t v = (((uint16_t)ADRESH << 8) | ADRESL); // Discard result
    
    // 2. Sample Loop: Accumulate 8 samples spaced apart
    uint16_t raw_sum = 0;
    uint16_t min = 1023;
    uint16_t max = 0;
    
    for(uint8_t i = 0; i < 10; i++) {
        __delay_us(SAMPLE_DELAY);         // Spacing delay between samples
        
        ADCON0bits.GO_nDONE = 1;    // Start valid conversion
        while(ADCON0bits.GO_nDONE); // Wait until done
        
        // Accumulate 10-bit result safely cast to 16-bit
        v = ((((uint16_t)ADRESH) << 8) | ADRESL);
        if (v > max) max = v;
        if (v < min) min = v;
        raw_sum += v;
    }
    raw_sum -= (min+max);
    
    // 3. Return Average: Divide the sum by 8 using a bitwise shift (>> 3)
    
    return (uint16_t)(raw_sum >> 3); 
}

//ALL OF THE BELOW FUNCTIONS ASSUME VDD = VREF_MV
//ADJUST SCALING FACTORS BASED ON ACTUAL VALUES
// Returns temperature in decidegrees (0.1°C steps) assuming VDD = VREF_MV
int16_t read_temp(void) {
    uint16_t raw = adc_read(CH_TEMP);
    
    // Adjusted SCALE_FACTOR maps the 10-bit range perfectly to a VREF_MV scale
    uint32_t temp_shifted = ((uint32_t)raw * SCALE_FACTOR + 512) >> 10;
    
    return (int16_t)temp_shifted;
}

// Returns light level as a percentage (0 - 100%) - Unchanged because % is ratiometric
uint8_t read_lgt(void) {
    uint16_t raw = adc_read(CH_LGT);
    
    uint16_t percent = ((uint32_t)raw * 100 + 511) / 1023;
    
    if (percent > 100) {
        percent = 100;
    }
    
    return (uint8_t)percent;
}
// Returns potentiometer voltage in millivolts assuming VDD = VREF_MV
uint16_t read_pot(void) {
    uint16_t raw = adc_read(CH_POT);
    
    uint32_t millivolts = ((uint32_t)raw * SCALE_FACTOR + 512) >> 10;
    
    return (uint16_t)millivolts;
}