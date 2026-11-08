/*
 * File:   init.c
 * Author: Alex
 *
 */


#include <xc.h>
#include "init.h"
#include "motor.h"
#include "spi.h"
void system_init(){
    OSCCON = 0x71; //8MHz
    adc_init();
    motor_init();
    spi_init();
    
    INTCONbits.T0IE = 0; //dezactivam intrerupere Timer0 (folosit pentru SPI timeout cu polling)
    OPTION_REGbits.T0CS = 0; //clock intern
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS2 = 1;
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 0;
    TMR0 = 0;
    INTCONbits.T0IF = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
}
void __interrupt() isr_handler(){
    if(PIR1bits.SSPIF)
        spi_isr();
    if(PIR1bits.CCP1IF){ // pas motor (CCP1 compare, special event)
        motor_isr_handler();
        // Un octet SPI terminat in timpul pasului este servit imediat, fara
        // iesire + reintrare in ISR (~32 cicluri economisite pe calea critica).
        if(PIR1bits.SSPIF)
            spi_isr();
    }
    // Timer2 nu mai genereaza intreruperi: TMR2IF este verificat in main.
}