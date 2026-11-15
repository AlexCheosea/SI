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
    OSCCON = 0x71; // oscilator intern: IRCF = frecventa (31 kHz..8 MHz), SCS = sursa ceasului
    adc_init();
    motor_init();
    spi_init();

    // Timer0: timeout SPI, verificat prin polling (fara intrerupere)
    INTCONbits.T0IE = 0;
    OPTION_REGbits.T0CS = 0; // sursa Timer0: 0 = Fosc/4, 1 = pin T0CKI
    OPTION_REGbits.PSA = 0;  // prescaler: 0 = la Timer0, 1 = la watchdog
    OPTION_REGbits.PS2 = 1;  // PS2:PS0 = prescaler, 1:2 .. 1:256
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 0;
    TMR0 = 0;
    INTCONbits.T0IF = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    // Watchdog pornit dupa initializare; resetat doar din bucla principala
    // (nu din ISR: o bucla blocata ar fi mascata de intreruperi)
    CLRWDT();
    WDTCONbits.WDTPS = WDT_PRESCALER_1_4096;
    WDTCONbits.SWDTEN = 1;
}
void __interrupt() isr_handler(){
    if(PIR1bits.SSPIF)
        spi_isr();
    if(PIR1bits.CCP1IF){ // pas motor
        motor_isr_handler();
        // Octet SPI sosit in timpul pasului: servit fara reintrare in ISR
        if(PIR1bits.SSPIF)
            spi_isr();
    }
}
