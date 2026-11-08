#define TMR0_RELOAD_VAL 112 //valoarea pentru intrerupere de 1ms
#define TMR0_SET        0b10000100
#define _XTAL_FREQ 18432000UL
#include <xc.h>
#include "init.h"
void system_init(void) { //apeleaza toate functiile de initializare din assembly
    __delay_ms(50);
    buzzer_init();
    hmi_init();
    spi_init();
    uart_init();
    adc_init();
    //Timer0 (Baza de timp HMI))
    OPTION_REG = TMR0_SET;  // Prescaler 1:32 alocat lui Timer0
    TMR0 = TMR0_RELOAD_VAL; // Valoare pentru generare intrerupere la 1 ms 
    INTCONbits.T0IE = 1;    // Activează întreruperile Timer0 (pentru HMI)
    INTCONbits.PEIE = 1;    // Activează întreruperile Periferice (pentru UART)
    
    INTCONbits.GIE = 1;     // Activează întreruperile Globale
    return;
}

// =============================================================================
// WRAPPER-UL PRINCIPAL DE ÎNTRERUPERI (ISR)
// =============================================================================


void __interrupt() isr(void){
    
    // [SURSA 1]: Întrerupere de la Timer0 (Baza de timp de 1ms)
    if (T0IE && T0IF) {
        TMR0 = TMR0_RELOAD_VAL;
        hmi_update();
        
        if(buzzer_ms_ramase > 0){
            buzzer_ms_ramase--;
            if(buzzer_ms_ramase == 0){
                buzzer_off();
            }
        }
        if(sample_timer > 0){
            sample_timer--;
        }
        if(motor_voltage_timer > 0){
            motor_voltage_timer--;
        }
        if(motor_speed_report_timer > 0){
            motor_speed_report_timer--;
        }
        if(status_report_timer > 0){
            status_report_timer--;
        }
        T0IF = 0;
    }
    // [SURSA 2]: Întrerupere RX UART (A sosit un octet de la PC/Interfață)
    if (RCIE && RCIF) {
        uart_rx_isr();
    }
    
    // [SURSA 3]: Întrerupere TX UART (Bufferul hardware de transmisie este gol)
    if (TXIE && TXIF) {
        // NOTĂ: Funcția uart_tx_isr() va scrie următorul octet din buffer în TXREG.;
        uart_tx_isr();
    }
}
