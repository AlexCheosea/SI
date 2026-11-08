#ifndef INIT_H
#define	INIT_H

#include <stdint.h>
#include "buzzer.h"
#include "hmi.h"
#include "spi.h"
#include "uart.h"
#include "analog.h"
// =============================================================================
// CONFIGURATION BITS PENTRU PIC16F877A
// =============================================================================

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (High Speed Crystal/Resonator necesar pentru 18.432 MHz)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT dezactivat pentru dezvoltare)
#pragma config PWRTE = OFF       // Power-up Timer Enable bit (PWRT activat pentru stabilitate la pornire)
#pragma config BOREN = OFF       // Brown-out Reset Enable bit (Resetare automată dacă scade tensiunea)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (Dezactivat pentru a elibera pinul RB3)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Protecție EEPROM oprită)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Scrierea în Flash din cod dezactivată)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Protecție Flash oprită)
#pragma config DEBUG = OFF      

extern void uart_init(void);
extern void buzzer_init(void);
extern void hmi_init(void);
extern void spi_init(void);
extern void uart_init(void);
extern void hmi_update(void);
extern void uart_rx_isr(void);
extern void uart_tx_isr(void);
/**
 * Configures the microcontrollers peripherals, sets up Timer0 
 * for a 1ms periodic interrupt, and enables global interrupts.
 */
void system_init(void);

#endif	/* INIT_H */