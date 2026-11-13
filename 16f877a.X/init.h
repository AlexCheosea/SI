#ifndef INIT_H
#define	INIT_H

#include <stdint.h>
#include "buzzer.h"
#include "hmi.h"
#include "spi.h"
#include "uart.h"
#include "analog.h"

// Biti de configurare
#pragma config FOSC = HS        // tipul oscilatorului: LP, XT, HS (cristal rapid), RC
#pragma config WDTE = OFF       // watchdog timer: ON / OFF
#pragma config PWRTE = OFF      // intarziere de ~72 ms la pornire: ON / OFF
#pragma config BOREN = OFF      // reset la scaderea tensiunii de alimentare: ON / OFF
#pragma config LVP = OFF        // programare la tensiune joasa: ON ocupa pinul RB3
#pragma config CPD = OFF        // protectia la citire a EEPROM: ON / OFF
#pragma config WRT = OFF        // scrierea memoriei Flash din program: OFF sau zone permise
#pragma config CP = OFF         // protectia la citire a memoriei Flash: ON / OFF
#pragma config DEBUG = OFF      // depanare in circuit: ON ocupa RB6 / RB7

extern void uart_init(void);
extern void buzzer_init(void);
extern void hmi_init(void);
extern void spi_init(void);
extern void uart_init(void);
extern void hmi_update(void);
extern void uart_rx_isr(void);
extern void uart_tx_isr(void);
// Initializeaza perifericele, Timer0 (tick-ul de 1 ms) si intreruperile
void system_init(void);

#endif	/* INIT_H */
