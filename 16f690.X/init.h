/*
 * File:   init.h
 * Author: Cheosea Alexandru Gabriel
 */

#ifndef INIT_H
#define	INIT_H
#include "analog.h"
#include "spi.h"
#include "motor.h"
#include <xc.h>

// Biti de configurare
#pragma config FOSC  = INTRCIO  // oscilator intern (8 MHz din OSCCON), RA4 / RA5 = I/O
#pragma config WDTE  = OFF      // watchdog pornit din program (SWDTEN), dupa system_init
#pragma config PWRTE = ON       // intarziere de ~64 ms la pornire, pentru alimentare
#pragma config MCLRE = ON       // RA3 = MCLR (SW2 + rezistenta de pull-up R23)
#pragma config CP    = OFF      // protectia la citire a memoriei Flash
#pragma config CPD   = OFF      // protectia la citire a EEPROM
#pragma config BOREN = ON       // reset la scaderea tensiunii de alimentare
#pragma config IESO  = OFF      // comutare intern / extern: doar cu ceas extern
#pragma config FCMEN = OFF      // monitorul ceasului extern: doar cu ceas extern

// Watchdog: prescaler propriu (WDTPS) pe LFINTOSC (~31 kHz, necalibrat).
// 0111 = 1:4096 -> ~132 ms nominal. Prescaler-ul din OPTION_REG ramane la
// Timer0 (PSA = 0), deci WDT nu are postscaler suplimentar.
#define WDT_PRESCALER_1_4096 0b0111U

void system_init(void);



#endif	//INIT_H
