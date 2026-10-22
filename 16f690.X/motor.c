/*
 * File:   motor.c
 * Author: Alex
 *
 * Created on June 13, 2026, 7:34 AM
 */


#include <xc.h>
#include "motor.h"



// 4-Step Full-Torque Stepper Sequence mapped to Pins 0, 1, 2, 3
static const uint8_t step_sequence[4] = {0x05, 0x06, 0x0A, 0x09};

// Module-isolated runtime variables
static volatile int8_t step_index = 0;
__bit motor_dir;
static volatile uint16_t t1_reload_val = 0;
static volatile uint8_t motor_running = 0;

void motor_init(void) {
    TRIS_MOTOR &= 0xC0; // Configure RC0, RC1, RC2, RC3, RC4, RC5 as digital outputs
    //RC0 = COIL A+
    //RC1 = COIL A-
    //RC2 = COIL B+
    //RC3 = COIL B-
    //RC4 = ENABLE COIL A
    //RC5 = ENABLE COIL B
    MOTOR_PORT &= 0xC0; // Ensure motor coils are unpowered on bootup
    
    // --- TIMER1 CONFIGURATION ---
    T1CONbits.TMR1CS = 0;       // Clock source: Internal instruction clock (Fosc/4 = 1MHz)
    T1CONbits.T1CKPS = TMR1_PS; // Prescaler 1:4 -> Timer1 Frequency = 250,000 Hz
    T1CONbits.nT1SYNC = 1;      // Do not synchronize external clock
    
    // --- INTERRUPT CONFIGURATION ---
    PIR1bits.TMR1IF = 0;    // Clear flag initially
    PIE1bits.TMR1IE = 1;    // Enable Timer1 overflow interrupt
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts
    //INTCONbits.GIE = 1;     // Enable global interrupts
    
    T1CONbits.TMR1ON = 1;   // Fire up Timer1
    motor_dir = DIR_CW;
}
void motor_set_speed(uint16_t steps_per_second, uint8_t direction) {
    // Lower threshold forced by the physical bounds of a 16-bit register running at 500kHz
    if (steps_per_second < MIN_SPS) { 
        motor_running = 0;
        //MOTOR_PORT &= 0xC0; // De-energize H-bridges to prevent standby heat buildup
        return;
    }
    
    // Upper threshold restriction requested to keep the stepper reliable under load
    if (steps_per_second > MAX_SPS) {
        steps_per_second = MAX_SPS;
    }
    
    motor_dir = direction;
    
    // Mathematical mapping: Timer Ticks = TMR1_FREQ / Steps Per Second
    uint16_t ticks = TMR1_FREQ / steps_per_second;
    
    // Calculate the hardware counting preload register value
    t1_reload_val = (uint16_t)(65536 - ticks);
    MOTOR_PORT |= 0b00110000; //Set L293D enable lines high
    motor_running = 1;
}

void motor_isr_handler(void) {
    // Immediately reload both 8-bit registers to preserve exact cycle counts
    TMR1H = (uint8_t)(t1_reload_val >> 8);
    TMR1L = (uint8_t)(t1_reload_val & 0x00FF);

    if (motor_running) {
        // Index shifting according to rotational trajectory
        if (motor_dir == DIR_CW) {
            step_index++;
            if (step_index > 3) step_index = 0;
        } else {
            step_index--;
            if (step_index < 0) step_index = 3;
        }

        // Push sequence onto the port pins without overriding pins RC4 to RC7
        MOTOR_PORT = (MOTOR_PORT & 0xF0) | step_sequence[step_index];
    }

    PIR1bits.TMR1IF = 0; // Handshake completed; clear interrupt trigger

}