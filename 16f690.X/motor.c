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
//static const uint8_t step_sequence[4] = {0x05, 0x01, 0x03, 0x02}; //4-step full-torque sequence
//static const uint8_t step_sequence[8] = {0x05, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08}; //8-step half-torque sequence
// Module-isolated runtime variables
static volatile int8_t step_index = 0;
__bit motor_dir;
static volatile uint16_t t1_reload_val = 0;
static volatile uint8_t motor_running = 0;
volatile uint16_t motor_target_steps_per_second = 0;
static volatile uint16_t motor_current_sps = 0;
static volatile uint8_t motor_target_direction = DIR_CW;
static uint8_t timer2_interrupt_count = 0;
volatile uint8_t motor_speed_increment = MOTOR_SPEED_INCREMENT_DEFAULT;

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
    T1CONbits.TMR1CS = 0;       // Clock source: Fosc/4 = 2MHz at 8MHz Fosc
    T1CONbits.T1CKPS = TMR1_PS; // Prescaler 1:8 -> Timer1 Frequency = 250,000 Hz
    T1CONbits.nT1SYNC = 1;      // Do not synchronize external clock
    
    // --- INTERRUPT CONFIGURATION ---
    PIR1bits.TMR1IF = 0;    // Clear flag initially
    PIE1bits.TMR1IE = 1;    // Enable Timer1 overflow interrupt
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts
    //INTCONbits.GIE = 1;     // Enable global interrupts

    // Timer2 runs at 10 ms intervals; five interrupts make one 50 ms ramp tick.
    T2CON = 0;
    PR2 = 249;
    TMR2 = 0;
    T2CONbits.T2CKPS = 0b10; // Prescaler 1:16
    T2CONbits.TOUTPS = 0b0100; // Postscaler 1:5
    PIR1bits.TMR2IF = 0;
    PIE1bits.TMR2IE = 1;
    T2CONbits.TMR2ON = 1;
    
    T1CONbits.TMR1ON = 1;   // Fire up Timer1
    motor_dir = DIR_CW;
}

inline void motor_set_target_speed(uint16_t steps_per_second, uint8_t direction) {
    uint8_t global_interrupts_enabled = INTCONbits.GIE;
    INTCONbits.GIE = 0;

    motor_target_steps_per_second = steps_per_second;
    motor_target_direction = direction;

    if (global_interrupts_enabled) {
        INTCONbits.GIE = 1;
    }
}

void motor_set_speed(uint16_t steps_per_second, uint8_t direction) {
    // Lower threshold is constrained by Timer1's 16-bit range at 250kHz.
    if (steps_per_second < MIN_SPS) { 
        motor_running = 0;
        motor_current_sps = 0;
        //MOTOR_PORT &= 0xC0; // De-energize H-bridges to prevent standby heat buildup
        return;
    }
    
    // Upper threshold restriction requested to keep the stepper reliable under load
    if (steps_per_second > MAX_SPS) {
        steps_per_second = MAX_SPS;
    }
    
    motor_dir = direction;
    motor_current_sps = steps_per_second;
    
    // Mathematical mapping: Timer Ticks = TMR1_FREQ / Steps Per Second
    uint16_t ticks = TMR1_FREQ / steps_per_second;
    
    // Calculate the hardware counting preload register value
    t1_reload_val = (uint16_t)(65536 - ticks);
    MOTOR_PORT |= 0b00110000; //Set L293D enable lines high
    motor_running = 1;
}

void motor_ramp_isr_handler(void) {
    PIR1bits.TMR2IF = 0;

//    if (++timer2_interrupt_count < 5) {
//        return;
//    }
//    timer2_interrupt_count = 0;

    uint16_t target_sps = motor_target_steps_per_second;
    if (target_sps < MIN_SPS) {
        target_sps = 0;
    } else if (target_sps > MAX_SPS) {
        target_sps = MAX_SPS;
    }

    if (motor_current_sps > 0 && motor_dir != motor_target_direction) {
        target_sps = 0;
    } else if (motor_current_sps == 0 && motor_dir != motor_target_direction) {
        motor_dir = motor_target_direction;
    }

    if (motor_current_sps == target_sps) {
        return;
    }

    if (motor_speed_increment == 0) {
        return;
    }

    uint16_t next_sps;
    if (motor_current_sps < target_sps) {
        uint16_t difference = target_sps - motor_current_sps;
        next_sps = difference <= motor_speed_increment
            ? target_sps
            : motor_current_sps + motor_speed_increment;
    } else {
        uint16_t difference = motor_current_sps - target_sps;
        next_sps = difference <= motor_speed_increment
            ? target_sps
            : motor_current_sps - motor_speed_increment;
    }

    motor_set_speed(next_sps, motor_dir);
    // Preserve sub-MIN_SPS ramp progress while the motor output remains stopped.
    motor_current_sps = next_sps;
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