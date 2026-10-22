#ifndef MOTOR_H
#define	MOTOR_H

#include <stdint.h>

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000 // Assumed 8MHz Internal Clock Speed
#endif
// --- HARDWARE CONFIGURATION ---
// Port C pins connected to L293D (RC0=1A, RC1=2A, RC2=3A, RC3=4A)
#define MOTOR_PORT  PORTC
#define TRIS_MOTOR  TRISC

// Direction Constants
#define DIR_CW   1
#define DIR_CCW  0

// Constants: (Change these when changing operating freq))
#define PS_1_1      0x0
#define PS_1_2      0x1
#define PS_1_4      0x2
#define PS_1_8      0x3
#define TMR1_PS     PS_1_8 //must match divisor below
#define TMR1_FREQ   (_XTAL_FREQ/4/8UL)
#define MIN_SPS 5
#define MAX_SPS 1000

// -- Macros
#define motor_off() (MOTOR_PORT &= 0xC0) //Turn coils off
#define motor_on()  (MOTOR_PORT |= 0x30) //Turn coils on (motor holds position)

// --- FUNCTION PROTOTYPES ---

/**
 * @brief Configures RC0-RC3 as digital outputs and initializes Timer1 
 * with a static 1:4 prescaler for accurate background stepping.
 */
void motor_init(void);

/**
 * @brief Sets the background stepping speed and direction.
 * Automatically clips the speed between 4 and 1000 steps/second.
 * @param steps_per_second Target speed (4 to 1000). Pass 0 to STOP.
 * @param direction DIR_CW or DIR_CCW
 */
void motor_set_speed(uint16_t steps_per_second, uint8_t direction);

/**
 * @brief Background step sequencer. Must be called inside the high-priority 
 * global __interrupt() ISR block in your main.c file.
 */
void motor_isr_handler(void);
void motor_hold();
#endif	/* MOTOR_H */