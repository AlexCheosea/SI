#ifndef MOTOR_H
#define	MOTOR_H

#include <stdint.h>

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000 // Assumed 8MHz Internal Clock Speed
#endif
// --- HARDWARE CONFIGURATION ---
// Port C pins connected to L293D (RC0=1A, RC1=2A, RC2=3A, RC3=4A,
// RC4 = EN1 (coil A enable), RC5 = EN2 (coil B enable))
#define MOTOR_PORT  PORTC
#define TRIS_MOTOR  TRISC

// Direction Constants
#define DIR_CW   1
#define DIR_CCW  0

// Constants (change these when changing the operating frequency)
#define PS_1_1      0x0
#define PS_1_2      0x1
#define PS_1_4      0x2
#define PS_1_8      0x3
#define TMR1_PS     PS_1_2 //must match divisor below
#define TMR1_FREQ   (_XTAL_FREQ/4/2UL)
#define MIN_SPS 20      // sub aceasta viteza motorul sta
#define MAX_SPS 5000
// Rampa de viteza: pasi/s adaugati / scazuti la fiecare tick de 10 ms
// (x100 = pasi/s^2). Ambele trebuie sa fie > 0.
#define MOTOR_SPD_INC 15U   // accelerare (viteza creste spre tinta)
#define MOTOR_SPD_DEC 40U   // decelerare (franare, inclusiv la schimbarea sensului)

extern volatile uint16_t motor_target_steps_per_second;
extern volatile uint16_t motor_current_sps;

// -- Macros
#define motor_off() (MOTOR_PORT &= 0xC0) //Turn coils off (currently unused)
#define motor_on()  (MOTOR_PORT |= 0x30) //Turn coils on (motor holds position; currently unused)

// --- FUNCTION PROTOTYPES ---

/**
 * @brief Configures RC0-RC5 as digital outputs, Timer1 (1:2 prescaler, 1 MHz)
 * with CCP1 in Special Event Trigger mode as the step period, and Timer2 as
 * the 10 ms ramp tick.
 */
void motor_init(void);

/**
 * @brief Sets the stepping speed and direction immediately (no ramp).
 * Clips the speed to MAX_SPS. Values below MIN_SPS stop the motor but are kept
 * in motor_current_sps, so the ramp keeps its progress. Call it from the main
 * loop only (it contains a 32-bit division).
 * @param steps_per_second Speed in steps per second.
 * @param direction DIR_CW or DIR_CCW
 */
void motor_set_speed(uint16_t steps_per_second, uint8_t direction);

/**
 * @brief Sets the target speed and direction; the 10 ms ramp moves towards it.
 * @param steps_per_second Target speed. Values below MIN_SPS stop the motor.
 * @param direction DIR_CW or DIR_CCW (the ramp goes through 0 to reverse)
 */
inline void motor_set_target_speed(uint16_t steps_per_second, uint8_t direction);

/**
 * @brief Background step sequencer, called from the ISR on CCP1IF.
 * CCP1 runs in Compare/Special Event Trigger mode, so the hardware resets
 * Timer1 at each match and the step period is exact.
 */
void motor_isr_handler(void);
/**
 * @brief Runs one speed-ramp step when Timer2's 10 ms flag (TMR2IF, polled,
 * no interrupt) is set, including the step-period division. Call it from the
 * main loop, never from an ISR.
 */
void motor_ramp_task(void);
void motor_hold();
#endif	/* MOTOR_H */