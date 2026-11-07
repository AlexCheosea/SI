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
#define TMR1_PS     PS_1_2 //must match divisor below
#define TMR1_FREQ   (_XTAL_FREQ/4/2UL)
#define MIN_SPS 20
#define MAX_SPS 3000
#define MOTOR_SPEED_INCREMENT_DEFAULT 10U

extern volatile uint16_t motor_target_steps_per_second;
extern volatile uint8_t motor_speed_increment;

// -- Macros
#define motor_off() (MOTOR_PORT &= 0xC0) //Turn coils off
#define motor_on()  (MOTOR_PORT |= 0x30) //Turn coils on (motor holds position)

// --- FUNCTION PROTOTYPES ---

/**
 * @brief Configures RC0-RC3 as digital outputs and initializes Timer1 
 * with a static 1:8 prescaler for accurate background stepping.
 */
void motor_init(void);

/**
 * @brief Sets the background stepping speed and direction.
 * Automatically clips the speed between MIN_SPS and MAX_SPS steps/second.
 * @param steps_per_second Speed (MIN_SPS to MAX_SPS). Values below MIN_SPS stop the motor.
 * @param direction DIR_CW or DIR_CCW
 */
void motor_set_speed(uint16_t steps_per_second, uint8_t direction);

/**
 * @brief Sets the target speed and direction for the 50 ms speed ramp.
 * @param steps_per_second Target speed. Values below 5 stop the motor.
 * @param direction DIR_CW or DIR_CCW
 */
inline void motor_set_target_speed(uint16_t steps_per_second, uint8_t direction);

/**
 * @brief Background step sequencer. Must be called inside the high-priority 
 * global __interrupt() ISR block in your main.c file.
 */
void motor_isr_handler(void);
/**
 * @brief Timer2 interrupt handler. Updates the speed ramp every 50 ms.
 */
void motor_ramp_isr_handler(void);
void motor_hold();
#endif	/* MOTOR_H */