#ifndef MOTOR_H
#define	MOTOR_H

#include <stdint.h>

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000
#endif
// L293D pe PORTC: RC0-RC3 = intrarile 1A-4A, RC4 / RC5 = EN1 / EN2
#define MOTOR_PORT  PORTC
#define TRIS_MOTOR  TRISC

#define DIR_CW   1
#define DIR_CCW  0

// Prescaler Timer1 (T1CKPS)
#define PS_1_1      0x0
#define PS_1_2      0x1
#define PS_1_4      0x2
#define PS_1_8      0x3
#define TMR1_PS     PS_1_2 // trebuie sa corespunda impartitorului din TMR1_FREQ
// Frecventa Timer1 = Fosc / 4 / prescaler; perioada pasului = TMR1_FREQ / pasi pe secunda
#define TMR1_FREQ   (_XTAL_FREQ/4/2UL)
#define MIN_SPS 20      // sub aceasta viteza motorul sta
#define MAX_SPS 5000
// Rampa: pasi/s adaugati / scazuti la fiecare tick Timer2, deci acceleratia =
// valoare / tick. Tick = Fosc / 4 / prescaler / (PR2 + 1) / postscaler (10 ms,
// motor_init). Ambele > 0.
#define MOTOR_SPD_INC 15U   // accelerare
#define MOTOR_SPD_DEC 40U   // franare (si la schimbarea sensului)

extern volatile uint16_t motor_target_steps_per_second;
extern volatile uint16_t motor_current_sps;

#define motor_off() (MOTOR_PORT &= 0xC0) // bobine oprite (nefolosit)
#define motor_on()  (MOTOR_PORT |= 0x30) // bobine alimentate, motorul tine pozitia (nefolosit)

// Pinii motorului, Timer1 + CCP1 (perioada pasului) si Timer2 (tick-ul rampei)
void motor_init(void);

// Seteaza imediat viteza si sensul (fara rampa), limitata la MAX_SPS. Sub
// MIN_SPS motorul sta, dar valoarea ramane in motor_current_sps pentru rampa.
// Doar din main (impartire pe 32 de biti).
void motor_set_speed(uint16_t steps_per_second, uint8_t direction);

// Viteza si sensul tinta; rampa se apropie de ele (trece prin 0 la schimbarea sensului)
inline void motor_set_target_speed(uint16_t steps_per_second, uint8_t direction);

// Pasul motorului, din ISR la CCP1IF (hardware-ul reseteaza Timer1 la potrivire)
void motor_isr_handler(void);
// Un pas de rampa la fiecare tick Timer2 (TMR2IF, prin polling). Doar din main.
void motor_ramp_task(void);
void motor_hold();
#endif	/* MOTOR_H */
