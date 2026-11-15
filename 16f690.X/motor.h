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
#define MAX_SPS 2667    // 800 RPM (200 pasi/rotatie); acelasi plafon ca la master
// Mod pozitie: viteza de apropiere (pasi/s). Franarea se termina la ea, iar
// ultimii pasi pana la tinta se fac cu ea; motorul se opreste brusc la tinta,
// deci trebuie sa fie sub viteza la care motorul porneste / se opreste fara
// sa piarda pasi. Minim MIN_SPS.
#define POSITION_CREEP_SPS 100U
// Rezerva la franare: atatea tick-uri de rampa (10 ms) parcurse la viteza
// curenta, adunate la distanta de franare (intarzieri, rotunjiri)
#define POSITION_BRAKE_MARGIN_TICKS 2U
// Rampa: pasi/s adaugati / scazuti la fiecare tick Timer2, deci acceleratia =
// valoare / tick. Tick = Fosc / 4 / prescaler / (PR2 + 1) / postscaler (10 ms,
// motor_init). Setate prin SPI (CMD_SET_MOTOR_RAMP); ambele > 0.
#define MOTOR_ACCEL_DEFAULT 15U   // accelerare
#define MOTOR_DECEL_DEFAULT 40U   // franare (si la schimbarea sensului)

extern volatile uint16_t motor_target_steps_per_second;
extern volatile uint16_t motor_current_sps;

// Pozitia (pasi, DIR_CW = +1). Scrisa doar in ISR-ul motorului; octetii
// separati permit citirea rapida din ISR-ul SPI. Din main: cu intreruperile oprite.
typedef union {
    int32_t v;
    uint8_t b[4];
} motor_position_t;
extern volatile motor_position_t motor_position;
// Scrise in ISR-ul SPI (un octet: citire atomica)
extern volatile uint8_t motor_accel;
extern volatile uint8_t motor_decel;

#define motor_off() (MOTOR_PORT &= 0xC0) // bobine oprite (nefolosit)
#define motor_on()  (MOTOR_PORT |= 0x30) // bobine alimentate, motorul tine pozitia (nefolosit)

// Pinii motorului, Timer1 + CCP1 (perioada pasului) si Timer2 (tick-ul rampei)
void motor_init(void);

// Seteaza imediat viteza si sensul (fara rampa), limitata la MAX_SPS. Sub
// MIN_SPS motorul sta, dar valoarea ramane in motor_current_sps pentru rampa.
// Doar din main (impartire pe 32 de biti).
void motor_set_speed(uint16_t steps_per_second, uint8_t direction);

// Mod viteza: viteza si sensul tinta; rampa se apropie de ele (trece prin 0
// la schimbarea sensului). Iese din modul pozitie.
inline void motor_set_target_speed(uint16_t steps_per_second, uint8_t direction);

// Mod pozitie: viteza maxima spre tinta
void motor_set_speed_limit(uint16_t steps_per_second);
// Mod pozitie: tinta absoluta (pasi); intra in modul pozitie
void motor_set_position_target(int32_t target);

// Pasul motorului, din ISR la CCP1IF (hardware-ul reseteaza Timer1 la potrivire)
void motor_isr_handler(void);
// Un pas de rampa la fiecare tick Timer2 (TMR2IF, prin polling). Doar din main.
// force_stop != 0: franare pana la 0 in ambele moduri (legatura SPI pierduta).
void motor_ramp_task(uint8_t force_stop);
void motor_hold();
#endif	/* MOTOR_H */
