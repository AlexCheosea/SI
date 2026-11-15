/*
 * File:   motor.c
 * Author: Alex
 *
 */


#include <xc.h>
#include "motor.h"



// Secventa full-step pe RC0-RC3. In RAM: citire mai rapida in ISR decat din memoria program.
static uint8_t step_sequence[4] = {0x05, 0x06, 0x0A, 0x09};
static uint8_t step_index = 0;  // doar in ISR
__bit motor_dir;
// Perioada pasului pentru CCPR1 (Timer1 numara 0..CCPR1, deci perioada - 1).
// Scrisa de main, copiata in CCPR1 de ISR.
static volatile uint16_t ccp1_period = 0xFFFF;
static volatile uint8_t motor_running = 0;
volatile uint16_t motor_target_steps_per_second = 0;
volatile uint16_t motor_current_sps = 0;
volatile uint8_t motor_accel = MOTOR_ACCEL_DEFAULT;
volatile uint8_t motor_decel = MOTOR_DECEL_DEFAULT;
static volatile uint8_t motor_target_direction = DIR_CW;
static uint8_t timer2_interrupt_count = 0;

// Mod pozitie. pos_mode si pos_target sunt citite in ISR, scrise din main
// cu intreruperile oprite; restul doar in main.
volatile motor_position_t motor_position;
static volatile int32_t pos_target = 0;
static __bit pos_mode;              // 0 = mod viteza, 1 = mod pozitie
static __bit pos_braking;           // franarea spre tinta a inceput
static uint16_t pos_speed_limit = 0;

void motor_init(void) {
    ANSEL &= 0x0F;      // ANSEL: 1 = pin analogic, 0 = digital (pinii analogici se citesc 0 in PORTC)
    TRIS_MOTOR &= 0xC0; // TRIS: 1 = intrare, 0 = iesire
    MOTOR_PORT &= 0xC0; // bobine oprite la pornire

    // Timer1: baza de timp a pasilor
    T1CONbits.TMR1CS = 0;       // sursa Timer1: 0 = Fosc/4, 1 = extern (T1CKI / oscilator Timer1)
    T1CONbits.T1CKPS = TMR1_PS; // prescaler Timer1: 1:1, 1:2, 1:4, 1:8
    // Special Event Trigger cere Timer1 sincron cu Fosc (T1SYNC e ignorat cu Fosc/4)
    TMR1H = 0;
    TMR1L = 0;

    // CCP1 in Special Event Trigger: la TMR1 == CCPR1 hardware-ul reseteaza
    // TMR1 si seteaza CCP1IF; pinii CCP1 / P1x raman pini de port.
    CCPR1H = (uint8_t)(ccp1_period >> 8);
    CCPR1L = (uint8_t)ccp1_period;
    CCP1CON = 0x0B;             // CCP1CON: P1M = iesirile PWM, CCP1M = modul (captura, comparare, special event, PWM)

    PIE1bits.TMR1IE = 0;    // perioada vine din CCP1IF, nu din overflow
    PIR1bits.TMR1IF = 0;
    PIR1bits.CCP1IF = 0;
    PIE1bits.CCP1IE = 1;
    INTCONbits.PEIE = 1;    // GIE se activeaza in system_init

    // Timer2: tick-ul rampei, TMR2IF verificat prin polling (fara intrerupere,
    // ca sa nu intarzie SPI-ul)
    T2CON = 0;
    PR2 = 249;                 // PR2: perioada Timer2 (TMR2 numara 0..PR2)
    TMR2 = 0;
    T2CONbits.T2CKPS = 0b10;   // prescaler Timer2: 1:1, 1:4, 1:16
    T2CONbits.TOUTPS = 0b0100; // postscaler Timer2: 1:1 .. 1:16 (TMR2IF la fiecare N perioade)
    PIR1bits.TMR2IF = 0;
    PIE1bits.TMR2IE = 0;
    T2CONbits.TMR2ON = 1;

    T1CONbits.TMR1ON = 1;
    motor_dir = DIR_CW;
}

inline void motor_set_target_speed(uint16_t steps_per_second, uint8_t direction) {
    uint8_t global_interrupts_enabled = INTCONbits.GIE;
    INTCONbits.GIE = 0;

    motor_target_steps_per_second = steps_per_second;
    motor_target_direction = direction;
    pos_mode = 0;

    if (global_interrupts_enabled) {
        INTCONbits.GIE = 1;
    }
}

void motor_set_speed_limit(uint16_t steps_per_second) {
    pos_speed_limit = steps_per_second;
}

void motor_set_position_target(int32_t target) {
    uint8_t global_interrupts_enabled = INTCONbits.GIE;
    INTCONbits.GIE = 0;

    pos_target = target;
    pos_mode = 1;

    if (global_interrupts_enabled) {
        INTCONbits.GIE = 1;
    }
    pos_braking = 0;    // franarea se reevalueaza pentru tinta noua
}

// Franare spre tinta: distanta pana la oprire de la viteza v, cu deceleratia
// a = decel pasi/s la fiecare 10 ms = 100 x decel pasi/s^2, este v^2 / (2a).
// Se franeaza cand distanta <= v^2 / (2a) + rezerva (POSITION_BRAKE_MARGIN_TICKS
// tick-uri la viteza v), fara impartiri:
//   200 x decel x distanta <= v^2 + 2 x ticks x decel x v
static uint8_t position_must_brake(uint32_t distance, uint16_t v) {
    uint8_t decel = motor_decel;
    uint32_t lhs, rhs;

    // Peste orice distanta de franare posibila (v <= MAX_SPS, decel >= 1):
    // 2667^2 / 200 ~ 35600 pasi. Limita tine si produsul pe 32 de biti.
    if (distance > 65535UL) {
        return 0;
    }
    lhs = (uint32_t)(200U * decel) * distance;
    rhs = (uint32_t)v * v +
          (uint32_t)(2U * POSITION_BRAKE_MARGIN_TICKS * decel) * v;
    return (uint8_t)(lhs <= rhs);
}

// Doar din main: impartirea pe 32 de biti ar intarzia SPI-ul daca ar rula in ISR
void motor_set_speed(uint16_t steps_per_second, uint8_t direction) {
    uint8_t global_interrupts_enabled;

    // Sub MIN_SPS motorul sta (perioada nu ar mai incapea in Timer1)
    if (steps_per_second < MIN_SPS) {
        global_interrupts_enabled = INTCONbits.GIE;
        INTCONbits.GIE = 0;
        motor_running = 0;
        // Valoarea ramane, ca rampa sa nu-si piarda progresul
        motor_current_sps = steps_per_second;
        if (global_interrupts_enabled) {
            INTCONbits.GIE = 1;
        }
        return;
    }

    // Limita superioara: pasi fiabili sub sarcina
    if (steps_per_second > MAX_SPS) {
        steps_per_second = MAX_SPS;
    }

    // Impartirea se face cu intreruperile active
    uint16_t ticks = (uint16_t)(TMR1_FREQ / steps_per_second);
    uint16_t period = ticks - 1U; // Timer1 numara 0..CCPR1

    // Sectiune critica: valori pe 16 biti citite in ISR, iar PORTC este
    // modificat read-modify-write si in motor_isr_handler
    global_interrupts_enabled = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    motor_dir = direction;
    motor_current_sps = steps_per_second;
    ccp1_period = period;
    MOTOR_PORT |= 0b00110000; // liniile EN ale L293D
    motor_running = 1;
    if (global_interrupts_enabled) {
        INTCONbits.GIE = 1;
    }
}

// Un pas de rampa la fiecare TMR2IF (flag-ul ramane setat, deci un tick nu se
// pierde). La schimbarea sensului franeaza intai pana la 0.
// Mod pozitie: sensul spre tinta, viteza maxima pana incepe franarea, apoi
// viteza de apropiere pana la tinta (unde ISR-ul se opreste exact).
void motor_ramp_task(uint8_t force_stop) {
    uint16_t target_sps;
    uint8_t target_dir;

    if (!PIR1bits.TMR2IF) {
        return;
    }
    PIR1bits.TMR2IF = 0;

    if (pos_mode) {
        int32_t remaining;
        uint32_t distance;
        uint8_t global_interrupts_enabled = INTCONbits.GIE;

        INTCONbits.GIE = 0;
        remaining = pos_target - motor_position.v;
        if (global_interrupts_enabled) {
            INTCONbits.GIE = 1;
        }

        if (remaining == 0) {
            // La tinta: ISR-ul nu mai face pasi, viteza se anuleaza direct
            pos_braking = 0;
            if (motor_current_sps != 0U) {
                motor_set_speed(0, motor_dir);
            }
            return;
        }
        if (remaining > 0) {
            target_dir = DIR_CW;
            distance = (uint32_t)remaining;
        } else {
            target_dir = DIR_CCW;
            distance = (uint32_t)0 - (uint32_t)remaining;
        }

        target_sps = pos_speed_limit;
        if (motor_dir != target_dir) {
            pos_braking = 0;    // tinta in spate: mai jos se franeaza pana la 0
        } else if (!pos_braking && motor_current_sps > 0U &&
                   position_must_brake(distance, motor_current_sps)) {
            pos_braking = 1;
        }
        if (pos_braking && target_sps > POSITION_CREEP_SPS) {
            target_sps = POSITION_CREEP_SPS;
        }
    } else {
        target_sps = motor_target_steps_per_second;
        target_dir = motor_target_direction;
    }

    if (force_stop) {
        target_sps = 0;
    }
    if (target_sps < MIN_SPS) {
        target_sps = 0;
    } else if (target_sps > MAX_SPS) {
        target_sps = MAX_SPS;
    }

    if (motor_current_sps > 0 && motor_dir != target_dir) {
        target_sps = 0;
    } else if (motor_current_sps == 0 && motor_dir != target_dir) {
        motor_dir = target_dir;
    }

    if (motor_current_sps == target_sps) {
        return;
    }

    uint16_t next_sps;
    if (motor_current_sps < target_sps) {
        // Accelerare (pas citit o data: il scrie ISR-ul SPI)
        uint8_t accel = motor_accel;
        uint16_t difference = target_sps - motor_current_sps;
        next_sps = difference <= accel
            ? target_sps
            : motor_current_sps + accel;
    } else {
        // Franare
        uint8_t decel = motor_decel;
        uint16_t difference = motor_current_sps - target_sps;
        next_sps = difference <= decel
            ? target_sps
            : motor_current_sps - decel;
    }

    motor_set_speed(next_sps, motor_dir);
}

// ISR CCP1: TMR1 a fost deja resetat de hardware la potrivire
void motor_isr_handler(void) {
    PIR1bits.CCP1IF = 0;
    // Perioada noua se scrie dupa resetul TMR1, deci nu il poate anula.
    // CCPR1H = 0xFF intai: altfel, intre scrierea lui L si H, CCPR1 poate
    // trece temporar sub TMR1 si potrivirea se rateaza (Timer1 face un tur complet).
    CCPR1H = 0xFF;
    CCPR1L = (uint8_t)ccp1_period;
    CCPR1H = (uint8_t)(ccp1_period >> 8);

    if (motor_running) {
        // Mod pozitie: la tinta nu se mai face niciun pas (oprire exacta,
        // indiferent cand ajunge rampa din main sa anuleze viteza)
        if (pos_mode && motor_position.v == pos_target) {
            return;
        }
        if (motor_dir == DIR_CW) {
            step_index++;
            motor_position.v++;
        } else {
            step_index--;
            motor_position.v--;
        }
        step_index &= 0x03; // index circular 0..3

        // Doar RC0-RC3; RC4-RC7 raman neschimbati
        MOTOR_PORT = (MOTOR_PORT & 0xF0) | step_sequence[step_index];
    }
}
