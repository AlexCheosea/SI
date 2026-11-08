/*
 * File:   motor.c
 * Author: Alex
 *
 */


#include <xc.h>
#include "motor.h"



// 4-Step Full-Torque Stepper Sequence mapped to Pins 0, 1, 2, 3
// In RAM (nu const): citirea prin FSR costa ~5 cicluri, fata de ~12 pentru
// apelul tabelei din memoria de program (stringdir) in ISR.
static uint8_t step_sequence[4] = {0x05, 0x06, 0x0A, 0x09};
//static const uint8_t step_sequence[4] = {0x05, 0x01, 0x03, 0x02}; //4-step full-torque sequence
//static const uint8_t step_sequence[8] = {0x05, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08}; //8-step half-torque sequence
// Module-isolated runtime variables
static uint8_t step_index = 0;  // 0..3, folosit doar in ISR
__bit motor_dir;
// Perioada pasului pentru CCP1 (special event): Timer1 numara 0..CCPR1, deci
// perioada = CCPR1 + 1 tick-uri. Scrisa de main, copiata in CCPR1 de ISR.
static volatile uint16_t ccp1_period = 0xFFFF;
static volatile uint8_t motor_running = 0;
volatile uint16_t motor_target_steps_per_second = 0;
volatile uint16_t motor_current_sps = 0;
static volatile uint8_t motor_target_direction = DIR_CW;
static uint8_t timer2_interrupt_count = 0;

void motor_init(void) {
    ANSEL &= 0x0F;      // ANS4-ANS7 (RC0-RC3) digitale: altfel citirea PORTC da 0
                        // si orice read-modify-write pe PORTC opreste bobinele
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
    T1CONbits.T1CKPS = TMR1_PS; // Prescaler 1:2 -> Timer1 Frequency = 1 MHz
    // (T1SYNC este ignorat cand TMR1CS = 0: Timer1 e sincron cu FOSC, cum cere
    //  Special Event Trigger-ul)
    TMR1H = 0;
    TMR1L = 0;

    // --- CCP1: Compare, Special Event Trigger ---
    // La TMR1 == CCPR1 hardware-ul reseteaza TMR1 (perioada exacta, fara
    // reload software) si seteaza CCP1IF. Pinul CCP1 (RC5) nu este controlat
    // de modul in acest regim, iar P1B-P1D raman pini de port (DS40001262F,
    // Register 11-1 / 11.2.4). ADC-ul nu e pornit, deci nu porneste conversii.
    CCPR1H = (uint8_t)(ccp1_period >> 8);
    CCPR1L = (uint8_t)ccp1_period;
    CCP1CON = 0x0B;             // P1M=00, CCP1M=1011

    // --- INTERRUPT CONFIGURATION ---
    PIE1bits.TMR1IE = 0;    // Timer1 nu mai face overflow; folosim CCP1IF
    PIR1bits.TMR1IF = 0;
    PIR1bits.CCP1IF = 0;    // Clear flag initially
    PIE1bits.CCP1IE = 1;    // Enable CCP1 compare interrupt
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts (GIE: in system_init)

    // Timer2: 2 MHz / 16 / 250 / 5 = 100 Hz -> TMR2IF la fiecare 10 ms (tick-ul
    // rampei). Nu genereaza intrerupere; flag-ul e verificat in motor_ramp_task.
    T2CON = 0;
    PR2 = 249;
    TMR2 = 0;
    T2CONbits.T2CKPS = 0b10; // Prescaler 1:16
    T2CONbits.TOUTPS = 0b0100; // Postscaler 1:5
    PIR1bits.TMR2IF = 0;
    PIE1bits.TMR2IE = 0;     // fara intrerupere, ca sa nu intarzie SPI
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

// Apelata DOAR din main loop (nu din ISR): impartirea pe 32 de biti dureaza
// ~0.6-0.8 ms si ar intarzia raspunsul SPI daca ar rula in intrerupere.
void motor_set_speed(uint16_t steps_per_second, uint8_t direction) {
    uint8_t global_interrupts_enabled;

    // Sub MIN_SPS motorul sta (Timer1 pe 16 biti la 1 MHz ar permite minim
    // ~16 pasi/s).
    if (steps_per_second < MIN_SPS) {
        global_interrupts_enabled = INTCONbits.GIE;
        INTCONbits.GIE = 0;
        motor_running = 0;
        // Pastram valoarea sub MIN_SPS ca rampa sa nu-si piarda progresul
        // cat timp iesirea motorului este oprita.
        motor_current_sps = steps_per_second;
        if (global_interrupts_enabled) {
            INTCONbits.GIE = 1;
        }
        //MOTOR_PORT &= 0xC0; // De-energize H-bridges to prevent standby heat buildup
        return;
    }

    // Upper threshold restriction requested to keep the stepper reliable under load
    if (steps_per_second > MAX_SPS) {
        steps_per_second = MAX_SPS;
    }

    // Calculul greu se face cu intreruperile ACTIVE.
    // Mathematical mapping: Timer Ticks = TMR1_FREQ / Steps Per Second
    uint16_t ticks = (uint16_t)(TMR1_FREQ / steps_per_second);
    // Timer1 numara 0..CCPR1 => perioada = CCPR1 + 1
    uint16_t period = ticks - 1U;

    // Sectiune critica scurta: ccp1_period si motor_current_sps sunt pe
    // 16 biti si sunt citite din ISR (CCP1 / SPI), iar PORTC este modificat
    // read-modify-write si de motor_isr_handler.
    global_interrupts_enabled = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    motor_dir = direction;
    motor_current_sps = steps_per_second;
    ccp1_period = period;
    MOTOR_PORT |= 0b00110000; //Set L293D enable lines high
    motor_running = 1;
    if (global_interrupts_enabled) {
        INTCONbits.GIE = 1;
    }
}

// Pasul de rampa, apelat din main loop. Timer2 seteaza TMR2IF la fiecare 10 ms;
// flag-ul ramane setat pana il stergem, deci un tick nu se pierde (doua tick-uri
// neprocesate se contopesc). La fiecare tick viteza se apropie de tinta cu
// MOTOR_SPD_INC pasi/s cand accelereaza si cu MOTOR_SPD_DEC cand franeaza;
// la schimbarea sensului franeaza intai pana la 0.
void motor_ramp_task(void) {
    if (!PIR1bits.TMR2IF) {
        return;
    }
    PIR1bits.TMR2IF = 0;

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

    uint16_t next_sps;
    if (motor_current_sps < target_sps) {
        // Accelerare
        uint16_t difference = target_sps - motor_current_sps;
        next_sps = difference <= MOTOR_SPD_INC
            ? target_sps
            : motor_current_sps + MOTOR_SPD_INC;
    } else {
        // Decelerare
        uint16_t difference = motor_current_sps - target_sps;
        next_sps = difference <= MOTOR_SPD_DEC
            ? target_sps
            : motor_current_sps - MOTOR_SPD_DEC;
    }

    // motor_set_speed pastreaza si progresul sub MIN_SPS in motor_current_sps.
    motor_set_speed(next_sps, motor_dir);
}

// CCP1 ISR: TMR1 a fost deja resetat de hardware la potrivire.
void motor_isr_handler(void) {
    PIR1bits.CCP1IF = 0;
    // Noua perioada se scrie DUPA resetul TMR1 (cel putin un tick de 1 us dupa
    // potrivire; intrarea in ISR dureaza mai mult), deci nu poate anula resetul
    // (DS40001262F 11.2.4 Nota 2).
    // Peste ~3900 SPS perioada are CCPR1H = 0, asa ca o scriere H-apoi-L poate
    // lasa temporar CCPR1 sub TMR1 (ex. 0x0105 -> 0x00F0 trece prin 0x0005) =>
    // potrivire ratata si un pas de ~65 ms. Punem intai CCPR1H = 0xFF: toate
    // valorile intermediare sunt >= 0xFF00. Valoarea finala (min. 199) ramane
    // peste TMR1, care aici este cel mult ~60.
    CCPR1H = 0xFF;
    CCPR1L = (uint8_t)ccp1_period;
    CCPR1H = (uint8_t)(ccp1_period >> 8);

    if (motor_running) {
        // Index shifting according to rotational trajectory (wrap prin masca)
        if (motor_dir == DIR_CW) {
            step_index++;
        } else {
            step_index--;
        }
        step_index &= 0x03;

        // Push sequence onto the port pins without overriding pins RC4 to RC7
        MOTOR_PORT = (MOTOR_PORT & 0xF0) | step_sequence[step_index];
    }
}