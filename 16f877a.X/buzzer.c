#include <xc.h>
#include "buzzer.h"
// Folosita de buzzer_asm.s (nu in intreruperi)
volatile __bank(3) uint8_t temp_var;

unsigned char buzzer_volume_level = 5; // 0-10

// Contoarele buzzerului de parcare (vezi buzzer.h); la pornire: oprit
volatile unsigned char buzzer_on_time = 0;
volatile unsigned char buzzer_off_time = 0;

void buzzer_setup(NoteMuzicale_t nota, unsigned char level_0_to_10) {
    if (level_0_to_10 > 10) level_0_to_10 = 10;
    if (nota >= NUM_NOTES) nota = (NoteMuzicale_t)(NUM_NOTES - 1);
    buzzer_volume_level = level_0_to_10;
    PR2 = (uint8_t)(buzzer_get_note_value(nota)); // PR2: perioada PWM (Timer2 numara 0..PR2)
    buzzer_apply_volume();      // nu porneste si nu opreste sunetul
}

void buzzer_set_step_tone(unsigned char mult_50hz) {
    buzzer_off();
    if (mult_50hz < MIN_MULT_50HZ) {
        mult_50hz = MIN_MULT_50HZ; // limita inferioara a frecventei
    }

    PR2 = (uint8_t)(((5760U + (mult_50hz >> 1)) / mult_50hz) - 1);
    buzzer_apply_volume();
    buzzer_on();
}

void buzzer_set_volume(unsigned char level_0_to_10) {
    buzzer_off();
    if (level_0_to_10 > 10) level_0_to_10 = 10;
    buzzer_volume_level = level_0_to_10;
    buzzer_play_note(RE6, 100);
}


void buzzer_play_note(NoteMuzicale_t index_nota, unsigned int durata_ms) {
    if (index_nota >= NUM_NOTES || durata_ms == 0) return;
    buzzer_off();
    PR2 = (uint8_t)(buzzer_get_note_value(index_nota));
    buzzer_apply_volume();

    di();
    buzzer_ms_ramase = durata_ms;
    ei();
    buzzer_on();
}
void buzzer_apply_volume(void) {
    unsigned int max_vol = PR2 + 1;

    if (buzzer_volume_level == 0) {
        // Sectiune critica: intreruperea Timer0 schimba bitii de mod din CCP1CON
        di();
        CCPR1L = 0;      // CCPR1L: cei 8 biti superiori ai factorului de umplere
        CCP1CON &= 0xCF; // CCP1CON<5:4>: cei 2 biti inferiori ai factorului de umplere
        ei();
    }
    else {
        // Factorul de umplere pe 10 biti: plin = (PR2 + 1) x 4.
        // duty = (PR2 + 1) x nivel / 20, cu / 20 inlocuit de x 3277 >> 16.
        // Constanta stabileste maximul: 819 -> 1/32, 1638 -> 1/16, 3277 -> 1/8
        // (1/8 poate depasi puterea maxima a buzzerului).
        unsigned int duty_10bit = (unsigned int)(((unsigned long)max_vol * buzzer_volume_level * 3277ul) >> 16);
        if(duty_10bit == 0) duty_10bit = 1; // treapta 1 nu este niciodata 0%

        // Sectiune critica: CCP1CON este citit si rescris, iar intreruperea
        // Timer0 ii schimba bitii de mod (buzzer_on / buzzer_off)
        di();
        CCPR1L = (unsigned char)(duty_10bit >> 2);
        CCP1CON = (uint8_t)((CCP1CON & 0xCF) | ((unsigned char)(duty_10bit & 0x03) << 4));
        ei();
    }
}
