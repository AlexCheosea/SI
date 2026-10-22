#include <xc.h>
#include "buzzer.h"

unsigned char buzzer_volume_level = 5; // Default: Volum mediu (Treapta 5 din 10)

void buzzer_set_step_tone(unsigned char mult_50hz) {
    buzzer_off();
    if (mult_50hz < MIN_MULT_50HZ) {
        mult_50hz = MIN_MULT_50HZ; // Limita de ~1150 Hz
    }
    
    // Scriem direct în hardware calculul perioadei
    PR2 = (uint8_t)(((5760U + (mult_50hz >> 1)) / mult_50hz) - 1);
    buzzer_apply_volume();
    buzzer_on();
}

void buzzer_set_volume(unsigned char level_0_to_10) {
    buzzer_off(); // Oprim sunetul curent înainte de a schimba volumul
    if (level_0_to_10 > 10) level_0_to_10 = 10;
    buzzer_volume_level = level_0_to_10;
    buzzer_play_note(RE6, 100);
}


void buzzer_play_note(NoteMuzicale_t index_nota, unsigned int durata_ms) {
    if (index_nota >= NUM_NOTES || durata_ms == 0) return;
    buzzer_off(); // Oprim sunetul curent înainte de a seta noua notă și durată
    // 1. Setează nota muzicală în hardware (PWM)
    PR2 = (uint8_t)(buzzer_get_note_value(index_nota)); 
    buzzer_apply_volume();
    
    // 2. Încarcă durata direct în contor
    di();
    buzzer_ms_ramase = durata_ms;
    ei();
    // 3. Pornește sunetul
    buzzer_on();
}
void buzzer_apply_volume(void) {
    // 1. Păstrăm perioada completă pe 8 biți (fără să pierdem rezoluție prin shiftare timpurie)
    unsigned int max_vol = PR2 + 1;
    
    if (buzzer_volume_level == 0) {
        CCPR1L = 0;      // Primii 8 MSB se află în CCPR1L
        CCP1CON &= 0xCF; // Stingem și cei 2 biți de jos (biții 5 și 4)
    } 
    else {
        // Valoarea maximă a duty-cycle-ului pe 10 biți este (PR2 + 1) * 4.
        // Pentru un Duty Cycle maxim de 12.5% (adică 1/8), valoarea la volum maxim trebuie să fie:
        // Target = ((PR2 + 1) * 4) / 8 = (PR2 + 1) / 2.
        //
        // Pentru a include și treptele de volum (1-10), formula ideală este:
        // duty_10bit = (max_vol * buzzer_volume_level) / 20;
        //
        // Pentru a evita împărțirea lentă la 20, folosim trucul cu "constanta magică".
        // 1 / 20 este exact 3276.8 / 65536. Rotunjim la 3277.
        // 819ul -> 1/32
        // 1638ul -> 1/16
        // 3277ul -> 1/8 (posibil sa fie periculos, putere disipata dubla peste cea maxima)
        unsigned int duty_10bit = (unsigned int)(((unsigned long)max_vol * buzzer_volume_level * 3277ul) >> 16);
        if(duty_10bit == 0) duty_10bit = 1; // Asigurăm un minim de volum pentru treapta 1
        
        // 2. Încărcăm cei 10 biți rezultați în registrele hardware ale PIC-ului
        // Cei mai semnificativi 8 biți se duc în CCPR1L (shiftare la dreapta cu 2)
        CCPR1L = (unsigned char)(duty_10bit >> 2);
        
        // Cei 2 biți rămași jos (biții 1 și 0 din duty_10bit) se pun în CCP1CON pe pozițiile 5 și 4
        CCP1CON = (uint8_t)((CCP1CON & 0xCF) | ((unsigned char)(duty_10bit & 0x03) << 4));
    }
}