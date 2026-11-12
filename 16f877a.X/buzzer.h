#ifndef BUZZER_H
#define	BUZZER_H

#define MIN_MULT_50HZ 23
// Funcțiile implementate în fișierul de asamblare (.s)
void buzzer_init(void);
extern void buzzer_on(void);
extern void buzzer_off(void);

volatile unsigned int buzzer_ms_ramase = 0;

// ---------------------------------------------------------------------------
// Buzzer de parcare (bip-uri generate in intreruperea de 1 ms)
// ---------------------------------------------------------------------------
#define BUZZER_TONE      DO7    // nota bip-ului (NoteMuzicale_t), setata la initializare
#define BUZZER_VOLUME    5      // volumul (0-10), setat la initializare
#define BUZZER_BEEP_TIME 6U     // durata unui bip, in unitati de 10 ms
#define BUZZER_MIN_GAP   4U     // pauza cea mai scurta dintre bip-uri, in unitati de 10 ms

// Cele doua contoare, scrise din main.c si citite in intrerupere (unitati de 10 ms):
//   buzzer_on_time  = durata unui bip;          0 = buzzer oprit
//   buzzer_off_time = pauza dintre doua bip-uri; 0 = sunet continuu
extern volatile unsigned char buzzer_on_time;
extern volatile unsigned char buzzer_off_time;
// Structura pe octave (începând cu RE6 = 1175 Hz)
typedef enum {
    RE6 = 0, RE_SHARP6, MI6, FA6, FA_SHARP6, SOL6, SOL_SHARP6, LA6, LA_SHARP6, SI6,
    DO7, DO_SHARP7, RE7, RE_SHARP7, MI7, FA7, FA_SHARP7, SOL7, SOL_SHARP7, LA7, LA_SHARP7, SI7,
    DO8, DO_SHARP8, RE8, NUM_NOTES
} NoteMuzicale_t;
unsigned char buzzer_get_note_value(NoteMuzicale_t index);

void buzzer_set_step_tone(unsigned char mult_50hz);

/**
 * @brief Setează volumul PWM în 11 trepte (0 = Mute, 10 = Max protejat ~3.125% Duty Cycle).
 * @details Folosește o înmulțire și shiftare pentru conversie rapidă fără împărțiri software. 
 * Include corecție de rotunjire pentru ca treapta 1 să emită mereu un impuls minim (fără 0% parazit).
 * @param level_0_to_10 Nivel volum (0 pentru liniște, 1-10 pentru progresie acustică).
 */
void buzzer_set_volume(unsigned char level_0_to_10);
/** 
 * @brief Redă o notă muzicală specificată de index pentru o durată în milisecunde.
 * @param index_nota Indexul notei muzicale de redat. Valid între 0 și 24, corespunzător notelor din enum-ul NoteMuzicale_t.
 * @param durata_ms Durata redării notei în milisecunde. Dacă este 0, funcția nu va face nimic.
*/
void buzzer_play_note(NoteMuzicale_t index_nota, unsigned int durata_ms);
void buzzer_apply_volume(void);
/**
 * @brief Seteaza nota si volumul buzzerului fara sa porneasca sau sa opreasca
 * sunetul. Se apeleaza doar din bucla principala (contine o inmultire pe 32 de
 * biti); poate fi apelata si in timp ce intreruperea genereaza bip-uri.
 */
void buzzer_setup(NoteMuzicale_t nota, unsigned char level_0_to_10);

#endif	/* BUZZER_H */