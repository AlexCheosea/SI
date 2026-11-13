#ifndef BUZZER_H
#define	BUZZER_H

#define MIN_MULT_50HZ 23
// Implementate in buzzer_asm.s
void buzzer_init(void);
extern void buzzer_on(void);
extern void buzzer_off(void);

volatile unsigned int buzzer_ms_ramase = 0;

// Buzzer de parcare: bip-urile sunt generate in intreruperea Timer0
#define BUZZER_TONE      DO7    // nota bip-ului
#define BUZZER_VOLUME    5      // volumul, 0-10
#define BUZZER_BEEP_TIME 6U     // durata unui bip, x10 ms
#define BUZZER_MIN_GAP   4U     // pauza minima dintre bip-uri, x10 ms

// Scrise din main.c, citite in intrerupere (x10 ms):
//   buzzer_on_time  = durata bip-ului;  0 = oprit
//   buzzer_off_time = pauza dintre bip-uri; 0 = sunet continuu
extern volatile unsigned char buzzer_on_time;
extern volatile unsigned char buzzer_off_time;
// Notele, in ordinea tabelului PR2 din buzzer_asm.s
typedef enum {
    RE6 = 0, RE_SHARP6, MI6, FA6, FA_SHARP6, SOL6, SOL_SHARP6, LA6, LA_SHARP6, SI6,
    DO7, DO_SHARP7, RE7, RE_SHARP7, MI7, FA7, FA_SHARP7, SOL7, SOL_SHARP7, LA7, LA_SHARP7, SI7,
    DO8, DO_SHARP8, RE8, NUM_NOTES
} NoteMuzicale_t;
unsigned char buzzer_get_note_value(NoteMuzicale_t index);

void buzzer_set_step_tone(unsigned char mult_50hz);

// Volumul: 0 = mut, 1-10 = factor de umplere crescator (treapta 1 nu este niciodata 0%)
void buzzer_set_volume(unsigned char level_0_to_10);
// Reda o nota timp de durata_ms (0 = nimic)
void buzzer_play_note(NoteMuzicale_t index_nota, unsigned int durata_ms);
void buzzer_apply_volume(void);
// Seteaza nota si volumul fara sa porneasca / opreasca sunetul.
// Doar din main (inmultire pe 32 de biti); merge si in timpul bip-urilor.
void buzzer_setup(NoteMuzicale_t nota, unsigned char level_0_to_10);

#endif	/* BUZZER_H */
