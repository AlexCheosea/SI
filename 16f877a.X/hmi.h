/* * File:   hmi.h
 * Author: Alexandru Gabriel Cheosea
 * Comments: Interfață de abstractizare (Wrapper) pentru modulul HMI
 */

#ifndef HMI_H
#define HMI_H
#include <xc.h>
#include <stdint.h>
#define RED 1
#define GRN 0
#define DISP_BUF_SIZE 8
// Variable brute exportate direct din asamblare
extern volatile uint8_t disp_buf[DISP_BUF_SIZE];
extern volatile uint8_t bar_red_low;
extern volatile uint8_t bar_red_high;
extern volatile uint8_t bar_grn_low;
extern volatile uint8_t bar_grn_high;
extern volatile uint8_t btn_i0event;
extern volatile int8_t  dp_pos;
// Rutinele native din asamblare
void hmi_init(void);
void hmi_update(void); // Trebuie apelată în ISR la fiecare 1ms
// ===================================================================
// API HIGHER-LEVEL (WRAPPER C)
// ===================================================================
/**
 * Trimite un șir de caractere ASCII către bufferul de afișare pe segmente.
 * @param str Șir de caractere (maxim 8 caractere vizibile).
 */
void hmi_print_string(const char *str);
/**
 * Afișează un număr întreg cu semn, aliniat automat la dreapta pe ecran.
 * @param number Valoarea numerică (-32768 până la 32767).
 */
void hmi_print_number(int16_t number, uint8_t start_pos, uint8_t digits);
/**
 * Controlează bargraph-urile.
 * @param led_count Numărul de LED-uri de afișat.
 * @param is_red Setează 1 pentru bara Roșie, 0 pentru cea Verde.
 */
void hmi_set_bargraph(uint8_t led_count, uint8_t is_red);

/**
 * Citeste variabilele pentru evenimente de la tastara.
 * @return Codul ASCII pentru butonul apasat.
 */
unsigned char citeste_tastatura(void);

//Macros:
// 0-7 pentru pozitia digit point
// trebuie scris dupa afisarea unui caracter la acea pozitie
#define set_dp_pos(pos)         (dp_pos = (pos))
#endif /* HMI_H */