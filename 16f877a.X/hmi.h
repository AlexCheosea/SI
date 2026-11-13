/* * File:   hmi.h
 * Author: Alexandru Gabriel Cheosea
 * Comments: interfata C pentru modulul HMI (hmi_asm.s)
 */

#ifndef HMI_H
#define HMI_H
#include <xc.h>
#include <stdint.h>
#define RED 1
#define GRN 0
#define DISP_BUF_SIZE 8
// Variabile definite in hmi_asm.s; __bank(n) trebuie sa corespunda cu
// class=BANKn de acolo (necesita -maddrqual=require).
extern volatile __bank(3) uint8_t disp_buf[DISP_BUF_SIZE];
extern volatile __bank(3) uint8_t bar_red_low;
extern volatile __bank(3) uint8_t bar_red_high;
extern volatile __bank(3) uint8_t bar_grn_low;
extern volatile __bank(3) uint8_t bar_grn_high;
extern volatile __bank(3) uint8_t btn_i0event;
extern volatile __bank(3) int8_t  dp_pos;
// Rutine din asamblare
void hmi_init(void);
void hmi_update(void); // din ISR, la fiecare tick

// Afiseaza un sir ASCII (maxim DISP_BUF_SIZE caractere)
void hmi_print_string(const char *str);
// Afiseaza un numar cu semn, aliniat la dreapta in fereastra
// start_pos .. start_pos + digits - 1 (golita inainte). Daca nu incape, se
// pastreaza cifrele cele mai semnificative.
void hmi_print_number(int16_t number, uint8_t start_pos, uint8_t digits);
// Aprinde led_count LED-uri pe bara rosie (is_red = 1) sau verde (0)
void hmi_set_bargraph(uint8_t led_count, uint8_t is_red);

// Codul ASCII al butonului apasat, 0 daca niciunul
unsigned char citeste_tastatura(void);

// Punctul zecimal la pozitia pos (0 .. DISP_BUF_SIZE - 1), dupa afisarea caracterului
#define set_dp_pos(pos)         (dp_pos = (pos))
#endif /* HMI_H */
