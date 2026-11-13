#include "hmi.h"
// Date folosite de hmi_asm.s, toate in Bancul 3 (asm: BANKSEL pe una, BANKMASK pe celelalte)
volatile __bank(3) uint8_t seg_index;
volatile __bank(3) uint8_t disp_cnt;     // faza multiplexarii: 0-7 cifre, 8 bara verde, 9 bara rosie
volatile __bank(3) uint8_t bar_red_low;
volatile __bank(3) uint8_t bar_red_high;
volatile __bank(3) uint8_t bar_grn_low;
volatile __bank(3) uint8_t bar_grn_high;
volatile __bank(3) int8_t  dp_pos;       // pozitia punctului zecimal (-1 = niciunul)
volatile __bank(3) uint8_t disp_buf[DISP_BUF_SIZE];   // caracterele ASCII afisate
volatile __bank(3) uint8_t btn_i0state;
volatile __bank(3) uint8_t btn_i0prev;
volatile __bank(3) uint8_t btn_i0event;
void hmi_print_string(const char *str) {
    dp_pos = -1;
    for (uint8_t i = 0; i < DISP_BUF_SIZE; i++) {
        if (str[i] != '\0') {
            disp_buf[i] = (uint8_t)str[i];
        } else {
            // Restul afisajului se goleste cu spatii
            while (i < 8) {
                disp_buf[i++] = ' ';
            }
            break;
        }
    }
}

void hmi_print_number(int16_t number, uint8_t start_pos, uint8_t digits) {
    if (start_pos >= DISP_BUF_SIZE || digits == 0) {
        return;
    }

    uint16_t temp_val;
    uint8_t is_negative = 0;

    if (number < 0) {
        is_negative = 1;
        temp_val = (uint16_t)(-(int32_t)number);
    } else {
        temp_val = (uint16_t)number;
    }

    // Numarul de cifre
    uint16_t count_temp = temp_val;
    uint8_t total_digits = (number == 0) ? 1 : 0;
    while (count_temp > 0) {
        total_digits++;
        count_temp /= 10;
    }

    // Prea multe cifre: se renunta la cele de jos
    while (total_digits > digits) {
        temp_val /= 10;
        total_digits--;
    }

    // Fereastra din afisaj
    uint16_t window_start = start_pos;
    uint16_t window_end = start_pos + digits + is_negative - 1;

    if (window_end >= DISP_BUF_SIZE) {
        window_end = DISP_BUF_SIZE - 1;
    }

    for (uint16_t target = window_start; target <= window_end; target++) {
        disp_buf[target] = ' ';
    }

    // Scriere de la dreapta la stanga
    int16_t i = (int16_t)window_end;

    if (number == 0) {
        if (i >= (int16_t)window_start) {
            disp_buf[i--] = '0';
        }
    }
    else {
        while (temp_val > 0 && i >= (int16_t)window_start) {
            disp_buf[i--] = (char)('0' + (temp_val % 10));
            temp_val /= 10;
        }
    }

    // Semnul, daca mai este loc
    if (is_negative && i >= (int16_t)window_start) {
        disp_buf[i--] = '-';
    }
}

void hmi_set_bargraph(uint8_t led_count, uint8_t is_red) {
    uint16_t mask = 0;

    if (led_count > 10) { // LED-urile disponibile pe bara
        led_count = 10;
    }

    mask = (uint16_t)((1 << led_count) - 1); // led_count biti de 1, de jos

    // Masca de 10 biti: 8 in octetul low, 2 in cel high
    if (is_red) {
        bar_red_low  = (uint8_t)(mask & 0xFF);
        bar_red_high = (uint8_t)((mask >> 8) & 0x03);
    } else {
        bar_grn_low  = (uint8_t)(mask & 0xFF);
        bar_grn_high = (uint8_t)((mask >> 8) & 0x03);
    }
}
unsigned char citeste_tastatura(void){
    unsigned char ev0 = 0;
    di();
    ev0 = btn_i0event;
    btn_i0event = 0;
    ei();

    if(ev0 & 0x01) return '0';
    if(ev0 & 0x02) return '1';
    if(ev0 & 0x04) return '2';
    if(ev0 & 0x08) return '3';
    if(ev0 & 0x10) return '4';
    if(ev0 & 0x20) return '5';
    if(ev0 & 0x40) return '6';
    if(ev0 & 0x80) return '7';

    return 0;
}
