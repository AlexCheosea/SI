#include "hmi.h"
// --- Afișare Text ASCII ---
void hmi_print_string(const char *str) {
    dp_pos = -1;
    for (uint8_t i = 0; i < DISP_BUF_SIZE; i++) {
        if (str[i] != '\0') {
            disp_buf[i] = (uint8_t)str[i]; // Copiem codul ASCII în bufferul asamblării
        } else {
            // Dacă textul s-a terminat, golim restul display-ului cu spații
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

    // 1. Shave off the lower digits upfront
    // First, find out how many digits the number actually has
    uint16_t count_temp = temp_val;
    uint8_t total_digits = (number == 0) ? 1 : 0;
    while (count_temp > 0) {
        total_digits++;
        count_temp /= 10;
    }

    // If it has more digits than requested, chop the lower ones off the bottom
    // Example: 12345 (5 digits) with digits=4 -> divides by 10 once -> temp_val becomes 1234
    while (total_digits > digits) {
        temp_val /= 10;
        total_digits--;
    }

    // 2. Define the exact window inside the 0-7 buffer
    uint16_t window_start = start_pos;
    uint16_t window_end = start_pos + digits + is_negative - 1;

    if (window_end >= DISP_BUF_SIZE) {
        window_end = DISP_BUF_SIZE - 1; 
    }

    // 3. Clear the target display window with spaces first
    for (uint16_t target = window_start; target <= window_end; target++) {
        disp_buf[target] = ' ';
    }

    // 4. Clean, per-case right-to-left writing loops
    int16_t i = (int16_t)window_end;

    // Case A: Number is zero
    if (number == 0) {
        if (i >= (int16_t)window_start) {
            disp_buf[i--] = '0';
        }
    } 
    // Case B: Extract the remaining (higher) digits
    else {
        while (temp_val > 0 && i >= (int16_t)window_start) {
            disp_buf[i--] = (char)('0' + (temp_val % 10));
            temp_val /= 10;
        }
    }

    // Case C: Apply negative sign if room permits
    if (is_negative && i >= (int16_t)window_start) {
        disp_buf[i--] = '-';
    }
}

// --- Management Liniar Bargraph pe 10 biți (Număr exact de LED-uri) ---
void hmi_set_bargraph(uint8_t led_count, uint8_t is_red) {
    uint16_t mask = 0;
    
    // 1. Protecție: limităm numărul maxim de LED-uri la 10 (cât permite hardware-ul)
    if (led_count > 10) {
        led_count = 10;
    }
    
    // 2. Generăm masca (ex: led_count = 3 -> (1 << 3) - 1 = 0b00000111)
    mask = (uint16_t)((1 << led_count) - 1);
    
    // 3. Distribuim masca în registrele de 8 biți
    if (is_red) {
        bar_red_low  = (uint8_t)(mask & 0xFF);        // Primii 8 biți
        bar_red_high = (uint8_t)((mask >> 8) & 0x03); // Izolăm strict cei 2 LSBs
    } else {
        bar_grn_low  = (uint8_t)(mask & 0xFF);        // Primii 8 biți
        bar_grn_high = (uint8_t)((mask >> 8) & 0x03); // Izolăm strict cei 2 LSBs
    }
}
//extern volatile uint8_t btn_i0event;
//extern volatile uint8_t btn_i1event;
unsigned char citeste_tastatura(void){
    unsigned char ev0 = 0;
    unsigned char ev1 = 0;
    di();
    ev0 = btn_i0event;
    ev1 = btn_i1event;
    btn_i0event = 0;
    btn_i1event = 0;
    ei();
    
    //Mapare primele 8 butoane
    if(ev0 & 0x01) return '7'; 
    if(ev0 & 0x02) return '8';
    if(ev0 & 0x04) return '9';
    if(ev0 & 0x08) return '/';
    if(ev0 & 0x10) return '4';
    if(ev0 & 0x20) return '5';
    if(ev0 & 0x40) return '6';
    if(ev0 & 0x80) return '*';
    
    //Mapare ultimele 8 butoane
    if(ev1 & 0x01) return '1'; 
    if(ev1 & 0x02) return '2';
    if(ev1 & 0x04) return '3';
    if(ev1 & 0x08) return '-';
    if(ev1 & 0x10) return 'C';
    if(ev1 & 0x20) return '0';
    if(ev1 & 0x40) return '#';
    if(ev1 & 0x80) return '+';

    return 0;
}

