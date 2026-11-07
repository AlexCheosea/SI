#include "main.h"


#define BARGRAPH_EFFECT_NUM_FRAMES 19
static const uint16_t bargraph_effect_frames[BARGRAPH_EFFECT_NUM_FRAMES] = {
    0b0000000001, // 0 - start, coada nu exista inca (margine)
    0b0000000011, // 1
    0b0000000111, // 2
    0b0000001110, // 3
    0b0000011100, // 4
    0b0000111000, // 5
    0b0001110000, // 6
    0b0011100000, // 7
    0b0111000000, // 8
    0b1110000000, // 9 - capat dreapta, coada completa
    0b1100000000, // 10 - intoarcere, coada se ingusteaza (margine)
    0b1110000000, // 11
    0b0111000000, // 12
    0b0011100000, // 13
    0b0001110000, // 14
    0b0000111000, // 15
    0b0000011100, // 16
    0b0000001110, // 17
    0b0000000111, // 18 - revine spre start, ciclul se reia
};

void bargraph_play_knight_rider(uint8_t call_divider) {
    static uint8_t call_counter = 0;
    static uint8_t frame_index = 0;
 
    if (call_divider == 0) {
        call_divider = 1; 
    }
 
    call_counter++;
    if (call_counter < call_divider) {
        return; // inca nu e momentul pentru urmatorul frame
    }
    call_counter = 0;
 
    uint16_t mask = bargraph_effect_frames[frame_index];
 
    bar_red_low  = (uint8_t)(mask & 0xFF);        // Primii 8 biți
    bar_red_high = (uint8_t)((mask >> 8) & 0x03);  // Izolăm strict cei 2 LSBs
 
    frame_index++;
    if (frame_index >= BARGRAPH_EFFECT_NUM_FRAMES) {
        frame_index = 0; // reia ciclul de la inceput
    }
}

// 1. Main menu descriptor
const MenuDescriptor_t master_menu_desc = {
    .rows = menu_uart_txt,
    .num_rows = NUM_STATES
};
MeniuContext_t context = inMenu;
// 2. Mapping individual submenu data to state machine indices
const MenuDescriptor_t submenu_descriptors[NUM_STATES] = {
    [ST_MENU_SPI]    = { .rows = sub_spi_uart_txt,    .num_rows = NUM_SUB_SPI },
    [ST_MENU_SETARI] = { .rows = sub_setari_uart_txt, .num_rows = NUM_SUB_SET }
    // Unassigned states default to zero/null entries automatically
};
/**
 * @brief Takes a raw string, injects its list index number, frames it with '|',
 * pads it to exactly 15 characters, and transmits it.
 */
void uart_print_dressed_row(const char* raw_text, uint8_t row_index) {
    char temp_row[UART_BUF_SIZE]; // 16-byte workspace in data RAM
    uint8_t len;

    // 1. Establish the leading wall element
    temp_row[0] = '\xBA';
    
    // 2. Inject the dynamic row number (0 becomes '1', 1 becomes '2', etc.)
    temp_row[1] = '1' + row_index; 
    temp_row[2] = '.';
    temp_row[3] = '\0'; // Temporarily terminate to use strcat safely

    // 3. Append the menu option string directly from flash memory
    strcat(temp_row, raw_text);

    // 4. Evaluate total character length reached so far
    len = (uint8_t)(strlen(temp_row));

    // 5. Pad remaining slots out to column index 14 with blank space
    while (len < 14) {
        temp_row[len] = ' ';
        len++;
    }

    // 6. Apply the closing wall decoration and cap with a null terminator
    temp_row[14] = '\xBA';
    temp_row[15] = '\0';

    // 7. Pass to optimized assembly transmitter
    uart_print_string(temp_row);
}

/**
 * @brief Iterates dynamically through any variable menu descriptor object,
 * auto-numbers the rows, and pushes a complete bordered window structure.
 */
void uart_print_full_menu(MenuDescriptor_t menu) {
    // If a state has no defined menu contents, abort safely
    if (menu.rows == NULL || menu.num_rows == 0) {
        return;
    }
    
    // Clear the terminal window (ANSI escape codes)
    uart_print_string("\x1B[2J\x1B[H");
    
    // Print the upper decorative border cap
    uart_print_string("\xC9\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xBB");
    if(context != inMenu)
        uart_print_string("\xBA" "0.Back       \xBA");

    // Loop through items, passing the loop index 'i' to handle auto-numbering
    for (uint8_t i = 0; i < menu.num_rows; i++) {
        uart_print_dressed_row(menu.rows[i], i);
    }

    // Print the lower decorative border cap
    uart_print_string("\xC8\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xBC");
}

void delay_boot(void) {
    __delay_ms(120);
}
void boot_animation(void) {
    hmi_set_bargraph(0, RED);
    hmi_set_bargraph(0, GRN);
    uart_print_string("|Initializare |");
    hmi_print_string("        ");
    for(uint8_t i = 0; i < 17; i++)
        delay_boot();
    // Note 1: Upbeat
    buzzer_play_note(MI6, 120);
    delay_boot(); hmi_set_bargraph(0,  RED);
    
    // Note 2: Moving up
    buzzer_play_note(SOL6, 120);
    delay_boot(); hmi_set_bargraph(1,  RED);
    
    // Note 3: Building up
    buzzer_play_note(SI6, 120);
    delay_boot(); hmi_set_bargraph(2,  RED); disp_buf[0] = 'I';
    
    // Note 4: Peak
    buzzer_play_note(DO7, 120);
    delay_boot(); hmi_set_bargraph(3,  RED); disp_buf[1] = 'n';
    
    // Note 5: Higher
    buzzer_play_note(RE7, 120);
    delay_boot(); hmi_set_bargraph(4,  RED); disp_buf[2] = 'i';
    
    // Note 6: 
    buzzer_play_note(MI7, 240); 
    delay_boot(); hmi_set_bargraph(5,  RED); disp_buf[3] = 't';
    
    // (Skipped slot - MI7 is still playing through this delay)
    delay_boot(); hmi_set_bargraph(6,  RED); disp_buf[4] = '.';
    
    // Note 7: Coming back down
    buzzer_play_note(RE7, 120);
    delay_boot(); hmi_set_bargraph(7,  RED); disp_buf[5] = '.';
    
    // Note 8: Descending
    buzzer_play_note(DO7, 120);
    delay_boot(); hmi_set_bargraph(8,  RED); disp_buf[6] = '.';
    
    // Note 9: Almost resolved
    buzzer_play_note(SI6, 120);
    delay_boot(); hmi_set_bargraph(9,  RED); disp_buf[7] = '.';
    
    // Note 10: Final satisfying resolution note (Holds long as the bar finishes)
    buzzer_play_note(SOL6, 300);
    delay_boot(); hmi_set_bargraph(10, RED);

    hmi_print_string("        ");
    for(uint8_t i = 0; i < 4; i++) {
        if(i > 0) {
            disp_buf[3 - (i - 1)] = ' ';
            disp_buf[4 + (i - 1)] = ' ';
        }
        disp_buf[3 - i] = '<';
        disp_buf[4 + i] = '>';
        delay_boot();
    }
}


void main(void) {
    system_init();
    uint16_t data = 0;
    // Variabile pentru starea curentă a automatului
    MainState_t menuState = 0;
    SubMenu_t   subMenuState;
    NoteMuzicale_t pitch = MI6;
    uint16_t motor_speed = 10; // Viteza implicită a motorului în pași pe secundă
    static __bit motor_direction; // Direcția implicită a motorului
    static __bit sound = 0;
    motor_direction = DIR_CW;
    int8_t enc_pos = 0;
    uint8_t loop_cnt = 0;
    static __bit octava = 0; // 0 sau 1 pentru muzicuță
    static __bit refresh; // Flag pentru reîmprospătarea afișajului
    refresh = 1;
    unsigned char tasta = 0;
    char cifra;
    unsigned char muzicuta_volum = 3; // Volum implicit pentru muzicuță (0-10)
    unsigned int duration = 150;      // Durata implicită a notelor în ms
    if(eeprom_read(EE_ADDR_VOLUME) != 0xFF)
    {
        motor_speed = (uint16_t)(eeprom_read(EE_ADDR_SPEED_HIGH)<<8) | eeprom_read(EE_ADDR_SPEED_LOW);
        muzicuta_volum = eeprom_read(EE_ADDR_VOLUME);
        pitch = eeprom_read(EE_ADDR_PITCH);
        duration = (uint16_t)((eeprom_read(EE_ADDR_DUR_HIGH) << 8) | eeprom_read(EE_ADDR_DUR_LOW));
        uint8_t packed_flags = eeprom_read(EE_ADDR_FLAGS);
        sound           = (packed_flags & FLAG_SOUND)   ? 1 : 0;
        octava          = (packed_flags & FLAG_OCTAVA)  ? 1 : 0;
        motor_direction = (packed_flags & FLAG_DIR)     ? 1 : 0;
    }
    //boot_animation();
    hmi_set_bargraph(muzicuta_volum, GRN); // Inițializare bargraph la nivelul volumului muzicuței
    hmi_set_bargraph(0, RED);
    buzzer_set_volume(muzicuta_volum);
    context = inMenu;
    subMenuState.raw = 0;
    dp_pos = 0;
 
    while(1) {
        __delay_ms(5);
        if(!(context == inApp && menuState == ST_MENU_SPI && subMenuState.spi == MOTOR_SPEED))
            bargraph_play_knight_rider(12);
        //=========================================================
        //==================    CITIRE UART   =====================
        //=========================================================
        // 1. Direct, zero-allocation peek at the verified ASM buffer
        tasta = 0;
        if (UART_RX_STATUS == UART_RX_READY) {
            if(UART_RX_BUF[0] == '\n'){
                uart_release();
                continue;
            }
            if(UART_RX_BUF[1] == '\n'){
                if(menuState == ST_MENU_MUZICUTA && context != inMenu && UART_RX_BUF[0] >= 'A'){
                    if(UART_RX_BUF[0] < 'Z')
                    buzzer_play_note(UART_RX_BUF[0]-'A', duration);
                    if(UART_RX_BUF[0] == ']')
                        duration += 50;
                    if(UART_RX_BUF[0] == '[')
                        duration -=50;
                    uart_release();
                    continue;
                }
                if (UART_RX_BUF[0] >= '0' && UART_RX_BUF[0] <= '5') {
                    tasta = UART_RX_BUF[0];
                }
                if (UART_RX_BUF[0] == '+' || UART_RX_BUF[0] == '-' || UART_RX_BUF[0] == '#'){
                    tasta = UART_RX_BUF[0];
                    uart_release();
                    goto post_read;
                }
            }
            else{
                if(UART_RX_LEN <= DISP_BUF_SIZE+1)
                    hmi_print_string((const char*)UART_RX_BUF);
                //TODO: Display buffer with scrolling and uart_release
            }
            // Explicitly release the buffer state machine
            uart_release();            
        }
        
        // 2. Evaluate Direct Jumps (Using tasta)
        if (tasta == '0') {
            tasta = 'C';
            goto post_read;
        }
        if (tasta >= '1' && tasta <= '5') {
            uint8_t requested_index = tasta - '1';

            if (context == inMenu) {
                if (requested_index < master_menu_desc.num_rows) {
                    menuState = (MainState_t)requested_index;
                    context = inSubMenu;
                    goto state_machine; 
                }
            } 
            else if (context == inSubMenu) {
                if (requested_index < submenu_descriptors[menuState].num_rows) {
                    subMenuState.raw = requested_index;
                    context = inApp;
                    goto state_machine; 
                }
            }
        }
        
        tasta = citeste_tastatura();
    post_read:
        di();
        int8_t delta = enc_pos - enc_pos;
        ei();
        
        if(tasta == 0 && delta == 0 && refresh == 0){ // Daca nu exista evenimente
            if(context == inApp && loop_cnt == 120){ //valoarea update display la ~x*5 ms
                loop_cnt = 0;   //modifica rata de update a display ului
            }else{              //cane meniu == inApp
                loop_cnt++;
                continue;  // reiau bucla
            }
        }
                          
        enc_pos = enc_pos;
        if (sound && tasta && !(menuState == ST_MENU_MUZICUTA && context != inMenu)) {
            buzzer_play_note(pitch, duration); //Feedback audio pentru meniurile diferite de muzicuta
        }
        refresh = 0; // Resetam flag-ul de refresh
        

        if(context == inMenu) { // Meniu principal
            if(tasta == '+' || delta > 0) {  // Meniu urmator
                menuState++;
                delta = 0; // Consumam evenimentele
                tasta = 0;
                if(menuState >= NUM_STATES) // Rollover
                    menuState = 0;
            }
            else if(tasta == '-' || delta < 0) { // Meniu anterior
                delta = 0;
                tasta = 0; // Consumam evenimentele
                if(menuState == 0)
                    menuState = NUM_STATES - 1;   // Rollover
                else
                    menuState--;
            }
            else if(tasta == '#') {
                tasta = 0; // Consumam evenimentul
                context = inSubMenu;
            }
        }
        else { // In interiorul unui submeniu/aplicatie
            if(tasta == 'C' && context == inSubMenu) { // Revenire la meniul principal
                context = inMenu;
                subMenuState.raw = 0;
            }
        }
        state_machine:
        // Printare meniuri UART
        if (context == inMenu) {
            // Print the entire top-level main menu frame
            uart_print_full_menu(master_menu_desc);
        } 
        else if (context == inSubMenu) {
            // Print the entire active submenu box dynamically based on menuState
            uart_print_full_menu(submenu_descriptors[menuState]);
        }
 
    } // <-- while(1)
} // <-- void main(void)
