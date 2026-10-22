#include "main.h"
#include "bme280.h"


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
    [ST_MENU_METEO]  = { .rows = sub_meteo_uart_txt,  .num_rows = NUM_SUB_METEO },
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
    int16_t bme_temp;
    uint16_t bme_press;
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
    for(uint8_t i = 0; i < BME280_CALIB_SIZE; i++){
        SPI_PACKET->cmd = CMD_SEND_BME280_CALIB;
        SPI_PACKET->raw[1] = i;
        SPI_PACKET->raw[2] = c.bytes[i];
        spi_transfer_packet();
        __delay_us(50);
    }
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
        // --------------------------------------------------------------------
        // ---------------- START AUTOMAT PRINCIPAL DE STĂRI ------------------
        // --------------------------------------------------------------------
        
        switch (menuState) {
            
            case ST_MENU_METEO:
            {
                BME280_rez_t rez;
                bme280_read_raw();
                for(uint8_t i = 0; i < RAW_SIZE; i++){
                    SPI_PACKET->cmd = CMD_SEND_BME280_DATA;
                    SPI_PACKET->raw[1] = i;
                    SPI_PACKET->raw[2] = raw.bytes[i];
                    spi_transfer_packet();
                    __delay_us(50);
                }
                do{
                    __delay_us(250); //5 times longer delay to give
                    SPI_PACKET->cmd = CMD_READ_BME280_RES;
                    spi_transfer_packet();             
                    //main loop time to run
                }while(SPI_PACKET->val == MATH_NOT_DONE);
                for(uint8_t i = 0; i < REZ_SIZE; i++){
                    if(SPI_PACKET->raw[1] < REZ_SIZE)
                        rez.bytes[SPI_PACKET->raw[1]] = SPI_PACKET->raw[2];
                    SPI_PACKET->cmd = CMD_READ_BME280_RES;
                    spi_transfer_packet();
                    __delay_us(50);
                }
                
                if(context == inMenu) {
                    hmi_print_string(menu_seg[menuState]);
                    //uart_print_string(menu_uart[menuState]);
                }
                else {
                    if(context == inSubMenu) { // inSubMenu pentru ST_MENU_METEO
                        if(tasta == '+' || delta > 0) {  // Meniu urmator
                            subMenuState.meteo++;
                            if(subMenuState.meteo >= NUM_SUB_METEO) // Rollover
                                subMenuState.meteo = 0;
                        }
                        else if(tasta == '-' || delta < 0) { // Meniu anterior
                            if(subMenuState.meteo == 0)
                                subMenuState.meteo = NUM_SUB_METEO - 1;   // Rollover
                            else
                                subMenuState.meteo--;
                        }
                        else if(tasta == '#') {
                            context = inApp; // Accesam "aplicatia" din submeniu
                        }
                        hmi_print_string(submenu_meteo[subMenuState.meteo]);
                        //uart_print_string(submenu_meteo_uart[subMenuState.meteo]);
                    }
                }
                
                switch(subMenuState.meteo) {    // inApp pentru ST_MENU_METEO
                    
                    case TEMP_BME:
                    {
                        if(context == inApp) {
                            bme_temp = rez.fields.temperature;
                            cifra = '0';
                            if(tasta == 'C') {
                                context = inSubMenu; // Revenim la submeniu
                                refresh = 1; // Forțăm reîmprospătarea afișajului
                                continue;
                            }
                            disp_buf[0] = 'T';
                            disp_buf[1] = ' ';
                            hmi_print_number(bme_temp, 2, 4);
                            set_dp_pos(3);
                            disp_buf[6] = ' ';
                            disp_buf[7] = 'C';
                            
                        }
                        break;
                    }
                    case PRES_BME:
                    {
                        if(context == inApp) {
                            // TODO: Logica pentru citirea presiunii de la BME
                            if(tasta == 'C') {
                                context = inSubMenu; // Revenim la submeniu
                                refresh = 1; // Forțăm reîmprospătarea afișajului
                                continue;
                            }
                            disp_buf[0] = 'P';

                            hmi_print_number((int16_t)rez.fields.pressure,1,5);
                            set_dp_pos(4);
                            disp_buf[6] = 'h';
                            disp_buf[7] = 'P';

                        }
                        break;
                    }
                    case HUM_BME:
                    {
                        if(context == inApp) {
                            
                            if(tasta == 'C') {
                                context = inSubMenu; // Revenim la submeniu
                                refresh = 1; // Forțăm reîmprospătarea afișajului
                                continue;
                            }
                            disp_buf[0] = 'H';
                            disp_buf[1] = 'U';
                            disp_buf[2] = 'M';
                            hmi_print_number((int16_t)rez.fields.humidity, 3, 4);
                            set_dp_pos(5);
                            disp_buf[7] = '%';
                           
                            
                        }
                        break;
                    default:
                        break;
                    }
                }
            }
                break;
            case ST_MENU_SPI:
            {
                if(context == inMenu) {
                    hmi_print_string(menu_seg[menuState]);
                    //uart_print_string(menu_uart[menuState]);
                }
                else {
                    if(context == inSubMenu) { // Daca suntem in interiorul meniului, afisam submeniul
                        if(tasta == '+' || delta > 0) {  // Meniu urmator
                            subMenuState.spi++;
                            if(subMenuState.spi >= NUM_SUB_SPI) // Rollover
                                subMenuState.spi = 0;
                        }
                        else if(tasta == '-' || delta < 0) { // Meniu anterior
                            if(subMenuState.spi == 0)
                                subMenuState.spi = NUM_SUB_SPI - 1;   // Rollover
                            else
                                subMenuState.spi--;
                        }
                        else if(tasta == '#') {
                            context = inApp; // Accesam "aplicatia" din submeniu
                        }
                        hmi_print_string(submenu_spi[subMenuState.spi]);
                        //uart_print_string(submenu_spi_uart[subMenuState.spi]);
                    }
                }
                
                switch(subMenuState.spi) { // Logică pentru aplicatiile SPI
                    case TEMP_SPI:  //TODO: De implementat logica pentru citire/scriere SPI
                    {
                        if(context == inApp) {
                            if(tasta == 'C') {
                                context = inSubMenu;
                                refresh = 1; // Forțăm reîmprospătarea afișajului
                                continue; // Revenim la submeniu 
                            }
                            cifra = '0';
                            SPI_PACKET->cmd = CMD_READ_TEMP;
                            spi_transfer_packet();
                            disp_buf[0] = 'T';
                            hmi_print_number(SPI_PACKET->val,1,6);
                            set_dp_pos(5);                            
                            disp_buf[7] = 'C';
                        }
                        break;
                    }
                    case POT_SPI:
                    {
                        if(context == inApp) {
                            if(tasta == 'C') {
                                context = inSubMenu;
                                refresh = 1;
                                continue; 
                            }
                            SPI_PACKET->cmd = CMD_READ_POT;
                            spi_transfer_packet();
                            cifra = '0';
                            disp_buf[0] = 'U';
                            hmi_print_number(SPI_PACKET->val, 1, 6);
                            set_dp_pos(3);
                            disp_buf[7] = 'V';
                        }
                        break;
                    }
                    case LIGHT_SPI:
                    {
                        if(context == inApp) {
                            if(tasta == 'C') {
                                context = inSubMenu;
                                refresh = 1;
                                continue; 
                            }
                            SPI_PACKET->cmd = CMD_READ_LIGHT;
                            spi_transfer_packet();
                            uint8_t light_val = (uint8_t)SPI_PACKET->val;
                            hmi_print_string("LGHT");
                            hmi_print_number(SPI_PACKET->val, 4, 3);
                            disp_buf[7] = '%';
                            

                        }
                        break;
                    }
                    case MOTOR_SPEED:
                    {
                        if(context == inApp) {
                            if(tasta == 'C') {
                                context = inSubMenu; 
                                refresh = 1;
                                hmi_set_bargraph(0, RED);
                                continue;
                            }
                            if(tasta == '+' || delta > 0) {  
                                if(motor_speed < 1000)
                                    motor_speed += 5;
                            }
                            else if(tasta == '-' || delta < 0) { 
                                if(motor_speed >= 5)
                                    motor_speed -= 5;
                            }
                            SPI_PACKET->cmd = CMD_SET_MOTOR_SPEED;
                            SPI_PACKET->val = (int16_t)motor_speed;
                            spi_transfer_packet();
                            hmi_set_bargraph((uint8_t)(motor_speed/100), RED);
                            disp_buf[0] = 'S';
                            hmi_print_number((int16_t)motor_speed, 1,4);
                            disp_buf[5] = 'P';
                            disp_buf[6] = '/';
                            disp_buf[7] = 'S';
                            
                        }
                        break;
                    }
                    case MOTOR_DIR:
                    {
                        if(context == inApp) {
                            if(tasta == 'C') {
                                context = inSubMenu;
                                refresh = 1; 
                                continue;
                            }
                            if(tasta == '+' || delta > 0) {  
                                motor_direction = DIR_CW;
                            }
                            else if(tasta == '-' || delta < 0) { 
                                motor_direction = DIR_CCW;
                            }
                            SPI_PACKET->cmd = CMD_SET_MOTOR_DIR;
                            SPI_PACKET->val = motor_direction;
                            spi_transfer_packet();
                            hmi_print_string("DIR");
                            disp_buf[5] = 'C';
                            if(motor_direction == DIR_CW) {
                                disp_buf[6] = 'W';
                            }
                            else {
                                disp_buf[6] = 'C';
                                disp_buf[7] = 'W';
                            }
                        }
                        break;
                    default:
                        break;
                    } //END CASE submenu
                }   //END SWITCH submenu
                break;
            }   //END CASE Menu
            case ST_MENU_SETARI:
            {
                if(context == inMenu) {
                    hmi_print_string(menu_seg[menuState]);
                    //uart_print_string(menu_uart[menuState]);
                }
                else if(context == inSubMenu) { // Daca suntem in interiorul meniului, afisam submeniul
                    if(tasta == '+' || delta > 0) {  // Meniu urmator
                        subMenuState.setari++;
                        if(subMenuState.setari >= NUM_SUB_SET) // Rollover
                            subMenuState.setari = 0;
                    }
                    else if(tasta == '-' || delta < 0) { // Meniu anterior
                        if(subMenuState.setari == 0)
                            subMenuState.setari = NUM_SUB_SET - 1;   // Rollover
                        else
                            subMenuState.setari--;
                    }
                    else if(tasta == '#') {
                        context = inApp; // Accesam "aplicatia" din submeniu
                    }
                    hmi_print_string(submenu_setari[subMenuState.setari]);
                    //uart_print_string(submenu_setari_uart[subMenuState.setari]);
                }
                
                switch(subMenuState.setari) {
                    case TONE:
                    {
                        if(context == inApp) {
                            if(tasta == 'C') {
                                context = inSubMenu;
                                refresh = 1; // Forțăm reîmprospătarea afișajului 
                                continue;
                            }
                            if(tasta == '+' || delta > 0) {
                                if(pitch < RE8)
                                    pitch++;
                                buzzer_play_note(pitch, 100);
                            }
                            else if(tasta == '-' || delta < 0) { 
                                if(pitch > RE6)
                                    pitch--;
                                buzzer_play_note(pitch, 100);
                            }
                            hmi_print_string("TONE");
                            hmi_print_number((int16_t)pitch,6,2);
                        }    
                        break;
                    }
                    case DURATION:
                    {
                        if(context == inApp) {
                            if(tasta == 'C') {
                                context = inSubMenu;
                                refresh = 1; // Forțăm reîmprospătarea afișajului
                                continue; 
                            }
                            if(tasta == '+' || delta > 0) {  
                                if(duration < 1000)
                                    duration += 50;
                                buzzer_play_note(pitch,duration);
                            }
                            else if(tasta == '-' || delta < 0) { 
                                if(duration > 50)
                                    duration -= 50;
                                buzzer_play_note(pitch,duration);
                            }
                            disp_buf[0] = 'D';
                            hmi_print_number((int16_t)duration,1,5);
                            disp_buf[6] = 'm';
                            disp_buf[7] = 's';
                        }
                        break;
                    }
                    case VOLUME:
                    {
                        if(context == inApp) {
                            if(tasta == 'C') {
                                context = inSubMenu;
                                refresh = 1;
                                continue;
                            }
                            if(tasta == '+' || delta > 0) {  
                                if(muzicuta_volum < 10)
                                    muzicuta_volum++;
                                buzzer_set_volume(muzicuta_volum);
                             
                            }
                            else if(tasta == '-' || delta < 0) { 
                                if(muzicuta_volum > 0)
                                    muzicuta_volum--;
                                buzzer_set_volume(muzicuta_volum);
                            
                            }
                            hmi_set_bargraph(muzicuta_volum, GRN);
                            hmi_print_string("VOLUM");
                            hmi_print_number(muzicuta_volum, 6,2);                            
                        }

                        break;
                    }
                    case KEY_BEEP:
                    {
                        if(context == inApp){
                            if(tasta == 'C') {
                                context = inSubMenu;
                                refresh = 1;
                                continue;
                            }
                            if(tasta == '+' || delta > 0)
                                sound = 1;
                            else if(tasta == '-' || delta < 0)
                                sound = 0;
                            hmi_print_string("BEEP O");
                            if(sound){
                                disp_buf[6] = 'N';
                            }
                            else{
                                disp_buf[6] = 'F';
                                disp_buf[7] = 'F';
                            }
         
                        }
                        break;
                    }
                    case SAVE:
                    {
                        if(context == inApp){
                            if(tasta == 'C'){
                                context = inSubMenu;
                                refresh = 1;
                                continue;
                            }
                            if(tasta == '#'){
                                hmi_print_string("SAVED");
                                context = inSubMenu;
                                // 1. Pack the boolean flags into a single efficient 8-bit variable
                                uint8_t packed_flags = 0;
                                if (sound)           packed_flags |= FLAG_SOUND;
                                if (octava)          packed_flags |= FLAG_OCTAVA;
                                if (motor_direction) packed_flags |= FLAG_DIR;

                                // 2. Commit all single-byte variables
                                eeprom_write(EE_ADDR_VOLUME,     muzicuta_volum);
                                eeprom_write(EE_ADDR_PITCH,      (uint8_t)pitch);
                                eeprom_write(EE_ADDR_FLAGS,      packed_flags);

                                // 3. Split and commit 16-bit variables (High byte first, then Low byte)
                                eeprom_write(EE_ADDR_SPEED_HIGH, (uint8_t)(motor_speed >> 8));
                                eeprom_write(EE_ADDR_SPEED_LOW,  (uint8_t)(motor_speed & 0xFF));

                                eeprom_write(EE_ADDR_DUR_HIGH,   (uint8_t)(duration >> 8));
                                eeprom_write(EE_ADDR_DUR_LOW,    (uint8_t)(duration & 0xFF));
                                
                                        
                            }
                        }
                        break;
                    }
                    default:
                        break;
                }
                break;
            }

            case ST_MENU_MUZICUTA:
            {
                if(context == inMenu) {
                    hmi_print_string(menu_seg[menuState]);
                    //uart_print_string(menu_uart[menuState]);
                }
                else {
                    if(tasta == 'C') {
                        context = inMenu;
                        refresh = 1; // Forțăm reîmprospătarea afișajului
                        continue;
                    }
                    if(tasta == '+') 
                        octava = 1;
                    if(tasta == '-') 
                        octava = 0;
                    disp_buf[5] = 'T';
                    disp_buf[6] = 'O';
                    disp_buf[7] = 'N';
                    
                    switch(tasta) {

                        case '7':
                            buzzer_play_note(RE6 + octava*12, duration);
                            disp_buf[5] = 'D';
                            disp_buf[6] = '6';
                            disp_buf[7] = ' ';
                            break;

                        case '8':
                            buzzer_play_note(RE_SHARP6 + octava*12, duration);
                            disp_buf[5] = 'D';
                            disp_buf[6] = '6';
                            disp_buf[7] = '#';
                            break;

                        case '9':
                            buzzer_play_note(MI6 + octava*12, duration);
                            disp_buf[5] = 'E';
                            disp_buf[6] = '6';
                            disp_buf[7] = ' ';
                            break;

                        case '/':
                            buzzer_play_note(FA6 + octava*12, duration);
                            disp_buf[5] = 'F';
                            disp_buf[6] = '6';
                            disp_buf[7] = ' ';
                            break;

                        case '4':
                            buzzer_play_note(FA_SHARP6 + octava*12, duration);
                            disp_buf[5] = 'F';
                            disp_buf[6] = '6';
                            disp_buf[7] = '#';
                            break;

                        case '5':
                            buzzer_play_note(SOL6 + octava*12, duration);
                            disp_buf[5] = 'G';
                            disp_buf[6] = '6';
                            disp_buf[7] = ' ';
                            break;

                        case '6':
                            buzzer_play_note(SOL_SHARP6 + octava*12, duration);
                            disp_buf[5] = 'G';
                            disp_buf[6] = '6';
                            disp_buf[7] = '#';
                            break;

                        case '*':
                            buzzer_play_note(LA6 + octava*12, duration);
                            disp_buf[5] = 'A';
                            disp_buf[6] = '6';
                            disp_buf[7] = ' ';
                            break;

                        case '1':
                            buzzer_play_note(LA_SHARP6 + octava*12, duration);
                            disp_buf[5] = 'A';
                            disp_buf[6] = '6';
                            disp_buf[7] = '#';
                            break;

                        case '2':
                            buzzer_play_note(SI6 + octava*12, duration);
                            disp_buf[5] = 'B';
                            disp_buf[6] = '6';
                            disp_buf[7] = ' ';
                            break;

                        case '3':
                            buzzer_play_note(DO7 + octava*12, duration);
                            disp_buf[5] = 'C';
                            disp_buf[6] = '7';
                            disp_buf[7] = ' ';
                            break;

                        case '0':
                            buzzer_play_note(DO_SHARP7 + octava*12, duration);
                            disp_buf[5] = 'C';
                            disp_buf[6] = '7';
                            disp_buf[7] = '#';
                            break;

                        case '#':
                            buzzer_play_note(RE7 + octava*12, duration);
                            disp_buf[5] = 'D';
                            disp_buf[6] = '7';
                            disp_buf[7] = ' ';
                            break;

                        default:
                            break;
                    }
                    if(octava && (tasta != '+' || delta != 0) )
                        disp_buf[6]++;
                    if(delta > 0) {
                        if(muzicuta_volum < 10)
                            muzicuta_volum++;
                        buzzer_set_volume(muzicuta_volum);
                    }    
                    if(delta < 0) {
                        if(muzicuta_volum > 0)
                            muzicuta_volum--;
                        buzzer_set_volume(muzicuta_volum);
                    }
                    hmi_set_bargraph(muzicuta_volum, GRN);
                    disp_buf[0] = 'V';
                    hmi_print_number(muzicuta_volum, 1, 2);
                    disp_buf[3] = ' ';
                    disp_buf[4] = ' ';
                            
                    
                    
                      
                    //uart_print_string("|muzicuta    >|");
                }
                break;
            }

            case ST_MENU_UART:
            {
                if (context == inMenu) {
                    hmi_print_string(menu_seg[menuState]);
                    //uart_print_string(menu_uart[menuState]);
                }
                else {
                    char msg[UART_BUF_SIZE];
                    if (uart_read_string(msg) > 0) {
                        uint8_t len = 0;
                        while (msg[len] != '\0' && len < UART_BUF_SIZE - 1)
                            len++;

                        char view[9];
                        view[8] = '\0';

                        for (uint8_t pos = 0; pos < len; pos++) {
                            for (uint8_t i = 0; i < 8; i++)
                                view[i] = msg[(pos + i) % len];
                            hmi_print_string(view);
                            __delay_ms(150);
                        }
                    }
                    else {
                        context = inMenu;
                        hmi_print_string(menu_seg[menuState]);
                    }
                }
                break;
            }

            default:
            {
                menuState = ST_MENU_METEO;
                hmi_print_string(error_seg[ERR_MENU]);
                //uart_print_string(error_uart[ERR_MENU]);
                __delay_ms(1000);
                subMenuState.raw = 0;
                break;
            }
        } // <--switch (menuState)
    } // <-- while(1)
} // <-- void main(void)
