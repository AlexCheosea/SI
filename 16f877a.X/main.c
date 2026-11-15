#include "main.h"

// ===========================================================================
// Constante
// ===========================================================================
// Motor cu 200 pasi/rotatie: RPM = pasi/s x 60 / 200 = pasi/s x 3 / 10
// Plafon redus pentru precizie (mai putini pasi pierduti); acelasi in sclav (MAX_SPS)
#define MAX_CRUISE_RPM    800U
#define MAX_CRUISE_SPEED  ((MAX_CRUISE_RPM * 10U + 2U) / 3U)  // pasi/s (2667)

// Valorile comenzilor UART se citesc pe 32 de biti, limitate la INT32_MAX
// (mov: tinta / pas in pasi); celelalte comenzi le limiteaza la CMD_SMALL_MAX
#define CMD_VALUE_MAX   2147483647UL   // INT32_MAX
#define CMD_VALUE_TENTH 214748364UL    // partea intreaga din CMD_VALUE_MAX / 10
#define CMD_SMALL_MAX 9999U

// Pasii butoanelor: in RPM, convertiti in pasi/s
#define BUTTON_SMALL_STEP_RPM 10U
#define BUTTON_LARGE_STEP_RPM 100U
#define BUTTON_SMALL_STEP ((BUTTON_SMALL_STEP_RPM * 10U + 2U) / 3U)
#define BUTTON_LARGE_STEP ((BUTTON_LARGE_STEP_RPM * 10U + 2U) / 3U)

#define IS_DIGIT(c) ((c) >= '0' && (c) <= '9')

// Rampa (in sclav): pasi/s pe tick de 10 ms; 1 unitate = 30 RPM/s.
// Implicitele = cele din 16f690.X/motor.h.
#define RAMP_MIN           5U
#define RAMP_MAX           250U
#define RAMP_ACCEL_DEFAULT 15U
#define RAMP_DECEL_DEFAULT 40U

static uint16_t cruise_speed = 0;        // viteza ceruta de utilizator (pasi/s)
static uint16_t effective_speed = 0;     // viteza trimisa sclavului
static uint16_t actual_speed = 0;        // viteza raportata de sclav
static __bit motor_direction;
static __bit motor_undervoltage;
static __bit motor_overvoltage;
static __bit path_stop;                  // obstacol la STOP sau mai aproape
static __bit path_slow;                  // obstacol mai aproape de FAR
static __bit cruise_speed_changed;       // setat la orice modificare a cruise_speed
static uint8_t buzzer_volume = BUZZER_VOLUME;   // 0-10
static uint8_t buzzer_tone = BUZZER_TONE;       // index in NoteMuzicale_t (0 .. NUM_NOTES-1)
static __bit buzzer_settings_changed;    // volum / nota de aplicat in bucla principala
static uint8_t motor_accel = RAMP_ACCEL_DEFAULT;   // RAMP_MIN .. RAMP_MAX
static uint8_t motor_decel = RAMP_DECEL_DEFAULT;   // RAMP_MIN .. RAMP_MAX
static __bit ramp_changed;               // rampa de trimis sclavului
static __bit spi_err;                    // ultimul raspuns SPI nu a inceput cu SPI_ACK
// Moduri: continuu (motorul merge in run_direction cu effective_speed) sau
// pozitie (sclavul merge la target_position cu viteza maxima effective_speed
// si se opreste). mov -> pozitie; run comuta: din pozitie -> continuu, din
// continuu -> franare pana la 0 (hold_pending), apoi pozitie cu tinta = locul opririi.
static __bit position_mode;
static __bit hold_pending;               // iesire din modul continuu: se asteapta oprirea
static __bit run_direction;              // sensul modului continuu (rev / butonul de sens)
static __bit target_pending;             // tinta de trimis sclavului (2 pachete)
static int32_t target_position = 0;      // pasi
static int32_t reported_position = 0;    // pozitia citita de la sclav (pasi)

// ===========================================================================
// Setari in EEPROM (volum, nota, rampa): salvate la schimbare, aplicate la
// pornire. Valorile in afara limitelor raman implicite.
// ===========================================================================
#define EE_ADDR_SETTINGS 0x00U
enum { EE_VOLUME, EE_TONE, EE_ACCEL, EE_DECEL, EE_COUNT };
static uint8_t *const settings[EE_COUNT] = {
    &buzzer_volume, &buzzer_tone, &motor_accel, &motor_decel
};
static uint8_t ee_saved[EE_COUNT];      // copia EEPROM-ului

static void settings_load(void) {
    static const uint8_t limit_lo[EE_COUNT] = {0, 0, RAMP_MIN, RAMP_MIN};
    static const uint8_t limit_hi[EE_COUNT] = {
        10, NUM_NOTES - 1U, RAMP_MAX, RAMP_MAX   // volum 0-10, nota = index
    };
    uint8_t i;
    uint8_t value;

    for (i = 0; i < EE_COUNT; i++) {
        value = eeprom_read((uint8_t)(EE_ADDR_SETTINGS + i));
        ee_saved[i] = value;
        if (value >= limit_lo[i] && value <= limit_hi[i]) {
            *settings[i] = value;
        }
    }
}

// Un octet pe trecere, doar cu EEPROM-ul liber: nu blocheaza bucla
static void settings_save_task(void) {
    uint8_t i;

    if (EECON1bits.WR) {
        return;
    }
    for (i = 0; i < EE_COUNT; i++) {
        if (*settings[i] != ee_saved[i]) {
            ee_saved[i] = *settings[i];
            EEPROM_WRITE((uint8_t)(EE_ADDR_SETTINGS + i), ee_saved[i]);
            return;
        }
    }
}

// ===========================================================================
// Conversii de viteza
// ===========================================================================
static uint16_t sps_to_rpm(uint16_t sps) {
    return (uint16_t)((sps * 3U) / 10U);
}

// Rotunjita in sus, astfel incat sps_to_rpm(rpm_to_sps(r)) == r
static uint16_t rpm_to_sps(uint16_t rpm) {
    return (uint16_t)((rpm * 10U + 2U) / 3U);
}

// ===========================================================================
// Comunicatia cu sclavul (SPI)
// ===========================================================================
// Trimite o comanda; intoarce raspunsul sclavului (viteza curenta).
// spi_err: primul octet primit nu este SPI_ACK (sclav absent / desincronizat)
static int16_t send_motor_command(SPI_Commands_t command, int16_t value) {
    SPI_PACKET->cmd = (unsigned char)command;
    SPI_PACKET->val = value;
    spi_transfer_packet();
    spi_err = (SPI_PACKET->raw[0] != SPI_ACK);
    return SPI_PACKET->val;
}

// Viteza cu semn pentru CMD_SET_MOTOR_SPEED: sensul este in semn, deci se
// retrimite odata cu viteza (un sclav resetat primeste si sensul corect)
static int16_t motor_velocity(uint16_t speed) {
    return (motor_direction == DIR_CW) ? (int16_t)speed : -(int16_t)speed;
}

// Pozitia pe 32 de biti, in doua pachete; sclavul retine partea de sus la
// primul, deci jumatatile sunt din acelasi moment. Pastrata doar daca ambele
// raspunsuri incep cu SPI_ACK.
static void read_motor_position(void) {
    uint16_t low, high;
    uint8_t ok;

    low = (uint16_t)send_motor_command(CMD_GET_POSITION_LO, 0);
    ok = (uint8_t)!spi_err;
    high = (uint16_t)send_motor_command(CMD_GET_POSITION_HI, 0);
    if (ok && !spi_err) {
        reported_position = (int32_t)(((uint32_t)high << 16) | low);
    } else {
        spi_err = 1;    // valoarea veche ramane
    }
}

// Tinta in doua pachete (LO, apoi HI care o aplica); retrimisa la urmatoarea
// trecere daca un raspuns nu a inceput cu SPI_ACK
static void send_target_position(void) {
    uint8_t ok;

    send_motor_command(CMD_SET_TARGET_LO, (int16_t)(uint16_t)target_position);
    ok = (uint8_t)!spi_err;
    send_motor_command(CMD_SET_TARGET_HI,
                       (int16_t)(uint16_t)((uint32_t)target_position >> 16));
    if (ok && !spi_err) {
        target_pending = 0;
    } else {
        spi_err = 1;
    }
}

// Accelerarea in octetul low, franarea in cel high
static void send_motor_ramp(void) {
    send_motor_command(CMD_SET_MOTOR_RAMP,
                       (int16_t)(((uint16_t)motor_decel << 8) | motor_accel));
}


// ===========================================================================
// Afisaj si bargraph (HMI)
// ===========================================================================
static void display_speed(uint16_t speed_sps) {
    uint16_t rpm = sps_to_rpm(speed_sps);
    hmi_print_number((int16_t)rpm, 0, 4);
    disp_buf[4] = ' ';
    disp_buf[5] = 'R';
    disp_buf[6] = 'P';
    disp_buf[7] = 'M';
}

static void display_direction(void) {
    hmi_print_string(motor_direction == DIR_CCW ? "BACKWARD" : " FORWARD");
}

// rev / butonul de sens: doar sensul modului continuu. In modul continuu
// pleaca spre sclav cu urmatoarea viteza (se schimba semnul); in modul
// pozitie sensul vine din tinta, iar cel nou se aplica la revenirea in continuu.
static void reverse_direction(void) {
    run_direction = (run_direction == DIR_CW) ? DIR_CCW : DIR_CW;
}

// run: comuta modul continuu. Din pozitie (sau din oprirea in curs) ->
// continuu; din continuu -> oprire, apoi pozitie (vezi bucla principala).
static void toggle_continuous_mode(void) {
    if (position_mode || hold_pending) {
        position_mode = 0;
        hold_pending = 0;
    } else {
        hold_pending = 1;
    }
}

// ===========================================================================
// Distante si incetinire la obstacole
// ===========================================================================
// Obstacolul cel mai apropiat din sensul de mers (senzorii 0,1 = fata, 2,3 = spate)
static uint8_t closest_obstacle(void) {
    uint8_t first_sensor = (motor_direction == DIR_CW) ? 0U : 2U;
    uint8_t closest = sensor_distances[first_sensor];

    if (sensor_distances[first_sensor + 1U] < closest) {
        closest = sensor_distances[first_sensor + 1U];
    }
    return closest;
}

// Starea drumului, cu histerezis: intrare la prag, iesire la prag + DISTANCE_TOLERANCE
static void update_obstacle_status(void) {
    uint8_t distance = closest_obstacle();

    if (path_stop) {
        if (distance >= DISTANCE_STOP + DISTANCE_TOLERANCE) {
            path_stop = 0;
        }
    } else if (distance <= DISTANCE_STOP) {
        path_stop = 1;
    }

    if (path_slow) {
        if (distance >= DISTANCE_FAR + DISTANCE_TOLERANCE) {
            path_slow = 0;
        }
    } else if (distance < DISTANCE_FAR) {
        path_slow = 1;
    }
}

// Panta incetinirii, pasi/s pe unitate de distanta, cu 8 biti fractionari:
// cruise_speed x 256 / (FAR - STOP). Recalculata doar la cruise_speed_changed
// (impartire pe 32 de biti).
static uint16_t ramp_step_q8;

// Volumul si nota bip-ului; aplicate in bucla principala (buzzer_setup e lenta)
static void change_buzzer_volume(uint8_t increase) {
    if (increase) {
        if (buzzer_volume < 10U) {
            buzzer_volume++;
        }
    } else if (buzzer_volume > 0U) {
        buzzer_volume--;
    }
    buzzer_settings_changed = 1;
}

// Nota urmatoare / anterioara, circular
static void change_buzzer_tone(uint8_t increase) {
    if (increase) {
        buzzer_tone = (buzzer_tone >= NUM_NOTES - 1U) ? 0U : buzzer_tone + 1U;
    } else {
        buzzer_tone = (buzzer_tone == 0U) ? (uint8_t)(NUM_NOTES - 1U)
                                          : buzzer_tone - 1U;
    }
    buzzer_settings_changed = 1;
}

// Buzzer de parcare: liniste cu drum liber, bip-uri tot mai dese spre STOP,
// continuu la STOP. Aici se scriu doar contoarele; sunetul il face ISR-ul.
static void update_parking_buzzer(void) {
    uint8_t distance;

    if (path_stop) {
        buzzer_off_time = 0;                    // continuu
        buzzer_on_time = BUZZER_BEEP_TIME;
    } else if (path_slow) {
        distance = closest_obstacle();
        if (distance < DISTANCE_STOP) {
            distance = DISTANCE_STOP;
        }
        // Pauza creste cu distanta: 10 ms la fiecare 2 unitati peste STOP
        buzzer_off_time = (uint8_t)(BUZZER_MIN_GAP +
                                    ((uint8_t)(distance - DISTANCE_STOP) >> 1));
        buzzer_on_time = BUZZER_BEEP_TIME;
    } else {
        buzzer_on_time = 0;                     // oprit
    }
}

// Viteza trimisa motorului: cea de croaziera, redusa liniar intre FAR si STOP
static uint16_t calculate_effective_speed(void) {
    uint8_t distance;

    // hold_pending: iesire din modul continuu, motorul franeaza pana la 0
    if (motor_undervoltage || motor_overvoltage || path_stop || hold_pending) {
        return 0;
    }
    if (!path_slow) {
        return cruise_speed;
    }
    // In SLOW distanta poate fi inca peste FAR (banda de histerezis)
    distance = closest_obstacle();
    if (distance >= DISTANCE_FAR) {
        return cruise_speed;
    }
    if (distance <= DISTANCE_STOP) {
        return 0;
    }
    if (cruise_speed_changed) {
        cruise_speed_changed = 0;
        ramp_step_q8 = (uint16_t)(((uint32_t)cruise_speed << 8) /
                                  (DISTANCE_FAR - DISTANCE_STOP));
    }
    // Distanta (8 biti) primul: inmultirea itereaza pe bitii primului operand
    return (uint16_t)(((__uint24)(uint8_t)(distance - DISTANCE_STOP) *
                       ramp_step_q8) >> 8);
}

// LED-uri aprinse (0-10): 0 la FAR sau mai departe, 10 la STOP sau mai aproape.
// Intre ele, dintr-un tabel calculat la compilare din praguri (valoarea de la
// mijlocul fiecarui pas; langa o granita poate diferi cu un LED).
#define LED_LUT_SHIFT 3U                        // 2^LED_LUT_SHIFT unitati de distanta pe intrare
#define LED_LUT_SIZE  18U
#define LED_AT(i) ((uint8_t)(((DISTANCE_FAR - (DISTANCE_STOP + (i) * 8U + 4U)) * 10U) / \
                             (DISTANCE_FAR - DISTANCE_STOP)))
// Eroare de compilare daca zona STOP..FAR nu mai incape in tabel
typedef char led_lut_covers_zone[
    ((DISTANCE_FAR - DISTANCE_STOP + 7U) / 8U <= LED_LUT_SIZE) ? 1 : -1];

static uint8_t sensor_to_led_count(uint8_t distance) {
    static const uint8_t led_lut[LED_LUT_SIZE] = {
        LED_AT(0),  LED_AT(1),  LED_AT(2),  LED_AT(3),  LED_AT(4),  LED_AT(5),
        LED_AT(6),  LED_AT(7),  LED_AT(8),  LED_AT(9),  LED_AT(10), LED_AT(11),
        LED_AT(12), LED_AT(13), LED_AT(14), LED_AT(15), LED_AT(16), LED_AT(17)
    };

    if (distance <= DISTANCE_STOP) {
        return 10U;
    }
    if (distance >= DISTANCE_FAR) {
        return 0U;
    }
    return led_lut[(uint8_t)(distance - DISTANCE_STOP) >> LED_LUT_SHIFT];
}

// Bargraph-urile arata senzorii din sensul de mers
static void display_direction_sensors(void) {
    uint8_t first_sensor = (motor_direction == DIR_CW) ? 0U : 2U;

    hmi_set_bargraph(sensor_to_led_count(sensor_distances[first_sensor]), RED);
    hmi_set_bargraph(sensor_to_led_count(sensor_distances[first_sensor + 1U]), GRN);
}

static void update_motor_voltage_status(void) {
    uint16_t millivolts = motor_voltage_millivolts;

    if (motor_undervoltage) {
        if (millivolts >= MOTOR_VOLTAGE_MIN + MOTOR_VOLTAGE_TOLERANCE) {
            motor_undervoltage = 0;
        }
    } else if (millivolts < MOTOR_VOLTAGE_MIN - MOTOR_VOLTAGE_TOLERANCE) {
        motor_undervoltage = 1;
    }

    if (motor_overvoltage) {
        if (millivolts <= MOTOR_VOLTAGE_MAX - MOTOR_VOLTAGE_TOLERANCE) {
            motor_overvoltage = 0;
        }
    } else if (millivolts > MOTOR_VOLTAGE_MAX + MOTOR_VOLTAGE_TOLERANCE) {
        motor_overvoltage = 1;
    }

}

// ===========================================================================
// Terminal (UART + coduri ANSI), minim 31 de randuri, set de caractere CP437.
// Un chenar dublu, pe sectiuni:
//    1  STATUS     2-8   starea (rescrisa pe loc; randul 4 = antet fix)
//    9  SENSORS   10-13  senzorii
//   14  MENU      15-22  meniul (fix)
//   23  HISTORY   24-29  istoricul (regiune de derulare)
//                 30     linia de comanda: "> <ce se tasteaza>_"
//   31  marginea de jos
// Fiecare linie se formateaza direct in UART_TX_BUF, doar cand nu se transmite
// nimic, si incepe cu pozitionarea cursorului (ESC[<rand>H): liniile sunt
// independente, deci raportul si istoricul se pot intercala.
// O linie: ESC[rrH (5) + margine + BOX_W + margine = 32 = UART_TX_BUF_SIZE.
// ===========================================================================
#define BOX_W         25U              // latimea interioara a chenarului
#define BOX_RIGHT_COL (BOX_W + 2U)     // coloana marginii din dreapta

// Caracterele chenarului (CP437)
#define BOX_H  '\xCD'                   // linie orizontala dubla
#define BOX_V  '\xBA'                   // linie verticala dubla
#define BOX_TL '\xC9'                   // colt stanga sus
#define BOX_TR '\xBB'                   // colt dreapta sus
#define BOX_BL '\xC8'                   // colt stanga jos
#define BOX_BR '\xBC'                   // colt dreapta jos
#define BOX_ML '\xCC'                   // ramificatie spre dreapta (stanga)
#define BOX_MR '\xB9'                   // ramificatie spre stanga (dreapta)

#define ROW_TOP        1U
#define ROW_DIR        2U
#define ROW_PATH       3U
#define ROW_RPM_HDR    4U
#define ROW_RPM        5U
#define ROW_VMOT       6U
#define ROW_RAMP       7U
#define ROW_POS        8U
#define ROW_SENS_HDR   9U
#define ROW_SENS       10U              // SENSOR_COUNT randuri
#define ROW_MENU_HDR   14U
#define ROW_MENU       15U              // MENU_ROWS randuri
#define ROW_HIST_HDR   23U
#define ROW_HIST_TOP   24U
#define ROW_HIST_BOT   29U
#define ROW_INPUT      30U              // in afara regiunii de derulare
#define ROW_BOTTOM     31U

#define MENU_ROWS 8U
#define HIST_ROWS (ROW_HIST_BOT - ROW_HIST_TOP + 1U)

// Liniile unui raport, in ordinea trimiterii: cele fixe doar la redesenare,
// apoi starea (raportul normal incepe la TL_STATUS)
enum {
    TL_CLEAR,                           // ESC[r ESC[2J
    TL_TOP,
    TL_RPM_HDR,
    TL_SENS_HDR,
    TL_MENU_HDR,
    TL_MENU,                            // MENU_ROWS linii
    TL_HIST_HDR = TL_MENU + MENU_ROWS,
    TL_HIST,                            // HIST_ROWS randuri goale
    TL_BOTTOM = TL_HIST + HIST_ROWS,
    TL_REGION,                          // ESC[24;29r
    TL_STATUS,
    TL_DIR = TL_STATUS,
    TL_PATH,
    TL_RPM,
    TL_VMOT,
    TL_RAMP,
    TL_POS,
    TL_SENS,                            // SENSOR_COUNT linii
    TL_INPUT = TL_SENS + SENSOR_COUNT,  // linia de comanda
    TL_END
};
#define TL_IDLE 0xFFU                   // niciun raport in curs

// Secventa ESC[24;29r din TL_REGION este scrisa ca text: eroare de compilare
// daca randurile istoricului se schimba
typedef char hist_region_matches_rows[
    (ROW_HIST_TOP == 24U && ROW_HIST_BOT == 29U) ? 1 : -1];

// Stiva hardware are 8 niveluri, iar intreruperea foloseste 3 (intrarea,
// hmi_update, CALL-ul in tabelul de fonturi din hmi_asm.s). Calea de
// formatare are deci cel mult 4 niveluri sub main:
//   uart_print_status -> term_format_* -> tx_row_begin / tx_str / tx_uint /
//   tx_row_end -> tx_char / tx_csi (frunze). Ajutoarele de un rand sunt macro-uri.

// Urmatorul raport redeseneaza tot ecranul (la pornire si la Enter)
static __bit term_redraw_pending;
static uint8_t term_line = TL_IDLE;     // urmatoarea linie a raportului in curs
static uint8_t tx_len;                  // octeti formatati in UART_TX_BUF
static uint8_t tx_row;                  // indexul primului caracter din interiorul randului

// Adauga un caracter in UART_TX_BUF (prin index); ce nu mai incape se pierde
static void tx_char(char c) {
    if (tx_len < UART_TX_BUF_SIZE) {
        UART_TX_BUF[tx_len++] = (uint8_t)c;
    }
}

static void tx_str(const char *str) {
    while (*str != '\0') {
        tx_char(*str++);
    }
}

// value aliniat la dreapta pe minim width caractere (latime fixa)
static void tx_uint(uint16_t value, uint8_t width) {
    char digits[5];
    uint8_t first = 0;      // indexul primei cifre semnificative
    uint8_t count;
    uint8_t d;

    // Cifrele prin scaderi repetate, fara impartiri
    d = 0; while (value >= 10000U) { value -= 10000U; d++; } digits[0] = (char)('0' + d);
    d = 0; while (value >= 1000U)  { value -= 1000U;  d++; } digits[1] = (char)('0' + d);
    d = 0; while (value >= 100U)   { value -= 100U;   d++; } digits[2] = (char)('0' + d);
    d = 0; while (value >= 10U)    { value -= 10U;    d++; } digits[3] = (char)('0' + d);
    digits[4] = (char)('0' + (uint8_t)value);

    while (first < 4U && digits[first] == '0') {
        first++;                        // fara zerourile din fata (ramane o cifra)
    }
    count = (uint8_t)(5U - first);

    while (width > count) {
        tx_char(' ');
        width--;
    }
    while (first < 5U) {
        tx_char(digits[first++]);
    }
}

// value cu semn, aliniat la dreapta pe minim width caractere (semnul inclus).
// Tot prin scaderi repetate (pe 32 de biti), fara impartiri.
static void tx_int32(int32_t value, uint8_t width) {
    static const uint32_t powers[9] = {
        1000000000UL, 100000000UL, 10000000UL, 1000000UL, 100000UL,
        10000UL, 1000UL, 100UL, 10UL
    };
    char digits[10];
    uint32_t magnitude;
    uint8_t negative = (uint8_t)(value < 0);
    uint8_t first = 0;
    uint8_t count;
    uint8_t i;
    uint8_t d;

    // Modulul fara semn: corect si pentru INT32_MIN
    magnitude = negative ? (uint32_t)0 - (uint32_t)value : (uint32_t)value;
    for (i = 0; i < 9U; i++) {
        d = 0;
        while (magnitude >= powers[i]) {
            magnitude -= powers[i];
            d++;
        }
        digits[i] = (char)('0' + d);
    }
    digits[9] = (char)('0' + (uint8_t)magnitude);

    while (first < 9U && digits[first] == '0') {
        first++;
    }
    count = (uint8_t)(10U - first + negative);
    while (width > count) {
        tx_char(' ');
        width--;
    }
    if (negative) {
        tx_char('-');
    }
    while (first < 10U) {
        tx_char(digits[first++]);
    }
}

// ESC[<n><cmd>, n < 100: H = cursor la inceputul randului n, G = cursor la
// coloana n. Frunza: scrie direct in buffer; daca nu incape, se pierde.
static void tx_csi(uint8_t n, char cmd) {
    uint8_t tens = 0;

    if (tx_len > UART_TX_BUF_SIZE - 5U) {
        return;
    }
    while (n >= 10U) {
        n -= 10U;
        tens++;
    }
    UART_TX_BUF[tx_len++] = 0x1B;
    UART_TX_BUF[tx_len++] = '[';
    if (tens != 0U) {
        UART_TX_BUF[tx_len++] = (uint8_t)('0' + tens);
    }
    UART_TX_BUF[tx_len++] = (uint8_t)('0' + n);
    UART_TX_BUF[tx_len++] = (uint8_t)cmd;
}

// Linie noua: cursor la inceputul randului row, apoi marginea stanga
static void tx_row_begin(uint8_t row, char left) {
    tx_len = 0;
    tx_csi(row, 'H');
    tx_char(left);
    tx_row = tx_len;
}

// Rand de text: marginea stanga si un spatiu
#define tx_text_row(row) do { tx_row_begin((row), BOX_V); tx_char(' '); } while (0)

// Completeaza interiorul randului cu fill pana la BOX_W, apoi marginea dreapta.
// Campurile au latime fixa, deci randul suprascrie complet varianta anterioara.
static void tx_row_end(char fill, char right) {
    while ((uint8_t)(tx_len - tx_row) < BOX_W && tx_len < UART_TX_BUF_SIZE) {
        tx_char(fill);
    }
    tx_char(right);
}

// Trimite linia formatata (transmisia este oprita, verificat de apelant)
#define tx_send() do { UART_TX_LEN = tx_len; uart_send(); } while (0)

// Liniile fixe (doar la redesenare)
static void term_format_static(uint8_t line) {
    // Limite verificate de menu_ranges_match
    static const char *const menu[MENU_ROWS] = {
        "rpm speed      0..800",
        "inc/dec ramp   5..250",
        "vol 0..10    ton 1..25",
        "mov position (steps)",
        "rev reverse direction",
        "run continuous on/off",
        "<cmd>[+|-]N set/add/sub",
        "empty line = redraw"
    };
    uint8_t row;
    char left = BOX_ML;
    char right = BOX_MR;
    const char *title;

    if (line == TL_CLEAR) {
        // ESC[r = fara regiune de derulare, ESC[2J = sterge ecranul,
        // ESC[?25l = ascunde cursorul
        tx_len = 0;
        tx_str("\x1b[r\x1b[2J\x1b[?25l");
        return;
    }
    if (line == TL_REGION) {
        // Regiune de derulare doar pentru istoric (ROW_HIST_TOP;ROW_HIST_BOT)
        tx_len = 0;
        tx_str("\x1b[24;29r");
        return;
    }
    if (line == TL_RPM_HDR) {
        tx_text_row(ROW_RPM_HDR);
        tx_str("    CRUISE TARGET ACTUAL");
        tx_row_end(' ', BOX_V);
        return;
    }
    if (line >= TL_MENU && line < TL_HIST_HDR) {
        tx_text_row((uint8_t)(ROW_MENU + (line - TL_MENU)));
        tx_str(menu[line - TL_MENU]);
        tx_row_end(' ', BOX_V);
        return;
    }
    if (line >= TL_HIST && line < TL_BOTTOM) {
        // Rand gol de istoric: doar marginile (ecranul este deja sters)
        tx_row_begin((uint8_t)(ROW_HIST_TOP + (line - TL_HIST)), BOX_V);
        tx_csi(BOX_RIGHT_COL, 'G');
        tx_char(BOX_V);
        return;
    }

    // Margine orizontala cu titlu optional: <left>== TITLU ====...<right>
    if (line == TL_TOP) {
        row = ROW_TOP;
        left = BOX_TL;
        right = BOX_TR;
        title = "STATUS";
    } else if (line == TL_SENS_HDR) {
        row = ROW_SENS_HDR;
        title = "SENSORS";
    } else if (line == TL_MENU_HDR) {
        row = ROW_MENU_HDR;
        title = "MENU";
    } else if (line == TL_HIST_HDR) {
        row = ROW_HIST_HDR;
        title = "HISTORY";
    } else {                            // TL_BOTTOM
        row = ROW_BOTTOM;
        left = BOX_BL;
        right = BOX_BR;
        title = "";
    }
    tx_row_begin(row, left);
    tx_char(BOX_H);
    tx_char(BOX_H);
    if (*title != '\0') {
        tx_char(' ');
        tx_str(title);
        tx_char(' ');
    }
    tx_row_end(BOX_H, right);
}

// Liniile de stare. CRUISE = viteza ceruta, TARGET = trimisa (dupa
// incetinire), ACTUAL = raportata de sclav. Toate campurile au latime fixa.
static void term_format_status(uint8_t line) {
    static const char *const sensor_labels[SENSOR_COUNT] = {
        "S0F", "S1F", "S2B", "S3B"
    };
    static const char *const quarters[DISTANCE_UNITS_PER_CM] = {
        ".00", ".25", ".50", ".75"
    };
    uint8_t closest;            // sferturi de cm
    uint8_t channel;
    uint16_t millivolts;

    switch (line) {
        case TL_DIR:
            // "DIR FORWARD   TONE  1/25" (nota numarata de la 1)
            tx_text_row(ROW_DIR);
            tx_str(motor_direction == DIR_CCW ? "DIR BACKWARD" : "DIR FORWARD ");
            tx_str("  TONE ");
            tx_uint(buzzer_tone + 1U, 2);
            tx_char('/');
            tx_uint(NUM_NOTES, 1);
            break;

        case TL_PATH:
            // "PATH SLOW  73% VOL  5/10"; % = cat din viteza de croaziera e permis
            tx_text_row(ROW_PATH);
            tx_str("PATH ");
            closest = closest_obstacle();
            if (path_stop || closest <= DISTANCE_STOP) {
                tx_str("STOP     ");
            } else if (path_slow) {
                if (closest > DISTANCE_FAR) {
                    closest = DISTANCE_FAR;      // banda de histerezis: 100%
                }
                tx_str("SLOW");
                tx_uint((uint16_t)(((closest - DISTANCE_STOP) * 100U) /
                                   (DISTANCE_FAR - DISTANCE_STOP)), 4);
                tx_char('%');
            } else {
                tx_str("CLEAR    ");
            }
            tx_str(" VOL ");
            tx_uint(buzzer_volume, 2);
            tx_str("/10");
            break;

        case TL_RPM:
            // Sub antetul "CRUISE TARGET ACTUAL"
            tx_text_row(ROW_RPM);
            tx_str("RPM");
            tx_uint(sps_to_rpm(cruise_speed), 7);
            tx_uint(sps_to_rpm(effective_speed), 7);
            tx_uint(sps_to_rpm(actual_speed), 7);
            break;

        case TL_VMOT:
            // "VMOT 12.34V VOLTAGE OK"
            millivolts = motor_voltage_millivolts;
            tx_text_row(ROW_VMOT);
            tx_str("VMOT ");
            tx_uint(millivolts / 1000U, 2);
            tx_char('.');
            millivolts = (millivolts % 1000U) / 10U;
            if (millivolts < 10U) {
                tx_char('0');
            }
            tx_uint(millivolts, 1);
            tx_str("V ");
            if (spi_err) {
                tx_str("MOTOR ERROR");
            } else if (motor_overvoltage) {
                tx_str("OVERVOLTAGE");
            } else if (motor_undervoltage) {
                tx_str("UNDERVOLTAGE");
            } else {
                tx_str("VOLTAGE OK");
            }
            break;

        case TL_RAMP:
            // "ACCEL  15  DECEL  40": valorile brute (x30 RPM/s)
            tx_text_row(ROW_RAMP);
            tx_str("ACCEL");
            tx_uint(motor_accel, 4);
            tx_str("  DECEL");
            tx_uint(motor_decel, 4);
            break;

        case TL_POS:
            // "POS        -1234 MOVE": pozitia raportata de sclav (pasi) si modul
            // (RUN = continuu, STOP = iesire din continuu, se asteapta oprirea,
            // MOVE / DONE = pozitie, in drum / la tinta)
            tx_text_row(ROW_POS);
            tx_str("POS");
            tx_int32(reported_position, 12);
            tx_char(' ');
            if (hold_pending) {
                tx_str("STOP");
            } else if (!position_mode) {
                tx_str("RUN ");
            } else if (reported_position == target_position) {
                tx_str("DONE");
            } else {
                tx_str("MOVE");
            }
            break;

        default:
            // "S0F ADC 1023  12.25 cm": valoarea ADC filtrata si distanta
            channel = (uint8_t)(line - TL_SENS);
            closest = sensor_distances[channel];
            tx_text_row((uint8_t)(ROW_SENS + channel));
            tx_str(sensor_labels[channel]);
            tx_str(" ADC");
            tx_uint(sensor_values[channel], 5);
            tx_uint(closest / DISTANCE_UNITS_PER_CM, 4);
            tx_str(quarters[closest % DISTANCE_UNITS_PER_CM]);
            tx_str(" cm");
            break;
    }
    tx_row_end(' ', BOX_V);
}

// Linia de comanda "> text_": redesenata la schimbarea UART_RX_IDX si in
// fiecare raport (TL_INPUT).
static uint8_t input_shown_len = 0xFF;  // UART_RX_IDX la ultima desenare

// len caractere din UART_RX_BUF; neafisabilele apar ca '.'
static void tx_rx_text(uint8_t len) {
    uint8_t i;
    uint8_t c;

    for (i = 0; i < len; i++) {
        c = UART_RX_BUF[i];
        tx_char((c < 0x20U || c >= 0x7FU) ? '.' : (char)c);
    }
}

// Doar de pe nivelul 2 (limita de stiva)
static void term_format_input(void) {
    uint8_t len = UART_RX_IDX;          // citit o data (ISR)

    // Dupa Enter indexul e 0: linia se goleste
    if (len > UART_RX_BUF_SIZE - 1U) {
        len = UART_RX_BUF_SIZE - 1U;
    }
    input_shown_len = len;
    tx_row_begin(ROW_INPUT, BOX_V);
    tx_char('>');
    tx_char(' ');
    tx_rx_text(len);
    tx_char('_');
    tx_row_end(' ', BOX_V);
}

// Ecoul tastarii, la schimbarea UART_RX_IDX
static void uart_print_input(void) {
    if (UART_TX_STATUS == UART_TX_BUSY || UART_RX_IDX == input_shown_len) {
        return;
    }
    term_format_input();
    tx_send();
}

// Raportul de stare, o linie pe apel, ca bucla principala sa nu astepte:
// daca se transmite ceva, se intoarce imediat. Un raport incepe cand expira
// status_report_timer; cu term_redraw_pending incepe cu tot ecranul.
static void uart_print_status(void) {
    uint8_t line;

    if (UART_TX_STATUS == UART_TX_BUSY) {
        return;
    }
    if (term_line == TL_IDLE) {
        if (status_report_timer != 0U) {
            return;
        }
        status_report_timer = STATUS_REPORT_TIMER;
        line = term_redraw_pending ? (uint8_t)TL_CLEAR : (uint8_t)TL_STATUS;
        term_redraw_pending = 0;
    } else {
        line = term_line;
    }
    term_line = (line + 1U == TL_END) ? TL_IDLE : (uint8_t)(line + 1U);

    if (line < TL_STATUS) {
        term_format_static(line);
    } else if (line == TL_INPUT) {
        term_format_input();
    } else {
        term_format_status(line);
    }
    tx_send();
}

// Comenzi: <mnemonica>[ ][+|-]<numar> (rev fara numar). Fara semn = setare,
// cu semn = adunare / scadere; rezultatul e limitat.
// mov: fara semn = tinta absoluta (0..INT32_MAX), +N / -N = deplasare fata
// de tinta curenta (in modul pozitie) sau fata de pozitia citita (din modul
// continuu, din care iese); rezultatul limitat la int32.
// rev / run: fara numar (sensul modului continuu / comutarea modului continuu).
enum { UC_RPM, UC_INC, UC_DEC, UC_VOL, UC_TON, UC_MOV, UC_REV, UC_RUN, UC_COUNT };
#define UC_SET 0U
#define UC_ADD 1U
#define UC_SUB 2U
#define HIST_TEXT_W 10U         // latimea textului in istoric

// Limitele din meniu
typedef char menu_ranges_match[
    (MAX_CRUISE_RPM == 800U && RAMP_MIN == 5U && RAMP_MAX == 250U &&
     NUM_NOTES == 25U) ? 1 : -1];

// Linia terminata cu '\n' (tio: --map OCRNL); linie goala = redesenare.
// Istoric: "<text> = <rezultat>" sau "<text>?".
// Linia din istoric se formateaza in UART_TX_BUF: cat timp se transmite,
// comanda asteapta in UART_RX_BUF pana la o trecere urmatoare.
static void uart_handle_command(void) {
    static const char mnemonics[UC_COUNT][3] = {
        {'r', 'p', 'm'}, {'i', 'n', 'c'}, {'d', 'e', 'c'},
        {'v', 'o', 'l'}, {'t', 'o', 'n'}, {'m', 'o', 'v'}, {'r', 'e', 'v'},
        {'r', 'u', 'n'}
    };
    // Doar comenzile cu limite (UC_MOV are tratarea ei)
    static const uint16_t limit_min[UC_MOV] = {
        0, RAMP_MIN, RAMP_MIN, 0, 1
    };
    static const uint16_t limit_max[UC_MOV] = {
        MAX_CRUISE_RPM, RAMP_MAX, RAMP_MAX, 10, NUM_NOTES
    };
    uint8_t len;
    uint8_t cmd;
    uint8_t pos;
    uint8_t mode = UC_SET;
    uint8_t valid = 0;
    uint8_t digit;
    uint32_t value = 0;         // limitat la CMD_VALUE_MAX
    uint16_t small;             // value limitat la CMD_SMALL_MAX (comenzile cu limite)
    uint16_t current = 0;

    if (uart_receive() == 0U) {
        return;                 // nicio linie completa
    }
    if (UART_TX_STATUS == UART_TX_BUSY) {
        return;                 // bufferul de transmisie este ocupat
    }
    len = (uint8_t)(UART_RX_LEN - 1U); // excludem '\n'
    UART_RX_BUF[len] = '\0';

    // Eliminam '\r' / spatii de la final
    while (len > 0U &&
           (UART_RX_BUF[len - 1U] == '\r' || UART_RX_BUF[len - 1U] == ' ')) {
        UART_RX_BUF[--len] = '\0';
    }
    if (len == 0U) {
        // Linie goala: raportul in curs se abandoneaza si incepe imediat
        // unul nou, cu redesenarea intregului ecran
        term_redraw_pending = 1;
        term_line = TL_IDLE;
        status_report_timer = 0;
        uart_release();
        return;
    }

    // Mnemonica (| 0x20 = litera mica)
    for (cmd = 0; cmd < UC_COUNT; cmd++) {
        if (len >= 3U &&
            (UART_RX_BUF[0] | 0x20U) == (uint8_t)mnemonics[cmd][0] &&
            (UART_RX_BUF[1] | 0x20U) == (uint8_t)mnemonics[cmd][1] &&
            (UART_RX_BUF[2] | 0x20U) == (uint8_t)mnemonics[cmd][2]) {
            break;
        }
    }

    if (cmd == UC_REV || cmd == UC_RUN) {
        valid = (uint8_t)(len == 3U);
    } else if (cmd < UC_COUNT) {
        pos = 3;
        if (pos < len && UART_RX_BUF[pos] == ' ') {
            pos++;
        }
        if (pos < len && UART_RX_BUF[pos] == '+') {
            mode = UC_ADD;
            pos++;
        } else if (pos < len && UART_RX_BUF[pos] == '-') {
            mode = UC_SUB;
            pos++;
        }
        // Cel putin o cifra; peste CMD_VALUE_MAX ramane CMD_VALUE_MAX.
        // Sub CMD_VALUE_TENTH rezultatul incape pe 32 de biti (max 2147483649).
        // Inmultirea (__lmul, legata oricum de buzzer.c) e mai scurta decat
        // deplasarile pe 32 de biti, pe care compilatorul le scrie inline.
        valid = (uint8_t)(pos < len);
        while (pos < len && IS_DIGIT(UART_RX_BUF[pos])) {
            digit = (uint8_t)(UART_RX_BUF[pos] - '0');
            if (value <= CMD_VALUE_TENTH) {
                value = value * 10U + digit;
                if (value > CMD_VALUE_MAX) {
                    value = CMD_VALUE_MAX;
                }
            } else {
                value = CMD_VALUE_MAX;
            }
            pos++;
        }
        if (pos != len) {
            valid = 0;
        }
    }

    if (valid) {
        if (cmd == UC_REV) {
            reverse_direction();
        } else if (cmd == UC_RUN) {
            toggle_continuous_mode();
        } else if (cmd == UC_MOV) {
            // Baza deplasarii: tinta curenta (exacta) daca motorul este deja
            // in modul pozitie, altfel ultima pozitie citita (veche de maxim
            // POSITION_REPORT_TIMER ms). value <= INT32_MAX, deci
            // INT32_MAX - value si INT32_MIN + value incap pe 32 de biti.
            int32_t base = position_mode ? target_position : reported_position;

            if (mode == UC_ADD) {
                target_position = (base > INT32_MAX - (int32_t)value)
                    ? INT32_MAX : base + (int32_t)value;
            } else if (mode == UC_SUB) {
                target_position = (base < INT32_MIN + (int32_t)value)
                    ? INT32_MIN : base - (int32_t)value;
            } else {
                target_position = (int32_t)value;
            }
            // Din modul continuu (sau din oprirea in curs): direct in pozitie
            position_mode = 1;
            hold_pending = 0;
            target_pending = 1;
        } else {
            uint16_t lo = limit_min[cmd];
            uint16_t hi = limit_max[cmd];

            small = (value > CMD_SMALL_MAX) ? CMD_SMALL_MAX : (uint16_t)value;

            switch (cmd) {
                case UC_RPM: current = sps_to_rpm(cruise_speed); break;
                case UC_INC: current = motor_accel;              break;
                case UC_DEC: current = motor_decel;              break;
                case UC_VOL: current = buzzer_volume;            break;
                default:     current = buzzer_tone + 1U;         break; // UC_TON
            }
            // current e in [lo, hi]: fara depasiri
            if (mode == UC_ADD) {
                current = (small >= hi - current) ? hi : current + small;
            } else if (mode == UC_SUB) {
                current = (small >= current - lo) ? lo : current - small;
            } else {
                current = (small < lo) ? lo : ((small > hi) ? hi : small);
            }
            switch (cmd) {
                case UC_RPM:
                    cruise_speed = rpm_to_sps(current);
                    cruise_speed_changed = 1;
                    break;
                case UC_INC:
                    motor_accel = (uint8_t)current;
                    ramp_changed = 1;
                    break;
                case UC_DEC:
                    motor_decel = (uint8_t)current;
                    ramp_changed = 1;
                    break;
                case UC_VOL:
                    buzzer_volume = (uint8_t)current;
                    buzzer_settings_changed = 1;
                    break;
                default:        // UC_TON
                    buzzer_tone = (uint8_t)(current - 1U);
                    buzzer_settings_changed = 1;
                    break;
            }
        }
    }

    // Linia din istoric, pe ultimul rand al regiunii: '\n' deruleaza intai
    // regiunea, apoi randul nou se scrie cu margini (ESC[<col>G = marginea
    // dreapta, fara spatii de completare). Maxim 30 de octeti.
    tx_len = 0;
    tx_csi(ROW_HIST_BOT, 'H');
    tx_char('\n');
    tx_char(BOX_V);
    tx_char(' ');
    if (!valid) {
        tx_rx_text(len);                        // maxim 15 caractere
        tx_char('?');
    } else {
        if (len > HIST_TEXT_W) {
            len = HIST_TEXT_W;
        }
        tx_rx_text(len);
        for (pos = len; pos < HIST_TEXT_W; pos++) {
            tx_char(' ');
        }
        tx_str("= ");
        if (cmd == UC_REV) {
            tx_str((run_direction == DIR_CW) ? "FWD" : "BACK");
        } else if (cmd == UC_RUN) {
            tx_str(hold_pending ? "HOLD" : "RUN");
        } else if (cmd == UC_MOV) {
            // Tinta noua. Peste 6 caractere marginea dreapta nu mai incape
            // in UART_TX_BUF (se pierde), dar numarul ramane intreg.
            tx_int32(target_position, 1);
        } else {
            tx_uint(current, 1);
        }
    }
    tx_csi(BOX_RIGHT_COL, 'G');
    tx_char(BOX_V);
    tx_send();
    uart_release();
}

// ===========================================================================
// Program principal
// ===========================================================================
// Valori "nimic trimis": la schimbarea modului, comanda modului nou pleaca imediat
#define SENT_VELOCITY_NONE ((int16_t)-32768)    // |viteza| <= MAX_CRUISE_SPEED
#define SENT_LIMIT_NONE    0xFFFFU

void main(void) {
    int16_t sent_velocity;            // ultima viteza cu semn trimisa (mod viteza)
    uint16_t sent_limit = SENT_LIMIT_NONE; // ultima limita trimisa (mod pozitie)
    int16_t velocity;

    run_direction = DIR_CW;
    motor_direction = DIR_CW;
    position_mode = 0;                 // pornire in modul continuu (viteza 0)
    hold_pending = 0;
    target_pending = 0;
    term_redraw_pending = 1;
    motor_undervoltage = 0;
    motor_overvoltage = 0;
    path_stop = 0;
    path_slow = 0;
    cruise_speed_changed = 1;          // panta se calculeaza la prima folosire
    buzzer_settings_changed = 0;
    settings_load();                   // inainte de buzzer_setup si send_motor_ramp
    system_init();
    buzzer_off();                      // buzzer_init lasa PWM-ul pornit
    buzzer_setup((NoteMuzicale_t)buzzer_tone, buzzer_volume);
    sent_velocity = motor_velocity(cruise_speed);
    actual_speed = (uint16_t)send_motor_command(
        CMD_SET_MOTOR_SPEED, sent_velocity);
    send_motor_ramp();
    ramp_changed = 0;
    display_speed(actual_speed);
    display_direction_sensors();

    while (1) {
        // Pozitia de la sclav (2 pachete), periodic
        if (position_report_timer == 0U) {
            read_motor_position();
            position_report_timer = POSITION_REPORT_TIMER;
        }
        // Sensul de deplasare (obstacolele se cauta in el): in modul continuu
        // run_direction; in modul pozitie spre tinta (la tinta ramane ultimul)
        if (position_mode) {
            if (target_position > reported_position) {
                motor_direction = DIR_CW;
            } else if (target_position < reported_position) {
                motor_direction = DIR_CCW;
            }
        } else {
            motor_direction = run_direction;
        }

        // Iesirea din modul continuu: dupa ce sclavul raporteaza viteza 0,
        // pozitia proaspata devine tinta (motorul tine locul opririi). Viteza
        // vine la fiecare comanda de viteza (cel putin la MOTOR_SPEED_REPORT_TIMER).
        if (hold_pending && actual_speed == 0U && !spi_err) {
            read_motor_position();
            if (!spi_err) {
                target_position = reported_position;
                position_mode = 1;
                hold_pending = 0;
                target_pending = 1;
                position_report_timer = POSITION_REPORT_TIMER;
            }
        }

        if(analog_update()){
            update_motor_voltage_status();
            update_obstacle_status();
            update_parking_buzzer();
            display_direction_sensors();
        }
        // Volumul / nota bip-ului se aplica aici, nu in intrerupere
        if (buzzer_settings_changed) {
            buzzer_settings_changed = 0;
            buzzer_setup((NoteMuzicale_t)buzzer_tone, buzzer_volume);
        }
        settings_save_task();

        // Butoanele: viteza -/+, sensul, volumul si nota bip-ului
        switch (citeste_tastatura()) {
            case '0':
                cruise_speed = (cruise_speed >= BUTTON_LARGE_STEP)
                    ? cruise_speed - BUTTON_LARGE_STEP : 0;
                cruise_speed_changed = 1;
                break;
            case '1':
                cruise_speed = (cruise_speed >= BUTTON_SMALL_STEP)
                    ? cruise_speed - BUTTON_SMALL_STEP : 0;
                cruise_speed_changed = 1;
                break;
            case '2':
                cruise_speed = (cruise_speed <= MAX_CRUISE_SPEED - BUTTON_SMALL_STEP)
                    ? cruise_speed + BUTTON_SMALL_STEP : MAX_CRUISE_SPEED;
                cruise_speed_changed = 1;
                break;
            case '3':
                cruise_speed = (cruise_speed <= MAX_CRUISE_SPEED - BUTTON_LARGE_STEP)
                    ? cruise_speed + BUTTON_LARGE_STEP : MAX_CRUISE_SPEED;
                cruise_speed_changed = 1;
                break;
            case '4':
                reverse_direction();
                break;
            case '5':
                change_buzzer_volume(0);
                break;
            case '6':
                change_buzzer_volume(1);
                break;
            case '7':
                change_buzzer_tone(1);
                break;
            default:
                break;
        }

        uart_handle_command();
        uart_print_input();             // ecoul tastarii, inaintea raportului

        // Rampa se trimite doar la schimbare (si o data la pornire)
        if (ramp_changed) {
            ramp_changed = 0;
            send_motor_ramp();
        }

        // Raspunsul oricarei comenzi de viteza = viteza curenta
        effective_speed = calculate_effective_speed();
        if (position_mode) {
            // Tinta (la schimbare, pana e confirmata), apoi viteza maxima
            // spre ea (la schimbare si periodic)
            if (target_pending) {
                send_target_position();
            }
            if (effective_speed != sent_limit || motor_speed_report_timer == 0) {
                actual_speed = (uint16_t)send_motor_command(
                    CMD_SET_SPEED_LIMIT, (int16_t)effective_speed);
                display_speed(actual_speed);
                sent_limit = effective_speed;
                sent_velocity = SENT_VELOCITY_NONE;
                motor_speed_report_timer = MOTOR_SPEED_REPORT_TIMER;
            }
        } else {
            // Viteza cu semn (viteza + sens), la schimbare si periodic
            velocity = motor_velocity(effective_speed);
            if (velocity != sent_velocity || motor_speed_report_timer == 0) {
                actual_speed = (uint16_t)send_motor_command(
                    CMD_SET_MOTOR_SPEED, velocity);
                display_speed(actual_speed);
                sent_velocity = velocity;
                sent_limit = SENT_LIMIT_NONE;
                motor_speed_report_timer = MOTOR_SPEED_REPORT_TIMER;
            }
        }

        // Raportul de stare periodic: cel mult o linie pe trecere
        uart_print_status();
    }
}
