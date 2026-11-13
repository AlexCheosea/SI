#include "main.h"

// ===========================================================================
// Constante
// ===========================================================================
// Motor cu 200 pasi/rotatie: RPM = pasi/s x 60 / 200 = pasi/s x 3 / 10
#define MAX_CRUISE_SPEED  5000U                            // pasi/s
#define MAX_CRUISE_RPM    ((MAX_CRUISE_SPEED * 3U) / 10U)

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
// Trimite o comanda; intoarce raspunsul sclavului (viteza curenta)
static int16_t send_motor_command(SPI_Commands_t command, int16_t value) {
    SPI_PACKET->cmd = (unsigned char)command;
    SPI_PACKET->val = value;
    spi_transfer_packet();
    return SPI_PACKET->val;
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

static void set_motor_direction(uint8_t direction) {
    motor_direction = (direction != 0U);
    send_motor_command(CMD_SET_MOTOR_DIR, (int16_t)direction);
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

    if (motor_undervoltage || motor_overvoltage || path_stop) {
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
//    1  STATUS     2-7   starea (rescrisa pe loc; randul 4 = antet fix)
//    8  SENSORS    9-12  senzorii
//   13  MENU      14-21  meniul (fix)
//   22  HISTORY   23-29  istoricul (regiune de derulare)
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
#define ROW_SENS_HDR   8U
#define ROW_SENS       9U               // SENSOR_COUNT randuri
#define ROW_MENU_HDR   13U
#define ROW_MENU       14U              // MENU_ROWS randuri
#define ROW_HIST_HDR   22U
#define ROW_HIST_TOP   23U
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
    TL_REGION,                          // ESC[23;29r
    TL_STATUS,
    TL_DIR = TL_STATUS,
    TL_PATH,
    TL_RPM,
    TL_VMOT,
    TL_RAMP,
    TL_SENS,                            // SENSOR_COUNT linii
    TL_INPUT = TL_SENS + SENSOR_COUNT,  // linia de comanda
    TL_END
};
#define TL_IDLE 0xFFU                   // niciun raport in curs

// Secventa ESC[23;29r din TL_REGION este scrisa ca text: eroare de compilare
// daca randurile istoricului se schimba
typedef char hist_region_matches_rows[
    (ROW_HIST_TOP == 23U && ROW_HIST_BOT == 29U) ? 1 : -1];

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
        "rpm speed      0..1500",
        "inc accel      5..250",
        "dec decel      5..250",
        "vol volume     0..10",
        "ton tone       1..25",
        "rev reverse",
        "N set   +N / -N add/sub",
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
        tx_str("\x1b[23;29r");
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
            if (motor_overvoltage) {
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
enum { UC_RPM, UC_INC, UC_DEC, UC_VOL, UC_TON, UC_REV, UC_COUNT };
#define UC_SET 0U
#define UC_ADD 1U
#define UC_SUB 2U
#define HIST_TEXT_W 10U         // latimea textului in istoric

// Limitele din meniu
typedef char menu_ranges_match[
    (MAX_CRUISE_RPM == 1500U && RAMP_MIN == 5U && RAMP_MAX == 250U &&
     NUM_NOTES == 25U) ? 1 : -1];

// Linia terminata cu '\n' (tio: --map OCRNL); linie goala = redesenare.
// Istoric: "<text> = <rezultat>" sau "<text>?".
// Linia din istoric se formateaza in UART_TX_BUF: cat timp se transmite,
// comanda asteapta in UART_RX_BUF pana la o trecere urmatoare.
static void uart_handle_command(void) {
    static const char mnemonics[UC_COUNT][3] = {
        {'r', 'p', 'm'}, {'i', 'n', 'c'}, {'d', 'e', 'c'},
        {'v', 'o', 'l'}, {'t', 'o', 'n'}, {'r', 'e', 'v'}
    };
    static const uint16_t limit_min[UC_REV] = {
        0, RAMP_MIN, RAMP_MIN, 0, 1
    };
    static const uint16_t limit_max[UC_REV] = {
        MAX_CRUISE_RPM, RAMP_MAX, RAMP_MAX, 10, NUM_NOTES
    };
    uint8_t len;
    uint8_t cmd;
    uint8_t pos;
    uint8_t mode = UC_SET;
    uint8_t valid = 0;
    uint16_t value = 0;
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

    if (cmd == UC_REV) {
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
        // Cel putin o cifra; peste 9999 ramane 9999 (oricum limitat)
        valid = (uint8_t)(pos < len);
        while (pos < len && IS_DIGIT(UART_RX_BUF[pos])) {
            value = (value < 1000U)
                ? (uint16_t)(value * 10U + (uint8_t)(UART_RX_BUF[pos] - '0'))
                : 9999U;
            pos++;
        }
        if (pos != len) {
            valid = 0;
        }
    }

    if (valid) {
        if (cmd == UC_REV) {
            set_motor_direction((motor_direction == DIR_CW) ? DIR_CCW : DIR_CW);
        } else {
            uint16_t lo = limit_min[cmd];
            uint16_t hi = limit_max[cmd];

            switch (cmd) {
                case UC_RPM: current = sps_to_rpm(cruise_speed); break;
                case UC_INC: current = motor_accel;              break;
                case UC_DEC: current = motor_decel;              break;
                case UC_VOL: current = buzzer_volume;            break;
                default:     current = buzzer_tone + 1U;         break; // UC_TON
            }
            // current e in [lo, hi]: fara depasiri
            if (mode == UC_ADD) {
                current = (value >= hi - current) ? hi : current + value;
            } else if (mode == UC_SUB) {
                current = (value >= current - lo) ? lo : current - value;
            } else {
                current = (value < lo) ? lo : ((value > hi) ? hi : value);
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
            tx_str((motor_direction == DIR_CW) ? "FWD" : "BACK");
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
void main(void) {
    uint16_t sent_speed = 0xFFFF;     // ultima viteza trimisa (0xFFFF = niciuna)

    motor_direction = DIR_CW;
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
    actual_speed = (uint16_t)send_motor_command(
        CMD_SET_MOTOR_SPEED, (int16_t)cruise_speed);
    send_motor_command(CMD_SET_MOTOR_DIR, (int16_t)motor_direction);
    send_motor_ramp();
    ramp_changed = 0;
    display_speed(actual_speed);
    display_direction_sensors();

    while (1) {
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
                set_motor_direction((motor_direction == DIR_CW) ? DIR_CCW : DIR_CW);
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

        // Viteza se trimite la schimbare si periodic; raspunsul = viteza curenta
        effective_speed = calculate_effective_speed();
        if (effective_speed != sent_speed || motor_speed_report_timer == 0) {
            actual_speed = (uint16_t)send_motor_command(
                CMD_SET_MOTOR_SPEED, (int16_t)effective_speed);
            display_speed(actual_speed);
            sent_speed = effective_speed;
            motor_speed_report_timer = MOTOR_SPEED_REPORT_TIMER;
        }

        // Raportul de stare periodic: cel mult o linie pe trecere
        uart_print_status();
    }
}
