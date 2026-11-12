#include "main.h"

// ===========================================================================
// Constante
// ===========================================================================
// Motor cu 200 pasi/rotatie: RPM = pasi/s * 60 / 200 = pasi/s * 3 / 10
#define MAX_CRUISE_SPEED  5000U                            // pasi/s
#define MAX_CRUISE_RPM    ((MAX_CRUISE_SPEED * 3U) / 10U)  // 1500 RPM

// Pasii butoanelor de viteza (pasi/s)
#define BUTTON_SMALL_STEP_RPM 10U
#define BUTTON_LARGE_STEP_RPM 100U
#define BUTTON_SMALL_STEP ((BUTTON_SMALL_STEP_RPM * 10U + 2U) / 3U)
#define BUTTON_LARGE_STEP ((BUTTON_LARGE_STEP_RPM * 10U + 2U) / 3U)

// Terminal: uart_print_string trimite maxim UART_BUF_SIZE-1 (15) caractere
// + '\n' si asteapta terminarea liniei anterioare (~0.42 ms/linie la 384000).
// Fiecare linie include si '\r', deci raman maxim 14 caractere vizibile.
#define LINE_MAX (UART_BUF_SIZE - 1)

#define IS_DIGIT(c) ((c) >= '0' && (c) <= '9')

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

// ===========================================================================
// Conversii de viteza
// ===========================================================================
static uint16_t sps_to_rpm(uint16_t sps) {
    return (uint16_t)((sps * 3U) / 10U);
}

// Rotunjita in sus, astfel incat sps_to_rpm(rpm_to_sps(r)) == r
// (pasii de 5 RPM raman exacti pe afisaj).
static uint16_t rpm_to_sps(uint16_t rpm) {
    return (uint16_t)((rpm * 10U + 2U) / 3U);
}

// ===========================================================================
// Comunicatia cu sclavul (SPI)
// ===========================================================================
// Trimite o comanda si intoarce valoarea raspunsului (viteza curenta a motorului).
static int16_t send_motor_command(SPI_Commands_t command, int16_t value) {
    SPI_PACKET->cmd = (unsigned char)command;
    SPI_PACKET->val = value;
    spi_transfer_packet();
    return SPI_PACKET->val;
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
// Distanta pana la obstacolul cel mai apropiat din sensul de mers
// (senzorii 0,1 = fata/inainte, 2,3 = spate/inapoi). Citirea ADC cea mai mare
// este obstacolul cel mai apropiat, deci convertim doar pe aceea.
static uint8_t closest_obstacle(void) {
    uint8_t first_sensor = (motor_direction == DIR_CW) ? 0U : 2U;
    uint8_t closest = sensor_distances[first_sensor];

    if (sensor_distances[first_sensor + 1U] < closest) {
        closest = sensor_distances[first_sensor + 1U];
    }
    return closest;
}

// Starea drumului, cu histerezis (ca la tensiunea motorului): se intra in
// STOP / SLOW exact la prag si se iese abia la prag + DISTANCE_TOLERANCE,
// ca zgomotul senzorului sa nu comute starea la fiecare citire.
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

// Panta rampei de incetinire: cruise_speed * 256 / (FAR - STOP), adica pasi/s
// pe sfert de centimetru, cu 8 biti fractionari. Se recalculeaza doar cand
// cruise_speed_changed este setat, fiindca impartirea pe 32 de biti e scumpa.
static uint16_t ramp_step_q8;

// Volumul si nota bip-ului. Aici se schimba doar valorile; ele sunt aplicate
// in bucla principala (buzzer_setup este lenta), nu in intrerupere.
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

// Nota urmatoare / anterioara, cu revenire la celalalt capat al listei.
static void change_buzzer_tone(uint8_t increase) {
    if (increase) {
        buzzer_tone = (buzzer_tone >= NUM_NOTES - 1U) ? 0U : buzzer_tone + 1U;
    } else {
        buzzer_tone = (buzzer_tone == 0U) ? (uint8_t)(NUM_NOTES - 1U)
                                          : buzzer_tone - 1U;
    }
    buzzer_settings_changed = 1;
}

// Buzzer de parcare: fara sunet cand drumul e liber, bip-uri tot mai dese pe
// masura ce obstacolul se apropie, sunet continuu la STOP. Aici se scriu doar
// cele doua contoare; pornirea / oprirea se face in intreruperea de 1 ms.
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
        // Pauza creste cu distanta: 10 ms pentru fiecare jumatate de cm peste
        // STOP (40 ms langa STOP, ~740 ms la FAR).
        buzzer_off_time = (uint8_t)(BUZZER_MIN_GAP +
                                    ((uint8_t)(distance - DISTANCE_STOP) >> 1));
        buzzer_on_time = BUZZER_BEEP_TIME;
    } else {
        buzzer_on_time = 0;                     // oprit
    }
}

// Viteza trimisa motorului: cea de croaziera, redusa liniar intre FAR si STOP.
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
    // Distanta (8 biti) este primul operand: inmultirea face cate o iteratie
    // pentru fiecare bit al primului operand. Produsul incape pe 24 de biti.
    return (uint16_t)(((__uint24)(uint8_t)(distance - DISTANCE_STOP) *
                       ramp_step_q8) >> 8);
}

// Numarul de LED-uri aprinse (0-10) pentru o citire de senzor:
// 0 la FAR sau mai departe, 10 la STOP sau mai aproape.
//
// Intre STOP si FAR numarul vine dintr-un tabel cu o intrare la fiecare 8
// unitati de distanta (2 cm), ca sa nu se faca o inmultire si o impartire la
// fiecare actualizare. Intrarile se calculeaza la compilare din DISTANCE_FAR si
// DISTANCE_STOP (valoarea formulei la mijlocul fiecarui pas), deci urmaresc
// automat pragurile. Fata de formula exacta, langa o granita poate aparea un
// LED in plus sau in minus.
#define LED_LUT_SHIFT 3U                        // 2^3 = 8 unitati pe intrare
#define LED_LUT_SIZE  18U                       // acopera o zona de pana la 144 de unitati (36 cm)
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

// Bargraph-urile arata senzorii din sensul de mers.
static void display_direction_sensors(void) {
    uint8_t first_sensor = (motor_direction == DIR_CW) ? 0U : 2U;

    hmi_set_bargraph(sensor_to_led_count(sensor_distances[first_sensor]), RED);
    hmi_set_bargraph(sensor_to_led_count(sensor_distances[first_sensor + 1U]), GRN);
}

// ===========================================================================
// Terminal (UART + coduri ANSI)
// Ecran (terminal de minim 32 de randuri):
//   randurile  1-14 : starea, rescrisa pe loc la fiecare raport (latime fixa)
//   randurile 16-23 : meniul de comenzi
//   randul       24 : "-- HISTORY --"
//   randurile 25-32 : istoricul comenzilor (regiune de derulare proprie)
// ===========================================================================
// Setat la pornire si la o linie goala (Enter): urmatorul raport redeseneaza
// tot ecranul (meniu + regiunea de istoric).
static __bit term_redraw_pending;

// Adauga str in line incepand de la pos; intoarce noua pozitie.
static uint8_t line_put_str(volatile char *line, uint8_t pos,
                            const volatile char *str) {
    while (*str != '\0' && pos < LINE_MAX) {
        line[pos++] = *str++;
    }
    return pos;
}

// Adauga value aliniat la dreapta pe minim width caractere (spatii in fata),
// ca liniile sa aiba latime fixa si sa se suprascrie curat.
static uint8_t line_put_uint(volatile char *line, uint8_t pos, uint16_t value,
                             uint8_t width) {
    char digits[5];
    uint8_t first = 0;      // indexul primei cifre semnificative
    uint8_t count;
    uint8_t d;

    // Cifrele se obtin prin scaderi repetate, de la cea mai mare putere a lui
    // 10: fara impartiri (o impartire pe 16 biti costa cateva sute de cicluri).
    d = 0; while (value >= 10000U) { value -= 10000U; d++; } digits[0] = (char)('0' + d);
    d = 0; while (value >= 1000U)  { value -= 1000U;  d++; } digits[1] = (char)('0' + d);
    d = 0; while (value >= 100U)   { value -= 100U;   d++; } digits[2] = (char)('0' + d);
    d = 0; while (value >= 10U)    { value -= 10U;    d++; } digits[3] = (char)('0' + d);
    digits[4] = (char)('0' + (uint8_t)value);

    while (first < 4U && digits[first] == '0') {
        first++;                        // sarim zerourile din fata (ramane macar o cifra)
    }
    count = (uint8_t)(5U - first);

    while (width > count && pos < LINE_MAX) {
        line[pos++] = ' ';
        width--;
    }
    while (first < 5U && pos < LINE_MAX) {
        line[pos++] = digits[first++];
    }
    return pos;
}

// Adauga o distanta (tinuta in sferturi de cm) ca "cc.ff", in centimetri cu
// doua zecimale exacte (.00 / .25 / .50 / .75).
static uint8_t line_put_distance(volatile char *line, uint8_t pos,
                                 uint8_t distance) {
    static const char *const quarters[DISTANCE_UNITS_PER_CM] = {
        ".00", ".25", ".50", ".75"
    };

    pos = line_put_uint(line, pos, distance / DISTANCE_UNITS_PER_CM, 2);
    return line_put_str(line, pos, quarters[distance % DISTANCE_UNITS_PER_CM]);
}

// Termina linia cu '\r' (driverul adauga doar '\n'; fara '\r' liniile ar iesi
// "in scara") si o trimite.
static void line_send(volatile char *line, uint8_t pos) {
    if (pos >= LINE_MAX) {
        pos = LINE_MAX - 1U;   // pastram loc pentru '\r'
    }
    line[pos++] = '\r';
    line[pos] = '\0';
    uart_print_string((const char *)line);
}

// Trimite un text fix ca o linie.
static void term_print_text(const char *text) {
    char line[LINE_MAX + 1];
    line_send(line, line_put_str(line, 0, text));
}

// Trimite "<label><value><unit>" ca o linie (value pe width caractere).
static void term_print_value(const char *label, uint16_t value,
                             uint8_t width, const char *unit) {
    char line[LINE_MAX + 1];
    uint8_t pos;

    pos = line_put_str(line, 0, label);
    pos = line_put_uint(line, pos, value, width);
    pos = line_put_str(line, pos, unit);
    line_send(line, pos);
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

/**
 * Raportul de stare, rescris pe loc in randurile 1-14:
 *   --STATUS--
 *   DIR FORWARD | DIR BACKWARD
 *   CRUISE <n>RPM   viteza setata de utilizator
 *   TARGET <n>RPM   viteza trimisa motorului (dupa incetinirea la obstacole)
 *   ACTUAL <n>RPM   viteza raportata de sclav
 *   PATH CLEAR | PATH SLOW <n>% | PATH STOP   (senzorii din sensul de mers)
 *   S0F/S1F/S2B/S3B <adc> <cm>.<cc>  senzorii de distanta (F = fata, B = spate),
 *                                    in centimetri cu pas de 0.25
 *   VMOT <v>.<cc>V  tensiunea motorului
 *   VOLTAGE OK | UNDERVOLTAGE | OVERVOLTAGE
 *   VOLUME <n>/10   volumul bip-ului
 *   TONE <n>/<total>  nota bip-ului (1 = cea mai joasa)
 * Daca term_redraw_pending e setat, intai sterge ecranul, deseneaza meniul si
 * seteaza regiunea de derulare pentru istoric.
 * Blocheaza cat dureaza trimiterea liniilor (ultima linie pleaca in fundal).
 */
static void uart_print_status(void) {
    static const char *const sensor_labels[SENSOR_COUNT] = {
        "S0F ", "S1F ", "S2B ", "S3B "
    };
    char line[LINE_MAX + 1];
    uint8_t pos;
    uint8_t channel;
    uint8_t closest;            // sferturi de cm
    uint16_t millivolts;

    if (term_redraw_pending) {
        term_redraw_pending = 0;
        // ESC[r = fara regiune de derulare, ESC[2J = sterge ecranul
        uart_print_string("\x1b[r\x1b[2J");
        // ESC[16H = cursor la randul 16; meniul ocupa randurile 16-23
        uart_print_string("\x1b[16H--MENU--\r");
        term_print_text("1   reverse");
        term_print_text("2/3 +/-10RPM");
        term_print_text("4/5 +/-100RPM");
        term_print_text("6 max   0 stop");
        term_print_text("8/9 +/-VOLUME");
        term_print_text("10/11 +/-TONE");
        term_print_text("Enter = redraw");
        term_print_text("-- HISTORY --");          // randul 24
        // ESC[25;32r = regiune de derulare doar pentru istoric (randurile 25-32)
        uart_print_string("\x1b[25;32r");
    }

    // ESC[H = cursor la (1,1); liniile au latime fixa si se suprascriu
    term_print_text("\x1b[H--STATUS--");
    term_print_text(motor_direction == DIR_CCW ? "DIR BACKWARD"
                                               : "DIR FORWARD ");
    term_print_value("CRUISE ", sps_to_rpm(cruise_speed), 4, "RPM");
    term_print_value("TARGET ", sps_to_rpm(effective_speed), 4, "RPM");
    term_print_value("ACTUAL ", sps_to_rpm(actual_speed), 4, "RPM");

    // Starea drumului, cu aceleasi praguri ca la incetinire (% = cat din
    // viteza de croaziera e permis). Toate variantele au 14 caractere.
    closest = closest_obstacle();
    if (path_stop || closest <= DISTANCE_STOP) {
        term_print_text("PATH STOP     ");
    } else if (path_slow) {
        if (closest > DISTANCE_FAR) {
            closest = DISTANCE_FAR;      // banda de histerezis: 100%
        }
        term_print_value("PATH SLOW ",
            (uint16_t)(((closest - DISTANCE_STOP) * 100U) /
                       (DISTANCE_FAR - DISTANCE_STOP)), 3, "%");
    } else {
        term_print_text("PATH CLEAR    ");
    }

    // "S0F <adc> <cm>.<cc>": citirea bruta (pentru calibrare) si distanta in
    // cm. Linia are exact 14 caractere, deci unitatea nu mai incape.
    for (channel = 0; channel < SENSOR_COUNT; channel++) {
        pos = line_put_str(line, 0, sensor_labels[channel]);
        pos = line_put_uint(line, pos, sensor_values[channel], 4);
        pos = line_put_str(line, pos, " ");
        pos = line_put_distance(line, pos, sensor_distances[channel]);
        line_send(line, pos);
    }

    // VMOT <volti>.<sutimi>V
    millivolts = motor_voltage_millivolts;
    pos = line_put_str(line, 0, "VMOT ");
    pos = line_put_uint(line, pos, millivolts / 1000U, 2);
    pos = line_put_str(line, pos, ".");
    millivolts = (millivolts % 1000U) / 10U;
    if (millivolts < 10U) {
        pos = line_put_str(line, pos, "0");
    }
    pos = line_put_uint(line, pos, millivolts, 1);
    pos = line_put_str(line, pos, "V");
    line_send(line, pos);
    if (motor_overvoltage) {
        term_print_text("OVERVOLTAGE ");
    } else if (motor_undervoltage) {
        term_print_text("UNDERVOLTAGE");
    } else {
        term_print_text("VOLTAGE OK  ");
    }

    // Buzzer: "VOLUME <n>/10" si "TONE <n>/<total>" (nota numarata de la 1)
    term_print_value("VOLUME ", buzzer_volume, 2, "/10");
    pos = line_put_str(line, 0, "TONE ");
    pos = line_put_uint(line, pos, buzzer_tone + 1U, 2);
    pos = line_put_str(line, pos, "/");
    pos = line_put_uint(line, pos, NUM_NOTES, 1);
    line_send(line, pos);
}

/**
 * Proceseaza o comanda UART: numarul din meniu, pe o linie terminata cu '\n'
 * (in tio: --map OCRNL).
 *   1 inverseaza sensul
 *   2/3 +/-10 RPM   4/5 +/-100 RPM
 *   6 viteza maxima   0 stop (viteza 0, cu rampa)
 *   8/9 +/- volumul bip-ului   10/11 +/- nota bip-ului (cu revenire)
 * Linie goala (doar Enter): redeseneaza ecranul.
 * Fiecare comanda adauga in istoric linia "<comanda> <actiune>",
 * de ex. "2 +10RPM"; orice altceva apare ca "<text> ?".
 */
static void uart_handle_command(void) {
    static const uint16_t menu_steps[2] = {10, 100};          // comenzile 2-5
    uint8_t len;
    uint8_t pos;
    uint8_t number = 0xFF;      // numarul din meniu; 0xFF = comanda invalida
    uint8_t increase = 0;
    uint16_t delta = 0;         // pasul in RPM; 0 = nu e comanda de viteza
    const char *action = "?";
    char line[LINE_MAX + 1];    // linia din istoric

    if (uart_receive() == 0U) {
        return;                 // nicio linie completa primita
    }
    len = (uint8_t)(UART_RX_LEN - 1U); // excludem '\n'
    UART_RX_BUF[len] = '\0';

    // Eliminam '\r' / spatii de la final
    while (len > 0U &&
           (UART_RX_BUF[len - 1U] == '\r' || UART_RX_BUF[len - 1U] == ' ')) {
        UART_RX_BUF[--len] = '\0';
    }
    if (len == 0U) {
        // Linie goala: redesenam ecranul cu urmatorul raport, chiar acum
        term_redraw_pending = 1;
        status_report_timer = 0;
        uart_release();
        return;
    }

    // Comenzile valide au cel mult doua cifre.
    if (len <= 2U && IS_DIGIT(UART_RX_BUF[0]) &&
        (len == 1U || IS_DIGIT(UART_RX_BUF[1]))) {
        number = (uint8_t)(UART_RX_BUF[0] - '0');
        if (len == 2U) {
            number = (uint8_t)(number * 10U +
                               (uint8_t)(UART_RX_BUF[1] - '0'));
        }
    }

    switch (number) {
        case 0:
            cruise_speed = 0;
            cruise_speed_changed = 1;
            action = "STOP";
            break;
        case 1:
            set_motor_direction((motor_direction == DIR_CW) ? DIR_CCW : DIR_CW);
            action = (motor_direction == DIR_CW) ? "FWD" : "BACK";
            break;
        case 2: case 3: case 4: case 5: {
            // Pas de viteza in RPM (par = +, impar = -), limitat la 0..MAX
            uint16_t rpm = sps_to_rpm(cruise_speed);

            delta = menu_steps[(number - 2U) / 2U];
            increase = (uint8_t)((number & 1U) == 0U);
            if (increase) {
                rpm = (delta >= MAX_CRUISE_RPM - rpm) ? MAX_CRUISE_RPM
                                                      : rpm + delta;
            } else {
                rpm = (rpm > delta) ? rpm - delta : 0;
            }
            cruise_speed = rpm_to_sps(rpm);
            cruise_speed_changed = 1;
            break;
        }
        case 6:
            cruise_speed = MAX_CRUISE_SPEED;
            cruise_speed_changed = 1;
            action = "MAX";
            break;
        case 8: case 9:
            change_buzzer_volume((uint8_t)(number == 8U));
            action = (number == 8U) ? "VOL+" : "VOL-";
            break;
        case 10: case 11:
            change_buzzer_tone((uint8_t)(number == 10U));
            action = (number == 10U) ? "TONE+" : "TONE-";
            break;
        default:
            break;              // necunoscuta: in istoric apare "<text> ?"
    }

    // Linia din istoric: ESC[32H = ultimul rand al regiunii; '\n'-ul adaugat de
    // uart_print_string deruleaza regiunea, deci istoricul urca cu un rand.
    // Linia se construieste intr-un buffer local (Bank 0), NU in UART_RX_BUF:
    // acesta e definit in asamblare (Bank 3), compilatorul nu-i cunoaste
    // bancul si un pointer catre el ar scrie in Bank 0/1 (IRP = 0).
    // UART_RX_BUF este accesat doar direct, prin index.
    pos = line_put_str(line, 0, "\x1b[32H");
    if (number == 0xFF) {
        pos = line_put_str(line, pos, "?");
    } else {
        line[pos++] = (char)UART_RX_BUF[0];     // cel mult doua cifre
        if (len > 1U) {
            line[pos++] = (char)UART_RX_BUF[1];
        }
        pos = line_put_str(line, pos, " ");
        if (delta != 0U) {
            pos = line_put_str(line, pos, increase ? "+" : "-");
            pos = line_put_uint(line, pos, delta, 1);
            pos = line_put_str(line, pos, "RPM");
        } else {
            pos = line_put_str(line, pos, action);
        }
    }
    line_send(line, pos);
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
    cruise_speed_changed = 1;          // calculeaza panta la prima folosire
    buzzer_settings_changed = 0;
    system_init();
    buzzer_off();                      // buzzer_init lasa PWM-ul pornit (cu factor 0)
    buzzer_setup((NoteMuzicale_t)buzzer_tone, buzzer_volume);
    actual_speed = (uint16_t)send_motor_command(
        CMD_SET_MOTOR_SPEED, (int16_t)cruise_speed);
    send_motor_command(CMD_SET_MOTOR_DIR, (int16_t)motor_direction);
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

        // Butoane: 0/1 = -100/-10 RPM, 2/3 = +10/+100 RPM,
        //          4 = schimba sensul, 5/6 = volum bip -/+,
        //          7 = nota urmatoare a bip-ului (cu revenire la prima)
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

        // Trimitem viteza sclavului cand se schimba si oricum la fiecare
        // MOTOR_SPEED_REPORT_TIMER ms; raspunsul este viteza lui curenta.
        effective_speed = calculate_effective_speed();
        if (effective_speed != sent_speed || motor_speed_report_timer == 0) {
            actual_speed = (uint16_t)send_motor_command(
                CMD_SET_MOTOR_SPEED, (int16_t)effective_speed);
            display_speed(actual_speed);
            sent_speed = effective_speed;
            motor_speed_report_timer = MOTOR_SPEED_REPORT_TIMER;
        }

        // Raport de stare pe UART la fiecare STATUS_REPORT_TIMER ms
        if (status_report_timer == 0) {
            status_report_timer = STATUS_REPORT_TIMER;
            uart_print_status();
        }
    }
}
