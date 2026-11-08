#include "main.h"

// ===========================================================================
// Constante
// ===========================================================================
// Motor cu 200 pasi/rotatie: RPM = pasi/s * 60 / 200 = pasi/s * 3 / 10
#define MAX_CRUISE_SPEED  5000U                            // pasi/s
#define MAX_CRUISE_RPM    ((MAX_CRUISE_SPEED * 3U) / 10U)  // 1500 RPM

// Pasii butoanelor de viteza (pasi/s)
#define BUTTON_SMALL_STEP 50U
#define BUTTON_LARGE_STEP 500U

// Distanta pana la obstacol (cm, din tabelul de calibrare de mai jos):
//   >= FAR       drum liber, viteza de croaziera
//   FAR..STOP    viteza scade liniar, bargraph-ul se umple (10 LED-uri)
//   <= STOP      oprire
#define DISTANCE_FAR_CM   100U
#define DISTANCE_STOP_CM  20U

// Tabel de calibrare pentru senzorii Sharp GP2Y0A60SZLF (versiunea de 5 V,
// referinta ADC = 5 V). Iesirea scade cu distanta, aproximativ ~ 1/d, deci
// valorile ADC sunt in ordine descrescatoare. Intre puncte se interpoleaza
// liniar. Sub 10 cm senzorul este ambiguu (iesirea scade din nou).
// VALORI ESTIMATE (V ~ 32.1/d + 0.39): se inlocuiesc cu media a cat mai multe
// citiri reale, masurate cu rigla pe o tinta mata, deschisa la culoare.
#define DISTANCE_MIN_CM   10U
#define DISTANCE_MAX_CM   100U   // ultimul punct; mai departe = "100 cm sau peste"
#define DISTANCE_TABLE_POINTS 8U
static const struct {
    uint16_t adc;
    uint8_t cm;
} distance_table[DISTANCE_TABLE_POINTS] = {
    {737,  10},
    {517,  15},
    {408,  20},
    {298,  30},
    {243,  40},
    {189,  60},
    {161,  80},
    {145, 100},
};

// Terminal: uart_print_string trimite maxim UART_BUF_SIZE-1 (15) caractere
// + '\n' si asteapta terminarea liniei anterioare (~0.42 ms/linie la 384000).
// Fiecare linie include si '\r', deci raman maxim 14 caractere vizibile.
#define LINE_MAX (UART_BUF_SIZE - 1)

#define IS_DIGIT(c) ((c) >= '0' && (c) <= '9')

// ===========================================================================
// Conversii de viteza
// ===========================================================================
static uint16_t sps_to_rpm(uint16_t sps) {
    return (uint16_t)(((uint32_t)sps * 3U) / 10U);
}

// Rotunjita in sus, astfel incat sps_to_rpm(rpm_to_sps(r)) == r
// (pasii de 5 RPM raman exacti pe afisaj).
static uint16_t rpm_to_sps(uint16_t rpm) {
    return (uint16_t)(((uint32_t)rpm * 10U + 2U) / 3U);
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
    di();
    hmi_print_number((int16_t)rpm, 0, 4);
    disp_buf[4] = ' ';
    disp_buf[5] = 'R';
    disp_buf[6] = 'P';
    disp_buf[7] = 'M';
    ei();
}

static void display_direction(uint8_t motor_direction) {
    hmi_print_string(motor_direction == DIR_CCW ? "BACKWARD" : " FORWARD");
}

static void set_motor_direction(uint8_t *motor_direction, uint8_t direction) {
    *motor_direction = direction;
    send_motor_command(CMD_SET_MOTOR_DIR, (int16_t)direction);
    display_direction(direction);
}

// ===========================================================================
// Distante si incetinire la obstacole
// ===========================================================================
// Citire ADC -> distanta in cm, prin interpolare liniara in distance_table.
// Peste primul punct: DISTANCE_MIN_CM; sub ultimul: DISTANCE_MAX_CM.
static uint8_t adc_to_cm(uint16_t adc) {
    uint8_t i;

    if (adc >= distance_table[0].adc) {
        return DISTANCE_MIN_CM;
    }
    for (i = 1; i < DISTANCE_TABLE_POINTS; i++) {
        if (adc >= distance_table[i].adc) {
            // Intre punctele i-1 (mai aproape) si i (mai departe).
            // cm_span * offset: cel mult ~50 cm * ~220 ADC, incape in 16 biti.
            uint16_t adc_span = distance_table[i - 1U].adc - distance_table[i].adc;
            uint16_t cm_span = distance_table[i].cm - distance_table[i - 1U].cm;
            uint16_t offset = distance_table[i - 1U].adc - adc;
            return (uint8_t)(distance_table[i - 1U].cm +
                             (cm_span * offset) / adc_span);
        }
    }
    return DISTANCE_MAX_CM;
}

// Distanta pana la obstacolul cel mai apropiat din sensul de mers
// (senzorii 0,1 = fata/inainte, 2,3 = spate/inapoi). Citirea ADC cea mai mare
// este obstacolul cel mai apropiat, deci convertim doar pe aceea.
static uint8_t closest_obstacle_cm(uint8_t motor_direction) {
    uint8_t first_sensor = (motor_direction == DIR_CW) ? 0U : 2U;
    uint16_t closest = sensor_values[first_sensor];

    if (sensor_values[first_sensor + 1U] > closest) {
        closest = sensor_values[first_sensor + 1U];
    }
    return adc_to_cm(closest);
}

// Viteza trimisa motorului: cea de croaziera, redusa liniar intre FAR si STOP.
static uint16_t calculate_effective_speed(uint16_t cruise_speed,
                                          uint8_t motor_direction) {
    uint8_t distance = closest_obstacle_cm(motor_direction);

    if (distance <= DISTANCE_STOP_CM) {
        return 0;
    }
    if (distance >= DISTANCE_FAR_CM) {
        return cruise_speed;
    }
    return (uint16_t)(((uint32_t)cruise_speed * (distance - DISTANCE_STOP_CM)) /
                      (DISTANCE_FAR_CM - DISTANCE_STOP_CM));
}

// Numarul de LED-uri aprinse (0-10) pentru o citire de senzor:
// 0 la FAR sau mai departe, 10 la STOP sau mai aproape.
static uint8_t sensor_to_led_count(uint16_t sensor_value) {
    uint8_t distance = adc_to_cm(sensor_value);

    if (distance <= DISTANCE_STOP_CM) {
        return 10U;
    }
    if (distance >= DISTANCE_FAR_CM) {
        return 0U;
    }
    return (uint8_t)(((DISTANCE_FAR_CM - distance) * 10U) /
                     (DISTANCE_FAR_CM - DISTANCE_STOP_CM));
}

// Bargraph-urile arata senzorii din sensul de mers.
static void display_direction_sensors(uint8_t motor_direction) {
    uint8_t first_sensor = (motor_direction == DIR_CW) ? 0U : 2U;

    hmi_set_bargraph(sensor_to_led_count(sensor_values[first_sensor]), RED);
    hmi_set_bargraph(sensor_to_led_count(sensor_values[first_sensor + 1U]), GRN);
}

// ===========================================================================
// Terminal (UART + coduri ANSI)
// Ecran (terminal de minim 29 de randuri):
//   randurile  1-11 : starea, rescrisa pe loc la fiecare raport (latime fixa)
//   randurile 13-20 : meniul de comenzi
//   randul       21 : "-- HISTORY --"
//   randurile 22-29 : istoricul comenzilor (regiune de derulare proprie)
// ===========================================================================
// Setat la pornire si la o linie goala (Enter): urmatorul raport redeseneaza
// tot ecranul (meniu + regiunea de istoric).
static uint8_t term_redraw_pending = 1;

// Adauga str in line incepand de la pos; intoarce noua pozitie.
static uint8_t line_put_str(char *line, uint8_t pos, const char *str) {
    while (*str != '\0' && pos < LINE_MAX) {
        line[pos++] = *str++;
    }
    return pos;
}

// Adauga value aliniat la dreapta pe minim width caractere (spatii in fata),
// ca liniile sa aiba latime fixa si sa se suprascrie curat.
static uint8_t line_put_uint(char *line, uint8_t pos, uint16_t value,
                             uint8_t width) {
    char digits[5];
    uint8_t count = 0;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && count < sizeof(digits));

    while (width > count && pos < LINE_MAX) {
        line[pos++] = ' ';
        width--;
    }
    while (count > 0U && pos < LINE_MAX) {
        line[pos++] = digits[--count];
    }
    return pos;
}

// Termina linia cu '\r' (driverul adauga doar '\n'; fara '\r' liniile ar iesi
// "in scara") si o trimite.
static void line_send(char *line, uint8_t pos) {
    if (pos >= LINE_MAX) {
        pos = LINE_MAX - 1U;   // pastram loc pentru '\r'
    }
    line[pos++] = '\r';
    line[pos] = '\0';
    uart_print_string(line);
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

// Tensiunea motorului in mV, din citirea ADC prin divizorul rezistiv.
// (factorii /100 tin produsul in 32 de biti: 1023 * 295000 < 2^32)
static uint16_t motor_voltage_mv(uint16_t adc) {
    return (uint16_t)(((uint32_t)adc *
        (ADC_REFERENCE_MV * ((MOTOR_VOLTAGE_DIVIDER_TOP_OHM +
                              MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM) / 100UL))) /
        (ADC_MAX_VALUE * (MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM / 100UL)));
}

/**
 * Raportul de stare, rescris pe loc in randurile 1-11:
 *   --STATUS--
 *   DIR FORWARD | DIR BACKWARD
 *   CRUISE <n>RPM   viteza setata de utilizator
 *   TARGET <n>RPM   viteza trimisa motorului (dupa incetinirea la obstacole)
 *   ACTUAL <n>RPM   viteza raportata de sclav
 *   PATH CLEAR | PATH SLOW <n>% | PATH STOP   (senzorii din sensul de mers)
 *   S0F/S1F/S2B/S3B <adc> <cm>cm   senzorii de distanta (F = fata, B = spate)
 *   VMOT <v>.<cc>V  tensiunea motorului
 * Daca term_redraw_pending e setat, intai sterge ecranul, deseneaza meniul si
 * seteaza regiunea de derulare pentru istoric.
 * Blocheaza cat dureaza trimiterea liniilor (ultima linie pleaca in fundal).
 */
static void uart_print_status(uint8_t motor_direction, uint16_t cruise_speed,
                              uint16_t effective_speed, uint16_t actual_speed) {
    static const char *const sensor_labels[SENSOR_COUNT] = {
        "S0F ", "S1F ", "S2B ", "S3B "
    };
    char line[LINE_MAX + 1];
    uint8_t pos;
    uint8_t channel;
    uint8_t closest;            // cm
    uint16_t millivolts;

    if (term_redraw_pending) {
        term_redraw_pending = 0;
        // ESC[r = fara regiune de derulare, ESC[2J = sterge ecranul
        uart_print_string("\x1b[r\x1b[2J");
        // ESC[13H = cursor la randul 13; meniul ocupa randurile 13-20
        uart_print_string("\x1b[13H--MENU--\r");
        term_print_text("1   reverse");
        term_print_text("2/3 +/-5RPM");
        term_print_text("4/5 +/-15RPM");
        term_print_text("6/7 +/-50RPM");
        term_print_text("8/9 +/-500RPM");
        term_print_text("10 max  0 stop");
        term_print_text("Enter = redraw");
        term_print_text("-- HISTORY --");          // randul 21
        // ESC[22;29r = regiune de derulare doar pentru istoric (randurile 22-29)
        uart_print_string("\x1b[22;29r");
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
    closest = closest_obstacle_cm(motor_direction);
    if (closest <= DISTANCE_STOP_CM) {
        term_print_text("PATH STOP     ");
    } else if (closest < DISTANCE_FAR_CM) {
        term_print_value("PATH SLOW ",
            (uint16_t)(((closest - DISTANCE_STOP_CM) * 100U) /
                       (DISTANCE_FAR_CM - DISTANCE_STOP_CM)), 3, "%");
    } else {
        term_print_text("PATH CLEAR    ");
    }

    // "S0F <adc> <cm>cm": citirea bruta (pentru calibrare) si distanta
    for (channel = 0; channel < SENSOR_COUNT; channel++) {
        pos = line_put_str(line, 0, sensor_labels[channel]);
        pos = line_put_uint(line, pos, sensor_values[channel], 4);
        pos = line_put_str(line, pos, " ");
        pos = line_put_uint(line, pos, adc_to_cm(sensor_values[channel]), 3);
        pos = line_put_str(line, pos, "cm");
        line_send(line, pos);
    }

    // VMOT <volti>.<sutimi>V
    millivolts = motor_voltage_mv(motor_voltage);
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
}

/**
 * Proceseaza o comanda UART: numarul din meniu, pe o linie terminata cu '\n'
 * (in tio: --map OCRNL).
 *   1 inverseaza sensul
 *   2/3 +/-5 RPM   4/5 +/-15 RPM   6/7 +/-50 RPM   8/9 +/-500 RPM
 *   10 viteza maxima   0 stop (viteza 0, cu rampa)
 * Linie goala (doar Enter): redeseneaza ecranul.
 * Fiecare comanda adauga in istoric linia "<comanda> <actiune>",
 * de ex. "6 +50RPM"; orice altceva apare ca "<text> ?".
 */
static void uart_handle_command(uint16_t *cruise_speed,
                                uint8_t *motor_direction) {
    static const uint16_t menu_steps[4] = {5, 15, 50, 500};   // comenzile 2-9
    char cmd[UART_BUF_SIZE];
    char reply[LINE_MAX + 1];
    uint8_t len;
    uint8_t pos;
    uint8_t number = 0xFF;      // numarul din meniu; 0xFF = comanda invalida
    uint8_t increase = 0;
    uint16_t delta = 0;         // pasul in RPM; 0 = nu e comanda de viteza
    const char *action = "?";

    if (uart_receive() == 0U) {
        return;                 // nicio linie completa primita
    }
    len = uart_read_string(cmd);

    // Eliminam '\r' / spatii de la final
    while (len > 0U && (cmd[len - 1U] == '\r' || cmd[len - 1U] == ' ')) {
        cmd[--len] = '\0';
    }
    if (len == 0U) {
        // Linie goala: redesenam ecranul cu urmatorul raport, chiar acum
        term_redraw_pending = 1;
        status_report_timer = 0;
        return;
    }

    // Numarul comenzii (1-2 cifre)
    if (len <= 2U && IS_DIGIT(cmd[0]) && (len == 1U || IS_DIGIT(cmd[1]))) {
        number = (uint8_t)(cmd[0] - '0');
        if (len == 2U) {
            number = (uint8_t)(number * 10U + (uint8_t)(cmd[1] - '0'));
        }
    }

    switch (number) {
        case 0:
            *cruise_speed = 0;
            action = "STOP";
            break;
        case 1:
            set_motor_direction(motor_direction,
                (*motor_direction == DIR_CW) ? DIR_CCW : DIR_CW);
            action = (*motor_direction == DIR_CW) ? "FWD" : "BACK";
            break;
        case 2: case 3: case 4: case 5:
        case 6: case 7: case 8: case 9: {
            // Pas de viteza in RPM (par = +, impar = -), limitat la 0..MAX
            uint16_t rpm = sps_to_rpm(*cruise_speed);

            delta = menu_steps[(number - 2U) / 2U];
            increase = (uint8_t)((number & 1U) == 0U);
            if (increase) {
                rpm = (delta >= MAX_CRUISE_RPM - rpm) ? MAX_CRUISE_RPM
                                                      : rpm + delta;
            } else {
                rpm = (rpm > delta) ? rpm - delta : 0;
            }
            *cruise_speed = rpm_to_sps(rpm);
            break;
        }
        case 10:
            *cruise_speed = MAX_CRUISE_SPEED;
            action = "MAX";
            break;
        default:
            break;              // necunoscuta: in istoric apare "<text> ?"
    }

    // Linia din istoric: ESC[29H = ultimul rand al regiunii; '\n'-ul adaugat de
    // uart_print_string deruleaza regiunea, deci istoricul urca cu un rand.
    pos = line_put_str(reply, 0, "\x1b[29H");
    pos = line_put_str(reply, pos, cmd);
    pos = line_put_str(reply, pos, " ");
    if (delta != 0U) {
        pos = line_put_str(reply, pos, increase ? "+" : "-");
        pos = line_put_uint(reply, pos, delta, 1);
        pos = line_put_str(reply, pos, "RPM");
    } else {
        pos = line_put_str(reply, pos, action);
    }
    reply[pos] = '\0';
    uart_print_string(reply);
}

// ===========================================================================
// Program principal
// ===========================================================================
void main(void) {
    uint16_t cruise_speed = 0;        // viteza ceruta de utilizator (pasi/s)
    uint16_t effective_speed = 0;     // viteza trimisa sclavului (dupa senzori)
    uint16_t actual_speed = 0;        // viteza raportata de sclav
    uint16_t sent_speed = 0xFFFF;     // ultima viteza trimisa (0xFFFF = niciuna)
    uint8_t motor_direction = DIR_CW;

    system_init();
    actual_speed = (uint16_t)send_motor_command(
        CMD_SET_MOTOR_SPEED, (int16_t)cruise_speed);
    send_motor_command(CMD_SET_MOTOR_DIR, (int16_t)motor_direction);
    display_speed(actual_speed);
    display_direction_sensors(motor_direction);

    while (1) {
        analog_update();
        display_direction_sensors(motor_direction);

        // Butoane: 0/1 = -500/-50 pasi/s, 2/3 = +50/+500 pasi/s,
        //          4 = inapoi, 5 = inainte
        switch (citeste_tastatura()) {
            case '0':
                cruise_speed = (cruise_speed >= BUTTON_LARGE_STEP)
                    ? cruise_speed - BUTTON_LARGE_STEP : 0;
                break;
            case '1':
                cruise_speed = (cruise_speed >= BUTTON_SMALL_STEP)
                    ? cruise_speed - BUTTON_SMALL_STEP : 0;
                break;
            case '2':
                cruise_speed = (cruise_speed <= MAX_CRUISE_SPEED - BUTTON_SMALL_STEP)
                    ? cruise_speed + BUTTON_SMALL_STEP : MAX_CRUISE_SPEED;
                break;
            case '3':
                cruise_speed = (cruise_speed <= MAX_CRUISE_SPEED - BUTTON_LARGE_STEP)
                    ? cruise_speed + BUTTON_LARGE_STEP : MAX_CRUISE_SPEED;
                break;
            case '4':
                set_motor_direction(&motor_direction, DIR_CCW);
                break;
            case '5':
                set_motor_direction(&motor_direction, DIR_CW);
                break;
            default:
                break;
        }

        uart_handle_command(&cruise_speed, &motor_direction);

        // Trimitem viteza sclavului cand se schimba si oricum la fiecare
        // MOTOR_SPEED_REPORT_TIMER ms; raspunsul este viteza lui curenta.
        effective_speed = calculate_effective_speed(cruise_speed, motor_direction);
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
            uart_print_status(motor_direction, cruise_speed,
                              effective_speed, actual_speed);
        }
    }
}
