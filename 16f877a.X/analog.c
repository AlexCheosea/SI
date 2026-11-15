
#define _XTAL_FREQ 18432000UL
#include "analog.h"

// Tabel ADC -> distanta (sferturi de cm), calculat la compilare din curva
// senzorului V = K / d + V0 si ADC_REFERENCE_MV; interpolare liniara intre
// intrari. Capetele depasesc DISTANCE_MAX / DISTANCE_MIN, rezultatul e limitat.
// Curba este cea tipica din foaia de catalog, necalibrata.
#define SENSOR_CURVE_K_MV_CM  31600UL   // K = 31.6 V*cm
#define SENSOR_CURVE_V0_MV    440UL     // V0 = 0.44 V

#define LUT_SHIFT     5U                // 2^LUT_SHIFT trepte ADC intre intrari
#define LUT_SIZE      20U
// Calcule in sutimi de mV, pe 32 de biti
#define LUT_NUM       (SENSOR_CURVE_K_MV_CM * 100UL * DISTANCE_UNITS_PER_CM)
#define LUT_V0_100    (SENSOR_CURVE_V0_MV * 100UL)
#define LUT_ADC_V100(adc) \
    (((unsigned long)(adc) * ADC_REFERENCE_MV * 100UL) / ADC_MAX_VALUE)
// Distanta (sferturi de cm, rotunjita) pentru o valoare ADC: K / (V - V0)
#define LUT_DIST(adc) \
    ((LUT_NUM + (LUT_ADC_V100(adc) - LUT_V0_100) / 2UL) / \
     (LUT_ADC_V100(adc) - LUT_V0_100))
// Prima intrare: cea mai mica valoare ADC cu distanta <= 255 (8 biti)
#define LUT_FIRST_ADC ((uint16_t)( \
    ((LUT_NUM / 255UL + LUT_V0_100) * ADC_MAX_VALUE + \
     ADC_REFERENCE_MV * 100UL - 1UL) / (ADC_REFERENCE_MV * 100UL)))
#define LUT_LAST_ADC  ((uint16_t)(LUT_FIRST_ADC + \
    ((unsigned long)(LUT_SIZE - 1U) << LUT_SHIFT)))
#define LUT_ENTRY(i)  ((uint8_t)LUT_DIST(LUT_FIRST_ADC + \
    ((unsigned long)(i) << LUT_SHIFT)))

static const uint8_t distance_lut[LUT_SIZE] = {
    LUT_ENTRY(0),  LUT_ENTRY(1),  LUT_ENTRY(2),  LUT_ENTRY(3),  LUT_ENTRY(4),
    LUT_ENTRY(5),  LUT_ENTRY(6),  LUT_ENTRY(7),  LUT_ENTRY(8),  LUT_ENTRY(9),
    LUT_ENTRY(10), LUT_ENTRY(11), LUT_ENTRY(12), LUT_ENTRY(13), LUT_ENTRY(14),
    LUT_ENTRY(15), LUT_ENTRY(16), LUT_ENTRY(17), LUT_ENTRY(18), LUT_ENTRY(19),
};
// Eroare de compilare daca tabelul nu acopera DISTANCE_MAX .. DISTANCE_MIN
typedef char lut_far_end_ok[
    (LUT_DIST(LUT_FIRST_ADC) <= 255UL &&
     LUT_DIST(LUT_FIRST_ADC) >= DISTANCE_MAX) ? 1 : -1];
typedef char lut_near_end_ok[
    (LUT_DIST(LUT_LAST_ADC) <= DISTANCE_MIN &&
     LUT_LAST_ADC <= ADC_MAX_VALUE) ? 1 : -1];

volatile uint8_t sample_timer = 0;
volatile uint8_t motor_voltage_timer = 0;
volatile uint8_t motor_speed_report_timer = 0;
volatile uint8_t status_report_timer = 0;
volatile uint8_t position_report_timer = 0;
volatile uint16_t sensor_values[SENSOR_COUNT];
volatile uint8_t sensor_distances[SENSOR_COUNT];
volatile uint16_t motor_voltage_millivolts;

// EMA adaptiv al senzorilor: acumulatorul = valoarea x 2^SLOW. Nu poate
// depasi 16 biti: esantionul este 0..1023, iar pasul nu trece niciodata de
// tinta, deci acumulatorul ramane intre valoarea veche si tinta. Se lucreaza
// doar cu diferente pozitive.
static uint16_t sensor_filter_acc[SENSOR_COUNT];
#define SENSOR_EMA_THRESHOLD_SCALED \
    ((uint16_t)(SENSOR_EMA_THRESHOLD << SENSOR_EMA_SLOW_SHIFT))
typedef char sensor_ema_fits_16_bits[
    (SENSOR_EMA_SLOW_SHIFT > SENSOR_EMA_FAST_SHIFT &&
     SENSOR_EMA_FAST_SHIFT >= 1U &&
     SENSOR_EMA_THRESHOLD <= ADC_MAX_VALUE &&
     (ADC_MAX_VALUE << SENSOR_EMA_SLOW_SHIFT) +
         (1UL << (SENSOR_EMA_SLOW_SHIFT - 1U)) <= 32767UL) ? 1 : -1];
// EMA al tensiunii motorului: acc = acc - acc / 2^shift + esantion,
// deci acumulatorul = media x 2^shift
#define MOTOR_VOLTAGE_EMA_SHIFT 3U      // alfa = 1 / 2^shift
static uint16_t motor_voltage_acc;
static __bit sensor_filter_initialized = 0;
#if SENSOR_MEDIAN_SAMPLES > 1
// Ultimele valori de la read_adc() pe fiecare canal (cea mai veche prima)
static uint16_t sensor_median_history[SENSOR_COUNT][SENSOR_MEDIAN_SAMPLES - 1];
#endif
static __bit motor_voltage_filter_initialized = 0;

// Microvolti la intrarea divizorului pentru o treapta ADC
#define MOTOR_VOLTAGE_UV_PER_COUNT \
    ((ADC_REFERENCE_MV * ((MOTOR_VOLTAGE_DIVIDER_TOP_OHM + \
                           MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM) / 100UL) * 1000UL) / \
     (ADC_MAX_VALUE * (MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM / 100UL)))

// Rapoarte sus / jos, x1000:
//  - ideal: mV pe treapta x 1023 / Vref - 1
//  - minim (pinul <= Vref la MOTOR_SUPPLY_MAX): MOTOR_SUPPLY_MAX / Vref - 1
//  - real: din rezistente
#define MOTOR_DIVIDER_IDEAL_X1000 \
    ((MOTOR_VOLTAGE_MV_PER_COUNT * ADC_MAX_VALUE * 1000UL + \
      ADC_REFERENCE_MV / 2UL) / ADC_REFERENCE_MV - 1000UL)
#define MOTOR_DIVIDER_MIN_X1000 \
    ((MOTOR_SUPPLY_MAX * 1000UL + ADC_REFERENCE_MV - 1UL) / \
     ADC_REFERENCE_MV - 1000UL)
#define MOTOR_DIVIDER_ACTUAL_X1000 \
    ((MOTOR_VOLTAGE_DIVIDER_TOP_OHM * 10UL) / \
     (MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM / 100UL))

// Avertisment la fiecare compilare: 'char (*)[ideal][minim][real]'
// (comparatia din sizeof nu se evalueaza: fara cod, fara memorie)
typedef char motor_divider_top_over_bottom_x1000_ideal_min_actual
    [MOTOR_DIVIDER_IDEAL_X1000][MOTOR_DIVIDER_MIN_X1000][MOTOR_DIVIDER_ACTUAL_X1000];
typedef char motor_divider_info[sizeof(
    (motor_divider_top_over_bottom_x1000_ideal_min_actual *)0 == (char (*)[1])0)];

// Erori de compilare: treapta = MOTOR_VOLTAGE_MV_PER_COUNT +/- 1.5%,
typedef char motor_divider_matches_conversion[
    (MOTOR_VOLTAGE_UV_PER_COUNT >= MOTOR_VOLTAGE_MV_PER_COUNT * 985UL &&
     MOTOR_VOLTAGE_UV_PER_COUNT <= MOTOR_VOLTAGE_MV_PER_COUNT * 1015UL) ? 1 : -1];
// pinul <= Vref la MOTOR_SUPPLY_MAX,
typedef char motor_divider_covers_supply_max[
    (MOTOR_SUPPLY_MAX * (MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM / 100UL) <=
     ADC_REFERENCE_MV * ((MOTOR_VOLTAGE_DIVIDER_TOP_OHM +
                          MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM) / 100UL)) ? 1 : -1];
// capatul de scala in mV pe 16 biti
typedef char motor_voltage_fits_16_bits[
    ((ADC_MAX_VALUE << MOTOR_VOLTAGE_MV_SHIFT) <= 65535UL) ? 1 : -1];

// Acumulatorul EMA are deja shift biti fractionari: mV = acc << (MV_SHIFT - EMA_SHIFT)
typedef char motor_ema_shift_fits_conversion[
    (MOTOR_VOLTAGE_MV_SHIFT >= MOTOR_VOLTAGE_EMA_SHIFT &&
     (ADC_MAX_VALUE << MOTOR_VOLTAGE_EMA_SHIFT) <= 65535UL) ? 1 : -1];

// Tensiunea motorului in mV, din acumulatorul EMA
static uint16_t motor_acc_to_mv(uint16_t acc) {
    return (uint16_t)(acc << (MOTOR_VOLTAGE_MV_SHIFT - MOTOR_VOLTAGE_EMA_SHIFT));
}

// Intoarce distanta in sferturi de cm (DISTANCE_MIN .. DISTANCE_MAX).
static uint8_t adc_to_distance(uint16_t adc) {
    uint8_t index, frac, near, span, distance;

    if (adc >= LUT_LAST_ADC) {
        return DISTANCE_MIN;
    }
    if (adc < LUT_FIRST_ADC) {
        return DISTANCE_MAX;
    }
    adc -= LUT_FIRST_ADC;
    index = (uint8_t)(adc >> LUT_SHIFT);
    frac = (uint8_t)adc & (uint8_t)((1U << LUT_SHIFT) - 1U);
    near = distance_lut[index];                     // ADC mai mic = mai departe
    span = (uint8_t)(near - distance_lut[index + 1U]);
    // frac primul: inmultirea itereaza pe bitii primului operand
    distance = (uint8_t)(near - (uint8_t)(((uint16_t)frac * span) >> LUT_SHIFT));
    if (distance > DISTANCE_MAX) {
        return DISTANCE_MAX;
    }
    return (distance < DISTANCE_MIN) ? DISTANCE_MIN : distance;
}

void adc_init(void){
    TRISAbits.TRISA0 = 1; // TRIS: 1 = intrare, 0 = iesire
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA2 = 1;
    TRISAbits.TRISA3 = 1;
    TRISAbits.TRISA5 = 1;
    // ADCS2:ADCS = ceasul conversiei: Fosc / 2..64 sau RC intern (TAD minim 1.6 us)
    ADCON1bits.ADCS2 = 0; 
    ADCON0bits.ADCS = 0b10; 
    ADCON1bits.PCFG = 0b0010; // PCFG: ce pini AN sunt analogici si sursa referintelor (VDD / VREF)
    ADCON1bits.ADFM = 1; // ADFM: alinierea rezultatului, 1 = la dreapta, 0 = la stanga
    ADCON0bits.CHS = 0; // CHS: canalul selectat, AN0-AN7
    ADCON0bits.ADON = 1; // ADON: modulul ADC pornit / oprit

    
}

#if ADC_MEDIAN_SAMPLES != 1 && ADC_MEDIAN_SAMPLES != 3 && ADC_MEDIAN_SAMPLES != 5
#error "ADC_MEDIAN_SAMPLES trebuie sa fie 1, 3 sau 5"
#endif
#if SENSOR_MEDIAN_SAMPLES != 1 && SENSOR_MEDIAN_SAMPLES != 3 && SENSOR_MEDIAN_SAMPLES != 5
#error "SENSOR_MEDIAN_SAMPLES trebuie sa fie 1, 3 sau 5"
#endif

// Retele de comparatii pentru mediana: rezultatul in s[1] (MEDIAN3) / s[2] (MEDIAN5).
// Macro-uri, nu functii: fara nivel de stiva hardware. t = variabila temporara.
#define MEDIAN_SORT2(s, t, a, b) \
    if ((s)[a] > (s)[b]) { (t) = (s)[a]; (s)[a] = (s)[b]; (s)[b] = (t); }
#define MEDIAN3(s, t) \
    MEDIAN_SORT2(s, t, 0, 1); MEDIAN_SORT2(s, t, 1, 2); MEDIAN_SORT2(s, t, 0, 1)
#define MEDIAN5(s, t) \
    MEDIAN_SORT2(s, t, 0, 1); MEDIAN_SORT2(s, t, 3, 4); \
    MEDIAN_SORT2(s, t, 0, 3); MEDIAN_SORT2(s, t, 1, 4); \
    MEDIAN_SORT2(s, t, 1, 2); MEDIAN_SORT2(s, t, 2, 3); \
    MEDIAN_SORT2(s, t, 1, 2)

uint16_t read_adc(uint8_t channel) {
#if ADC_MEDIAN_SAMPLES > 1
    uint16_t s[ADC_MEDIAN_SAMPLES];
    uint16_t t;
    uint8_t i;
#endif

    if (channel > 4) {
        return 0; // canal invalid
    }
    ADCON0bits.CHS = channel;
#if ADC_MEDIAN_SAMPLES == 1
    __delay_us(ADC_ACQUISITION_US);
    ADCON0bits.GO = 1; // GO: 1 porneste conversia, revine la 0 la final
    while (ADCON0bits.GO);
    return ((uint16_t)(ADRESH & 0x03U) << 8) | ADRESL; // masca: rezultat pe 10 biti (aliniat la dreapta)
#else
    for (i = 0; i < ADC_MEDIAN_SAMPLES; i++) {
        __delay_us(ADC_ACQUISITION_US);
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO);
        s[i] = ((uint16_t)(ADRESH & 0x03U) << 8) | ADRESL;
    }
#if ADC_MEDIAN_SAMPLES == 3
    MEDIAN3(s, t);
    return s[1];
#else
    MEDIAN5(s, t);
    return s[2];
#endif
#endif
}

uint8_t analog_update(void) {
    uint8_t channel;
    uint16_t sample;
    uint8_t tap;
    uint8_t updated = 0;
    uint16_t target, acc, diff, filtered;
    uint8_t up;
#if SENSOR_MEDIAN_SAMPLES > 1
    uint16_t m[SENSOR_MEDIAN_SAMPLES];
    uint16_t t;
#endif

    if (motor_voltage_timer == 0) {
#if USE_DUMMY_MOTOR_VOLTAGE
        motor_voltage_millivolts = motor_acc_to_mv(
            (uint16_t)(DUMMY_MOTOR_VOLTAGE_ADC << MOTOR_VOLTAGE_EMA_SHIFT));
#else
        sample = read_adc(ADC_CHANNEL_MOTOR_VOLTAGE);
        if (motor_voltage_filter_initialized == 0) {
            // Primul esantion umple filtrul
            motor_voltage_acc = (uint16_t)(sample << MOTOR_VOLTAGE_EMA_SHIFT);
            motor_voltage_filter_initialized = 1;
        } else {
            // acc / 2^shift rotunjit: filtrul se aseaza simetric in jurul valorii
            motor_voltage_acc = (uint16_t)(motor_voltage_acc -
                ((uint16_t)(motor_voltage_acc +
                            (1U << (MOTOR_VOLTAGE_EMA_SHIFT - 1U)))
                 >> MOTOR_VOLTAGE_EMA_SHIFT) + sample);
        }
        motor_voltage_millivolts = motor_acc_to_mv(motor_voltage_acc);
#endif
        motor_voltage_timer = MOTOR_VOLTAGE_TIMER;
        updated = 1;
    }

    if (sample_timer == 0) {
#if USE_DUMMY_SENSOR_VALUES
        for (channel = 0; channel < SENSOR_COUNT; channel++) {
            sensor_values[channel] = DUMMY_SENSOR_ADC_VALUE;
            sensor_distances[channel] = adc_to_distance(sensor_values[channel]);
        }
#else
        for (channel = 0; channel < SENSOR_COUNT; channel++) {
            sample = read_adc(channel);
#if SENSOR_MEDIAN_SAMPLES > 1
            // Mediana in timp; prima trecere umple istoricul
            if (sensor_filter_initialized == 0) {
                for (tap = 0; tap < SENSOR_MEDIAN_SAMPLES - 1; tap++) {
                    sensor_median_history[channel][tap] = sample;
                }
            }
            for (tap = 0; tap < SENSOR_MEDIAN_SAMPLES - 1; tap++) {
                m[tap] = sensor_median_history[channel][tap];
                if (tap != 0) {
                    sensor_median_history[channel][tap - 1] = m[tap];
                }
            }
            m[SENSOR_MEDIAN_SAMPLES - 1] = sample;
            sensor_median_history[channel][SENSOR_MEDIAN_SAMPLES - 2] = sample;
#if SENSOR_MEDIAN_SAMPLES == 3
            MEDIAN3(m, t);
            sample = m[1];
#else
            MEDIAN5(m, t);
            sample = m[2];
#endif
#endif
            target = (uint16_t)(sample << SENSOR_EMA_SLOW_SHIFT);
            if (sensor_filter_initialized == 0) {
                // Primul esantion umple filtrul
                sensor_filter_acc[channel] = target;
            } else {
                acc = sensor_filter_acc[channel];
                up = (target >= acc);
                diff = up ? (uint16_t)(target - acc) : (uint16_t)(acc - target);
                // Pas = diferenta / 2^k, rotunjit (deplasari constante: o
                // deplasare variabila ar fi o bucla)
                if (diff > SENSOR_EMA_THRESHOLD_SCALED) {
                    diff = (uint16_t)((diff + (1U << (SENSOR_EMA_FAST_SHIFT - 1U)))
                                      >> SENSOR_EMA_FAST_SHIFT);
                } else {
                    diff = (uint16_t)((diff + (1U << (SENSOR_EMA_SLOW_SHIFT - 1U)))
                                      >> SENSOR_EMA_SLOW_SHIFT);
                }
                sensor_filter_acc[channel] =
                    up ? (uint16_t)(acc + diff) : (uint16_t)(acc - diff);
            }
            // Valoarea ADC filtrata, rotunjita
            filtered = (uint16_t)((sensor_filter_acc[channel] +
                (1U << (SENSOR_EMA_SLOW_SHIFT - 1U))) >> SENSOR_EMA_SLOW_SHIFT);
            sensor_values[channel] = filtered;
            sensor_distances[channel] = adc_to_distance(filtered);
        }
        sensor_filter_initialized = 1;
#endif
        sample_timer = SAMPLE_TIMER;
        updated = 1;
    }
    return updated;
}
