
#define _XTAL_FREQ 18432000UL
#include "analog.h"

// Tabel de conversie ADC -> distanta (sferturi de cm): o intrare la fiecare
// 32 de trepte ADC, de la LUT_FIRST_ADC la LUT_LAST_ADC, cu interpolare liniara
// intre intrari. Valorile urca peste DISTANCE_MAX / coboara sub DISTANCE_MIN la
// capete ca interpolarea sa treaca exact prin limite; rezultatul este limitat.
//
// Tabelul este calculat la compilare din curva senzorului si ADC_REFERENCE_MV,
// deci urmeaza automat tensiunea reala de referinta (VDD). Pe PIC nu se face
// niciun calcul: rezultatul ajunge in memoria program ca si inainte.
//
// ATENTIE: curba NU este calibrata. Este curba tipica din foaia de catalog a
// GP2Y0A60SZ in modul 5 V, V = K / d + V0, si trebuie inlocuita cu masuratori
// pe senzorii reali (sau K si V0 ajustate dupa masuratori).
#define SENSOR_CURVE_K_MV_CM  31600UL   // K = 31.6 V*cm
#define SENSOR_CURVE_V0_MV    440UL     // V0 = 0.44 V

#define LUT_SHIFT     5U                // 2^5 = 32 trepte ADC intre intrari
#define LUT_SIZE      20U
// Calculele se fac in sutimi de mV, pe 32 de biti (maxim ~5.6e8, incape).
#define LUT_NUM       (SENSOR_CURVE_K_MV_CM * 100UL * DISTANCE_UNITS_PER_CM)
#define LUT_V0_100    (SENSOR_CURVE_V0_MV * 100UL)
#define LUT_ADC_V100(adc) \
    (((unsigned long)(adc) * ADC_REFERENCE_MV * 100UL) / ADC_MAX_VALUE)
// Distanta (sferturi de cm, rotunjita) pentru o valoare ADC: K / (V - V0)
#define LUT_DIST(adc) \
    ((LUT_NUM + (LUT_ADC_V100(adc) - LUT_V0_100) / 2UL) / \
     (LUT_ADC_V100(adc) - LUT_V0_100))
// Prima intrare: cea mai mica valoare ADC la care distanta incape pe 8 biti
// (<= 255), adica tabelul acopera cat mai departe posibil.
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
// Erori de compilare daca referinta sau curva scot tabelul din domeniu:
// capatul departat trebuie sa incapa pe 8 biti si sa treaca de DISTANCE_MAX,
// cel apropiat sa coboare pana la DISTANCE_MIN fara sa depaseasca 1023.
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
volatile uint16_t sensor_values[SENSOR_COUNT];
volatile uint8_t sensor_distances[SENSOR_COUNT];
volatile uint16_t motor_voltage_millivolts;

// Filtrul senzorilor de distanta: EMA adaptiv, dupa medianele din read_adc()
// si analog_update(). Acumulatorul tine valoarea de 2^SLOW ori (32 de ori,
// 5 biti fractionari), iar la fiecare esantion se apropie de tinta
// (esantion << SLOW) cu 1/4 din distanta (abatere peste prag) sau cu 1/32.
// Constante de timp la SAMPLE_TIMER = 16 ms: ~512 ms lent, ~64 ms rapid.
//
// Calculul pe 16 biti nu poate depasi pentru nicio citire:
//  - esantionul este 0..1023 (masca din read_adc(), medianele aleg una din
//    valorile de intrare), deci tinta este 0..32736;
//  - pasul este cel mult distanta pana la tinta si are semnul ei, deci
//    acumulatorul ramane intotdeauna intre valoarea veche si tinta: 0..32736;
//  - se lucreaza numai cu diferente pozitive (fara deplasari de numere cu
//    semn), maximul intermediar este 32736 + 16.
static uint16_t sensor_filter_acc[SENSOR_COUNT];
#define SENSOR_EMA_THRESHOLD_SCALED \
    ((uint16_t)(SENSOR_EMA_THRESHOLD << SENSOR_EMA_SLOW_SHIFT))
typedef char sensor_ema_fits_16_bits[
    (SENSOR_EMA_SLOW_SHIFT > SENSOR_EMA_FAST_SHIFT &&
     SENSOR_EMA_FAST_SHIFT >= 1U &&
     SENSOR_EMA_THRESHOLD <= ADC_MAX_VALUE &&
     (ADC_MAX_VALUE << SENSOR_EMA_SLOW_SHIFT) +
         (1UL << (SENSOR_EMA_SLOW_SHIFT - 1U)) <= 32767UL) ? 1 : -1];
// Filtrul tensiunii motorului: medie exponentiala (EMA) cu alfa = 1/8.
// Acumulatorul tine de 8 ori media, adica media cu 3 biti fractionari
// (maxim 1023 * 8 = 8184, incape pe 16 biti):
//   acc = acc - acc / 8 + esantion
// Constanta de timp: ~8 esantioane * MOTOR_VOLTAGE_TIMER (2 ms) = ~16 ms.
#define MOTOR_VOLTAGE_EMA_SHIFT 3U      // alfa = 1 / 2^3
static uint16_t motor_voltage_acc;
static __bit sensor_filter_initialized = 0;
#if SENSOR_MEDIAN_SAMPLES > 1
// Ultimele SENSOR_MEDIAN_SAMPLES - 1 valori de la read_adc() pe fiecare canal
// (cea mai veche prima), pentru mediana in timp din analog_update().
static uint16_t sensor_median_history[SENSOR_COUNT][SENSOR_MEDIAN_SAMPLES - 1];
#endif
static __bit motor_voltage_filter_initialized = 0;

// Microvolti la intrarea divizorului pentru o treapta ADC, calculati la
// compilare din rezistente: 5000 mV / 1023 * (56000 + 10000) / 10000 = 32.258 mV.
#define MOTOR_VOLTAGE_UV_PER_COUNT \
    ((ADC_REFERENCE_MV * ((MOTOR_VOLTAGE_DIVIDER_TOP_OHM + \
                           MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM) / 100UL) * 1000UL) / \
     (ADC_MAX_VALUE * (MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM / 100UL)))

// Rapoarte sus / jos, in miimi (5600 = 5.600):
//  - ideal: cel care da exact MOTOR_VOLTAGE_MV_PER_COUNT la ADC_REFERENCE_MV,
//    (mV pe treapta * 1023 / Vref) - 1, rotunjit;
//  - minim: cel care tine pinul <= Vref la MOTOR_SUPPLY_MAX,
//    (MOTOR_SUPPLY_MAX / Vref) - 1, rotunjit in sus;
//  - real: din rezistentele definite.
#define MOTOR_DIVIDER_IDEAL_X1000 \
    ((MOTOR_VOLTAGE_MV_PER_COUNT * ADC_MAX_VALUE * 1000UL + \
      ADC_REFERENCE_MV / 2UL) / ADC_REFERENCE_MV - 1000UL)
#define MOTOR_DIVIDER_MIN_X1000 \
    ((MOTOR_SUPPLY_MAX * 1000UL + ADC_REFERENCE_MV - 1UL) / \
     ADC_REFERENCE_MV - 1000UL)
#define MOTOR_DIVIDER_ACTUAL_X1000 \
    ((MOTOR_VOLTAGE_DIVIDER_TOP_OHM * 10UL) / \
     (MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM / 100UL))

// Afiseaza la fiecare compilare cele trei rapoarte, ca avertisment de forma
//   'char (*)[ideal][minim][real]'
// (comparatia din sizeof nu se evalueaza: nu ocupa memorie si nu genereaza cod).
typedef char motor_divider_top_over_bottom_x1000_ideal_min_actual
    [MOTOR_DIVIDER_IDEAL_X1000][MOTOR_DIVIDER_MIN_X1000][MOTOR_DIVIDER_ACTUAL_X1000];
typedef char motor_divider_info[sizeof(
    (motor_divider_top_over_bottom_x1000_ideal_min_actual *)0 == (char (*)[1])0)];

// Erori de compilare (vezi avertismentul de mai sus pentru valori):
// divizorul trebuie sa dea MOTOR_VOLTAGE_MV_PER_COUNT +/- 1.5% pe treapta...
typedef char motor_divider_matches_conversion[
    (MOTOR_VOLTAGE_UV_PER_COUNT >= MOTOR_VOLTAGE_MV_PER_COUNT * 985UL &&
     MOTOR_VOLTAGE_UV_PER_COUNT <= MOTOR_VOLTAGE_MV_PER_COUNT * 1015UL) ? 1 : -1];
// ...pinul nu trebuie sa depaseasca Vref la MOTOR_SUPPLY_MAX...
typedef char motor_divider_covers_supply_max[
    (MOTOR_SUPPLY_MAX * (MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM / 100UL) <=
     ADC_REFERENCE_MV * ((MOTOR_VOLTAGE_DIVIDER_TOP_OHM +
                          MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM) / 100UL)) ? 1 : -1];
// ...si capatul de scala in mV trebuie sa incapa pe 16 biti.
typedef char motor_voltage_fits_16_bits[
    ((ADC_MAX_VALUE << MOTOR_VOLTAGE_MV_SHIFT) <= 65535UL) ? 1 : -1];

// Acumulatorul EMA are deja 3 biti fractionari, deci ajunge o deplasare cu
// MOTOR_VOLTAGE_MV_SHIFT - MOTOR_VOLTAGE_EMA_SHIFT (32 mV / 8 = 4 mV pe
// unitate); bitii fractionari nu se pierd.
typedef char motor_ema_shift_fits_conversion[
    (MOTOR_VOLTAGE_MV_SHIFT >= MOTOR_VOLTAGE_EMA_SHIFT &&
     (ADC_MAX_VALUE << MOTOR_VOLTAGE_EMA_SHIFT) <= 65535UL) ? 1 : -1];

// Tensiunea motorului in milivolti din acumulatorul EMA: o simpla deplasare,
// fara inmultire sau impartire (maxim 8184 * 4 = 32736, incape pe 16 biti).
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
    near = distance_lut[index];                     // la ADC mai mic = mai departe
    span = (uint8_t)(near - distance_lut[index + 1U]);
    // frac (5 biti) este primul operand: inmultirea itereaza pe bitii lui
    distance = (uint8_t)(near - (uint8_t)(((uint16_t)frac * span) >> LUT_SHIFT));
    if (distance > DISTANCE_MAX) {
        return DISTANCE_MAX;
    }
    return (distance < DISTANCE_MIN) ? DISTANCE_MIN : distance;
}

void adc_init(void){
    //Set ADC channels as inputs
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA2 = 1;
    TRISAbits.TRISA3 = 1;
    TRISAbits.TRISA5 = 1;
    //Conversion clock FOSC/32: TAD = 32 / 18.432 MHz = 1.74 us (minim 1.6 us)
    ADCON1bits.ADCS2 = 0; 
    ADCON0bits.ADCS = 0b10; 
    ADCON1bits.PCFG = 0b0010; // AN0-4 analog, AN5-7 digital
    ADCON1bits.ADFM = 1; //Right justified
    ADCON0bits.CHS = 0; //Select channel AN0
    ADCON0bits.ADON = 1; //Turn on ADC

    
}

#if ADC_MEDIAN_SAMPLES != 1 && ADC_MEDIAN_SAMPLES != 3 && ADC_MEDIAN_SAMPLES != 5
#error "ADC_MEDIAN_SAMPLES trebuie sa fie 1, 3 sau 5"
#endif
#if SENSOR_MEDIAN_SAMPLES != 1 && SENSOR_MEDIAN_SAMPLES != 3 && SENSOR_MEDIAN_SAMPLES != 5
#error "SENSOR_MEDIAN_SAMPLES trebuie sa fie 1, 3 sau 5"
#endif

// Retele de comparatii pentru mediana (verificate pe toate combinatiile):
// dupa MEDIAN3 valoarea din mijloc este in s[1], dupa MEDIAN5 in s[2].
// Sunt macro-uri, nu functii, ca sa nu consume un nivel de stiva hardware
// (stiva este deja plina in cazul cel mai defavorabil). t = variabila temporara.
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
        return 0; // Invalid channel
    }
    ADCON0bits.CHS = channel; // Select the channel
#if ADC_MEDIAN_SAMPLES == 1
    __delay_us(ADC_ACQUISITION_US); // Wait for acquisition time
    ADCON0bits.GO = 1; // Start the conversions
    while (ADCON0bits.GO); // Wait for the conversion to complete
    return ((uint16_t)(ADRESH & 0x03U) << 8) | ADRESL; // limitat la 10 biti (ADFM = 1, aliniat la dreapta)
#else
    for (i = 0; i < ADC_MEDIAN_SAMPLES; i++) {
        __delay_us(ADC_ACQUISITION_US); // Acelasi timp de achizitie la fiecare
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO);
        s[i] = ((uint16_t)(ADRESH & 0x03U) << 8) | ADRESL; // 10 biti
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
            // Primul esantion umple filtrul, ca sa nu porneasca de la 0 V
            motor_voltage_acc = (uint16_t)(sample << MOTOR_VOLTAGE_EMA_SHIFT);
            motor_voltage_filter_initialized = 1;
        } else {
            // acc / 8 rotunjit (nu trunchiat): filtrul se aseaza simetric,
            // in +/- 4 unitati (+/- 16 mV) in jurul valorii reale.
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
            // Mediana in timp: valoarea noua + ultimele SENSOR_MEDIAN_SAMPLES - 1,
            // la SAMPLE_TIMER ms distanta. Prima trecere umple istoricul.
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
                // Primul esantion umple filtrul, ca sa nu porneasca de la 0
                sensor_filter_acc[channel] = target;
            } else {
                acc = sensor_filter_acc[channel];
                up = (target >= acc);
                diff = up ? (uint16_t)(target - acc) : (uint16_t)(acc - target);
                // Pas = diferenta / 2^k, rotunjit; doua ramuri cu deplasari
                // constante (o deplasare variabila ar fi o bucla)
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
            // Valoarea ADC filtrata, rotunjita la cea mai apropiata treapta
            filtered = (uint16_t)((sensor_filter_acc[channel] +
                (1U << (SENSOR_EMA_SLOW_SHIFT - 1U))) >> SENSOR_EMA_SLOW_SHIFT);
#if SENSOR_REPORT_UNFILTERED
            sensor_values[channel] = sample;     // doar pentru raportul UART
#else
            sensor_values[channel] = filtered;
#endif
            sensor_distances[channel] = adc_to_distance(filtered);
        }
        sensor_filter_initialized = 1;
#endif
        sample_timer = SAMPLE_TIMER;
        updated = 1;
    }
    return updated;
}
