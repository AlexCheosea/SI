
#ifndef ANALOG_H
#define	ANALOG_H

#include <xc.h>
#include <stdint.h>

#define SAMPLE_TIMER 16                // ms intre esantioanele senzorilor de distanta
#define MOTOR_VOLTAGE_TIMER 2
#define MOTOR_SPEED_REPORT_TIMER 150
#define STATUS_REPORT_TIMER 200        // ms intre rapoartele de stare UART (max 255)
#define SENSOR_COUNT 4
// Timp de achizitie dupa schimbarea canalului, in us. Foaia de catalog
// PIC16F877A cere ~19.7 us (sursa 10 k, 50 C); 30 us lasa o marja.
#define ADC_ACQUISITION_US 30
// Mediana in read_adc(): 1 = o singura conversie (fara mediana), 3 sau 5 =
// atatea conversii consecutive pe acelasi canal, se intoarce valoarea din
// mijloc (elimina un varf izolat, respectiv doua). Fiecare conversie are
// propriul timp de achizitie ADC_ACQUISITION_US.
#define ADC_MEDIAN_SAMPLES 3
// Mediana in timp in analog_update(), inainte de filtrul EMA: 1 = fara,
// 3 sau 5 = mediana valorii noi si a ultimelor 2 / 4 citiri ale aceluiasi
// senzor (la SAMPLE_TIMER ms distanta). Elimina citiri izolate gresite;
// intarzie raspunsul cu 1 / 2 esantioane.
#define SENSOR_MEDIAN_SAMPLES 5
// Filtrul EMA adaptiv al senzorilor (dupa mediane): daca valoarea noua se
// abate de la cea filtrata cu mai mult de SENSOR_EMA_THRESHOLD trepte ADC,
// filtrul urmareste repede (alfa = 1 / 2^FAST), altfel netezeste puternic
// (alfa = 1 / 2^SLOW). Zgomotul mic este netezit, schimbarile reale (obstacol
// nou) sunt urmarite aproape imediat.
#define SENSOR_EMA_SLOW_SHIFT 5U       // alfa = 1/32 (maxim 5: calculul pe 16 biti)
#define SENSOR_EMA_FAST_SHIFT 2U       // alfa = 1/4
// Pragul, in trepte ADC (aceeasi unitate ca valoarea din raportul UART
// "S0F <adc> ..."). Se alege ~3 x abaterea standard a citirii nefiltrate
// (vezi SENSOR_REPORT_UNFILTERED), la distanta cea mai zgomotoasa.
// PROVIZORIU: 24 este estimat din prima observatie (salturi de 1-2 cm langa
// 15 cm, fara condensatoare, alimentare USB), nu masurat.
#define SENSOR_EMA_THRESHOLD 24U
// 1 = raportul UART arata valoarea de dupa mediane, inainte de EMA (pentru
// masurarea zgomotului si alegerea pragului). Distantele si comanda motorului
// folosesc in continuare valoarea filtrata. 0 = functionare normala.
#define SENSOR_REPORT_UNFILTERED 0
// Distantele sunt tinute in sferturi de centimetru, pe 8 biti (60 cm = 240).
// Pragurile se scriu in centimetri prin CM(); cel mai mare trebuie sa ramana
// <= 63 cm ca sa incapa intr-un octet.
#define DISTANCE_UNITS_PER_CM 4U
#define CM(x) ((uint8_t)((x) * DISTANCE_UNITS_PER_CM))
#define DISTANCE_MIN        CM(10)     // sub aceasta senzorul nu mai masoara corect
#define DISTANCE_MAX        CM(60)     // plafon: orice e mai departe se citeste 60 cm
#define DISTANCE_FAR        CM(50)     // de aici incepe incetinirea
#define DISTANCE_STOP       CM(15)     // de aici motorul sta
#define DISTANCE_TOLERANCE  CM(3)      // histerezis la iesirea din STOP / SLOW
#define ADC_CHANNEL_MOTOR_VOLTAGE 4
#define ADC_MAX_VALUE 1023UL
#define ADC_REFERENCE_MV 5000UL
// Divizorul tensiunii motorului: 56 k sus, 10 k jos (sus / jos = 5.6). Cu ADC
// pe 10 biti si referinta de 5 V, o treapta ADC = 32.26 mV la intrare, adica
// practic 32 mV: conversia in milivolti este o simpla deplasare (adc << 5).
// Capat de scala: ~32.7 V. La compilare se afiseaza (avertisment) raportul
// ideal sus / jos si se verifica divizorul (vezi analog.c).
#define MOTOR_SUPPLY_MAX 24000UL       // mV: cea mai mare tensiune (cu varfuri)
                                       // de la intrarea divizorului
#define MOTOR_VOLTAGE_DIVIDER_TOP_OHM 56000UL
#define MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM 10000UL
#define MOTOR_VOLTAGE_MV_SHIFT 5U       // milivolti = adc << 5
#define MOTOR_VOLTAGE_MV_PER_COUNT (1UL << MOTOR_VOLTAGE_MV_SHIFT) // 32 mV
#define MOTOR_VOLTAGE_MIN 5000UL       // mV
#define MOTOR_VOLTAGE_MAX 15000UL      // mV
#define MOTOR_VOLTAGE_TOLERANCE 500UL  // mV
#define USE_DUMMY_SENSOR_VALUES 0
#define DUMMY_SENSOR_ADC_VALUE 0U
#define USE_DUMMY_MOTOR_VOLTAGE 1
#define DUMMY_MOTOR_VOLTAGE_ADC 300U    // 300 * 32 mV = 9.6 V

extern volatile uint8_t sample_timer;
extern volatile uint8_t motor_voltage_timer;
extern volatile uint8_t motor_speed_report_timer;
extern volatile uint8_t status_report_timer;
extern volatile uint16_t sensor_values[SENSOR_COUNT];
extern volatile uint8_t sensor_distances[SENSOR_COUNT];
extern volatile uint16_t motor_voltage_millivolts;

void adc_init(void);
uint16_t read_adc(uint8_t channel);
uint8_t analog_update(void);



#endif	/* ANALOG_H */
