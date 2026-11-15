
#ifndef ANALOG_H
#define	ANALOG_H

#include <xc.h>
#include <stdint.h>

// Perioadele, in tick-uri de 1 ms (max 255)
#define SAMPLE_TIMER 16                // esantionarea senzorilor de distanta
#define MOTOR_VOLTAGE_TIMER 2
#define MOTOR_SPEED_REPORT_TIMER 150
#define STATUS_REPORT_TIMER 200        // raportul de stare UART
#define POSITION_REPORT_TIMER 20       // citirea pozitiei de la sclav (2 pachete SPI)
#define SENSOR_COUNT 4
// Timp de achizitie ADC, in us. Minimul din foaia de catalog:
// TACQ = 2 us + 120 pF x (8 kOhm + Rsursa) x ln(2047) + 0.05 us / C peste 25 C
#define ADC_ACQUISITION_US 30
// Mediana in read_adc(): 1 = o conversie, 3 / 5 = mediana a atatea conversii consecutive
#define ADC_MEDIAN_SAMPLES 3
// Mediana in timp (analog_update): 1 = fara, 3 / 5 = valoarea noua + ultimele
// 2 / 4. Elimina citiri izolate gresite; intarzie cu 1 / 2 esantioane.
#define SENSOR_MEDIAN_SAMPLES 5
// EMA adaptiv dupa mediane: abatere > prag -> alfa = 1 / 2^FAST, altfel
// 1 / 2^SLOW. Constanta de timp ~ 2^shift x SAMPLE_TIMER.
// SLOW maxim 5: 1023 x 2^SLOW trebuie sa incapa pe 16 biti (verificat in analog.c).
#define SENSOR_EMA_SLOW_SHIFT 5U
#define SENSOR_EMA_FAST_SHIFT 2U
// Pragul, in trepte ADC: ~3 x abaterea standard a valorii de dupa mediane,
// la distanta cea mai zgomotoasa. Provizoriu (estimat, nemasurat).
#define SENSOR_EMA_THRESHOLD 24U
// Distantele: sferturi de cm pe 8 biti, deci CM() maxim 63 cm
#define DISTANCE_UNITS_PER_CM 4U
#define CM(x) ((uint8_t)((x) * DISTANCE_UNITS_PER_CM))
#define DISTANCE_MIN        CM(10)     // sub aceasta senzorul nu mai masoara corect
#define DISTANCE_MAX        CM(60)     // plafon: mai departe se citeste DISTANCE_MAX
#define DISTANCE_FAR        CM(50)     // de aici incepe incetinirea
#define DISTANCE_STOP       CM(15)     // de aici motorul sta
#define DISTANCE_TOLERANCE  CM(3)      // histerezis la iesirea din STOP / SLOW
#define ADC_CHANNEL_MOTOR_VOLTAGE 4
#define ADC_MAX_VALUE 1023UL
#define ADC_REFERENCE_MV 5000UL
// Divizorul tensiunii motorului. Treapta ADC la intrarea lui:
//   mV pe treapta = Vref / 1023 x (sus + jos) / jos
// trebuie sa fie 2^MOTOR_VOLTAGE_MV_SHIFT +/- 1.5% (conversia este o deplasare);
// verificat la compilare, care afiseaza si raportul ideal sus / jos (analog.c).
#define MOTOR_SUPPLY_MAX 24000UL       // mV: tensiunea maxima (cu varfuri) la intrarea divizorului
#define MOTOR_VOLTAGE_DIVIDER_TOP_OHM 56000UL
#define MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM 10000UL
#define MOTOR_VOLTAGE_MV_SHIFT 5U       // milivolti = adc << shift
#define MOTOR_VOLTAGE_MV_PER_COUNT (1UL << MOTOR_VOLTAGE_MV_SHIFT)
#define MOTOR_VOLTAGE_MIN 5000UL       // mV
#define MOTOR_VOLTAGE_MAX 15000UL      // mV
#define MOTOR_VOLTAGE_TOLERANCE 500UL  // mV
#define USE_DUMMY_SENSOR_VALUES 1
#define DUMMY_SENSOR_ADC_VALUE 0U
#define USE_DUMMY_MOTOR_VOLTAGE 1
#define DUMMY_MOTOR_VOLTAGE_ADC 300U    // valoarea ADC simulata

extern volatile uint8_t sample_timer;
extern volatile uint8_t motor_voltage_timer;
extern volatile uint8_t motor_speed_report_timer;
extern volatile uint8_t status_report_timer;
extern volatile uint8_t position_report_timer;
extern volatile uint16_t sensor_values[SENSOR_COUNT];
extern volatile uint8_t sensor_distances[SENSOR_COUNT];
extern volatile uint16_t motor_voltage_millivolts;

void adc_init(void);
uint16_t read_adc(uint8_t channel);
uint8_t analog_update(void);



#endif	/* ANALOG_H */
