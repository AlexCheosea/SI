
#ifndef ANALOG_H
#define	ANALOG_H

#include <xc.h>
#include <stdint.h>

#define SAMPLE_TIMER 8
#define MOTOR_VOLTAGE_TIMER 2
#define MOTOR_SPEED_REPORT_TIMER 150
#define STATUS_REPORT_TIMER 200        // ms intre rapoartele de stare UART (max 255)
#define SENSOR_COUNT 4
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
#define MOTOR_VOLTAGE_DIVIDER_TOP_OHM 4700UL
#define MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM 1200UL
#define MOTOR_VOLTAGE_MIN 5000UL       // mV
#define MOTOR_VOLTAGE_MAX 15000UL      // mV
#define MOTOR_VOLTAGE_TOLERANCE 500UL  // mV
#define USE_DUMMY_SENSOR_VALUES 1
#define DUMMY_SENSOR_ADC_VALUE 0U
#define USE_DUMMY_MOTOR_VOLTAGE 1
#define DUMMY_MOTOR_VOLTAGE_ADC 400U

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
