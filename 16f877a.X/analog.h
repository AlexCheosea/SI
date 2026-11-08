
#ifndef ANALOG_H
#define	ANALOG_H

#include <xc.h>
#include <stdint.h>

#define SAMPLE_TIMER 8
#define MOTOR_VOLTAGE_TIMER 2
#define MOTOR_SPEED_REPORT_TIMER 50
#define STATUS_REPORT_TIMER 100        // ms intre rapoartele de stare UART (max 255)
#define SENSOR_COUNT 4
#define ADC_CHANNEL_MOTOR_VOLTAGE 4
#define ADC_MAX_VALUE 1023UL
#define ADC_REFERENCE_MV 5000UL
#define MOTOR_VOLTAGE_DIVIDER_TOP_OHM 4700UL
#define MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM 1200UL
#define MOTOR_VOLTAGE_MIN 5UL
#define MOTOR_VOLTAGE_MAX 15UL
#define USE_DUMMY_SENSOR_VALUES 1
#define DUMMY_SENSOR_ADC_VALUE 0U
#define USE_DUMMY_MOTOR_VOLTAGE 1
#define DUMMY_MOTOR_VOLTAGE_ADC 400U

#define MOTOR_VOLTAGE_TO_ADC(voltage) \
    ((uint16_t)(((voltage) * 1000UL * ADC_MAX_VALUE * \
    MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM) / \
    (ADC_REFERENCE_MV * (MOTOR_VOLTAGE_DIVIDER_TOP_OHM + \
    MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM))))

extern volatile uint8_t sample_timer;
extern volatile uint8_t motor_voltage_timer;
extern volatile uint8_t motor_speed_report_timer;
extern volatile uint8_t status_report_timer;
extern volatile uint16_t sensor_values[SENSOR_COUNT];
extern volatile uint16_t motor_voltage;

void adc_init(void);
uint16_t read_adc(uint8_t channel);
void analog_update(void);



#endif	/* ANALOG_H */
