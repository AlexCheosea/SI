
#ifndef ANALOG_H
#define	ANALOG_H

#include <xc.h>
#include <stdint.h>

#define SAMPLE_TIMER 8
#define MOTOR_VOLTAGE_TIMER 2
#define SENSOR_COUNT 4
#define ADC_CHANNEL_MOTOR_VOLTAGE 4

extern volatile uint8_t sample_timer;
extern volatile uint8_t motor_voltage_timer;
extern volatile uint16_t sensor_values[SENSOR_COUNT];
extern volatile uint16_t motor_voltage;

void adc_init(void);
uint16_t read_adc(uint8_t channel);
void analog_update(void);



#endif	/* ANALOG_H */
