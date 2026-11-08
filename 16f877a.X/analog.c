
#define _XTAL_FREQ 18432000UL
#include "analog.h"

#define DISTANCE_FIR_LENGTH 4

volatile uint8_t sample_timer = 0;
volatile uint8_t motor_voltage_timer = 0;
volatile uint8_t motor_speed_report_timer = 0;
volatile uint8_t status_report_timer = 0;
volatile uint16_t sensor_values[SENSOR_COUNT];
volatile uint16_t motor_voltage;

static uint16_t sensor_filter_samples[SENSOR_COUNT][DISTANCE_FIR_LENGTH];
static uint16_t sensor_filter_sums[SENSOR_COUNT];
static uint8_t sensor_filter_index = 0;
static __bit sensor_filter_initialized = 0;
static __bit motor_voltage_filter_initialized = 0;

void adc_init(void){
    //Set ADC channels as inputs
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA2 = 1;
    TRISAbits.TRISA3 = 1;
    TRISAbits.TRISA5 = 1;
    //Conversion clock FOSC/64
    ADCON1bits.ADCS2 = 1; 
    ADCON0bits.ADCS = 0b10; 
    ADCON1bits.PCFG = 0b0010; // AN0-4 analog, AN5-7 digital
    ADCON1bits.ADFM = 1; //Right justified
    ADCON0bits.CHS = 0; //Select channel AN0
    ADCON0bits.ADON = 1; //Turn on ADC

    
}

uint16_t read_adc(uint8_t channel) {
    if (channel > 4) {
        return 0; // Invalid channel
    }
    ADCON0bits.CHS = channel; // Select the channel
    __delay_us(30); // Wait for acquisition time
    ADCON0bits.GO = 1; // Start the conversions
    while (ADCON0bits.GO); // Wait for the conversion to complete
    return ((uint16_t)ADRESH << 8) | ADRESL;
}

void analog_update(void) {
    uint8_t channel;
    uint16_t sample;

    if (motor_voltage_timer == 0) {
#if USE_DUMMY_MOTOR_VOLTAGE
        motor_voltage = DUMMY_MOTOR_VOLTAGE_ADC;
#else
        sample = read_adc(ADC_CHANNEL_MOTOR_VOLTAGE);
        if (motor_voltage_filter_initialized == 0) {
            motor_voltage = sample;
            motor_voltage_filter_initialized = 1;
        } else {
            motor_voltage = (motor_voltage + sample) >> 1;
        }
#endif
        motor_voltage_timer = MOTOR_VOLTAGE_TIMER;
    }

    if (sample_timer == 0) {
#if USE_DUMMY_SENSOR_VALUES
        for (channel = 0; channel < SENSOR_COUNT; channel++) {
            sensor_values[channel] = DUMMY_SENSOR_ADC_VALUE;
        }
#else
        for (channel = 0; channel < SENSOR_COUNT; channel++) {
            sample = read_adc(channel);
            if (sensor_filter_initialized == 0) {
                sensor_filter_sums[channel] = sample * DISTANCE_FIR_LENGTH;
                sensor_filter_samples[channel][0] = sample;
                sensor_filter_samples[channel][1] = sample;
                sensor_filter_samples[channel][2] = sample;
                sensor_filter_samples[channel][3] = sample;
            } else {
                sensor_filter_sums[channel] -=
                    sensor_filter_samples[channel][sensor_filter_index];
                sensor_filter_samples[channel][sensor_filter_index] = sample;
                sensor_filter_sums[channel] += sample;
            }
            sensor_values[channel] =
                sensor_filter_sums[channel] / DISTANCE_FIR_LENGTH;
        }
        sensor_filter_initialized = 1;
        sensor_filter_index++;
        if (sensor_filter_index >= DISTANCE_FIR_LENGTH) {
            sensor_filter_index = 0;
        }
#endif
        sample_timer = SAMPLE_TIMER;
    }
}
