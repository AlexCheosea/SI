
#define _XTAL_FREQ 18432000UL
#include "analog.h"

#define DISTANCE_FIR_LENGTH 4
#define DISTANCE_TABLE_POINTS 6U

// Puncte de calibrare (ADC -> distanta in sferturi de cm), de la aproape la
// departe. Ultimul punct este plafonul DISTANCE_MAX.
static const struct {
    uint16_t adc;
    uint8_t distance;
} distance_table[DISTANCE_TABLE_POINTS] = {
    {737,  DISTANCE_MIN},
    {517,  CM(15)},
    {408,  CM(20)},
    {298,  CM(30)},
    {243,  CM(40)},
    {189,  DISTANCE_MAX},
};

volatile uint8_t sample_timer = 0;
volatile uint8_t motor_voltage_timer = 0;
volatile uint8_t motor_speed_report_timer = 0;
volatile uint8_t status_report_timer = 0;
volatile uint16_t sensor_values[SENSOR_COUNT];
volatile uint8_t sensor_distances[SENSOR_COUNT];
volatile uint16_t motor_voltage_millivolts;

static uint16_t sensor_filter_samples[SENSOR_COUNT][DISTANCE_FIR_LENGTH];
static uint16_t sensor_filter_sums[SENSOR_COUNT];
static uint16_t motor_voltage_adc;
static uint8_t sensor_filter_index = 0;
static __bit sensor_filter_initialized = 0;
static __bit motor_voltage_filter_initialized = 0;

static uint16_t adc_to_motor_voltage_mv(uint16_t adc) {
    return (uint16_t)(((uint32_t)adc *
        (ADC_REFERENCE_MV * ((MOTOR_VOLTAGE_DIVIDER_TOP_OHM +
                              MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM) / 100UL))) /
        (ADC_MAX_VALUE * (MOTOR_VOLTAGE_DIVIDER_BOTTOM_OHM / 100UL)));
}

// Intoarce distanta in sferturi de cm (DISTANCE_MIN .. DISTANCE_MAX),
// interpolata liniar intre punctele din tabel.
static uint8_t adc_to_distance(uint16_t adc) {
    uint8_t i;

    if (adc >= distance_table[0].adc) {
        return DISTANCE_MIN;
    }
    for (i = 1; i < DISTANCE_TABLE_POINTS; i++) {
        if (adc >= distance_table[i].adc) {
            uint16_t adc_span = distance_table[i - 1U].adc -
                                distance_table[i].adc;
            uint16_t distance_span = distance_table[i].distance -
                                     distance_table[i - 1U].distance;
            uint16_t offset = distance_table[i - 1U].adc - adc;
            return (uint8_t)(distance_table[i - 1U].distance +
                             (distance_span * offset) / adc_span);
        }
    }
    return DISTANCE_MAX;
}

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

uint8_t analog_update(void) {
    uint8_t channel;
    uint16_t sample;
    uint8_t updated = 0;

    if (motor_voltage_timer == 0) {
#if USE_DUMMY_MOTOR_VOLTAGE
        motor_voltage_millivolts =
            adc_to_motor_voltage_mv(DUMMY_MOTOR_VOLTAGE_ADC);
#else
        sample = read_adc(ADC_CHANNEL_MOTOR_VOLTAGE);
        if (motor_voltage_filter_initialized == 0) {
            motor_voltage_adc = sample;
            motor_voltage_filter_initialized = 1;
        } else {
            motor_voltage_adc = (motor_voltage_adc + sample) >> 1;
        }
        motor_voltage_millivolts = adc_to_motor_voltage_mv(motor_voltage_adc);
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
            sensor_distances[channel] = adc_to_distance(sensor_values[channel]);
        }
        sensor_filter_initialized = 1;
        sensor_filter_index++;
        if (sensor_filter_index >= DISTANCE_FIR_LENGTH) {
            sensor_filter_index = 0;
        }
#endif
        sample_timer = SAMPLE_TIMER;
        updated = 1;
    }
    return updated;
}
