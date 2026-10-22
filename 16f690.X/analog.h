#ifndef ANALOG_H
#define	ANALOG_H

#include <stdint.h>
//Frecventa procesorului pentru macrourile de delay
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000 
#endif

// Definirea canalelor ADC pentru PIC16F690
#define CH_TEMP  0  // AN0 (RA0) - LM35D
#define CH_LGT   1  // AN1 (RA1) - LDR PGM5506
#define CH_POT   2  // AN2 (RA2) - Poten?iometru

#define ACQ_DELAY 30 //Acquisition delay in us when switching ADC channels
#define SAMPLE_DELAY 500//Spacing delay between samples in us
// --- FUNCTION PROTOTYPES ---

/**
 * @brief Initializes RA0, RA1, RA2 as analog inputs and configures the ADC module.
 */
void adc_init(void);

/**
 * @brief Low-level hardware read for a specific ADC channel.
 * @param channel Analog channel (0-11)
 * @return 10-bit raw ADC value (0 - 1023)
 */
uint16_t adc_read(uint8_t channel);

/**
 * @brief Reads the LM35D sensor and returns temperature in xx.x format as an integer.
 * @return Temperature multiplied by 10 (e.g., 254 for 25.4°C)
 */
int16_t read_temp(void);

/**
 * @brief Reads the LDR light sensor and maps the raw value to degrees (0 - 180°).
 * @return Light level converted to degrees (0 - 180)
 */
uint8_t read_lgt(void);

/**
 * @brief Reads the potentiometer and returns the voltage.
 * @return Voltage on the wiper pin (0mV - 5000mV)
 */
uint16_t read_pot(void);

#endif	/* ANALOG_H */