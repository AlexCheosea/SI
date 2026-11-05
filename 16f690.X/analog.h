#ifndef ANALOG_H
#define	ANALOG_H

#include <stdint.h>
//Frecventa procesorului pentru macrourile de delay
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000 
#endif

// --- FUNCTION PROTOTYPES ---

/**
 * @brief Initializes RA0, RA1, RA2 as analog inputs and configures the ADC module.
 */
void adc_init(void);





#endif	/* ANALOG_H */