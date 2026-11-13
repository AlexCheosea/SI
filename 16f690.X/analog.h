#ifndef ANALOG_H
#define	ANALOG_H

#include <stdint.h>
// Frecventa procesorului, pentru macro-urile de delay
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000
#endif

// Seteaza RA0-RA2 ca intrari (ADC-ul nu este pornit)
void adc_init(void);





#endif	/* ANALOG_H */
