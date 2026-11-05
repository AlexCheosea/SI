/* 
 * File:   main.h
 * Author: Cheosea Alexandru Gabriel
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MAIN_H
#define	MAIN_H

#include <xc.h> // include processor files - each processor file is guarded.  

extern volatile uint16_t spi_motor_speed;
extern __bit spi_motor_dir;
extern __bit spi_motor_updated;


#endif //MAIN_H

