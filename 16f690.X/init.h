/* 
 * File:   init.h
 * Author: Cheosea Alexandru Gabriel
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef INIT_H
#define	INIT_H
#include "analog.h"
#include "spi.h"
#include "motor.h"
#include <xc.h> // include processor files - each processor file is guarded.  

void system_init(void);



#endif	//INIT_H

