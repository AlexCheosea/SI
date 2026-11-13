/*
 * File:   main.c
 * Author: Alex
 *
 * Sclav SPI: primeste viteza / sensul motorului si ruleaza rampa de viteza.
 * Pasii motorului se fac in ISR (CCP1).
 */

#include <xc.h>
#include <stdint.h>
#include "init.h"
#include "main.h"

void main(void) {
    system_init();

    while (1) {
        di();
        spi_resync();
        ei();

        // Copie atomica a comenzii primite in ISR-ul SPI
        if (spi_motor_updated) {
            di();
            uint16_t speed = spi_motor_speed;
            uint8_t dir = spi_motor_dir;
            spi_motor_updated = 0;
            ei();
            motor_set_target_speed(speed, dir);
        }

        motor_ramp_task();
    }
}
