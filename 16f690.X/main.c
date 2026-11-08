/*
 * File:   main.c
 * Author: Alex
 *
 * Sclavul: primeste comenzi de viteza/sens prin SPI (in ISR) si ruleaza
 * rampa de viteza in bucla principala; pasii motorului se fac in ISR (CCP1).
 */

#include <xc.h>
#include <stdint.h>
#include "init.h"
#include "main.h"

void main(void) {
    system_init();

    while (1) {
        // Resincronizare SPI dupa ~4 ms fara activitate pe magistrala
        di();
        spi_resync();
        ei();

        // Copiem atomic comanda primita in ISR-ul SPI, apoi o aplicam ca
        // tinta pentru rampa (motor_set_target_speed isi face singur
        // sectiunea critica).
        if (spi_motor_updated) {
            di();
            uint16_t speed = spi_motor_speed;
            uint8_t dir = spi_motor_dir;
            spi_motor_updated = 0;
            ei();
            motor_set_target_speed(speed, dir);
        }

        motor_ramp_task(); // pasul de rampa (la 10 ms), in afara ISR-ului
    }
}
