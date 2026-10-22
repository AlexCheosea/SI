/*
 * File:   main.c
 * Author: Alex
 *
 * Created on June 13, 2026, 6:24 AM
 */


#include <xc.h>
#include "init.h"
#include "main.h"

#include <xc.h>
#include <stdint.h>
//void delay_10ms(uint8_t ticks){
//    for(uint8_t i = 0; i < ticks; i++){
//        __delay_ms(10);
//    }
//}
//#define boot_delay delay_10ms
void main(void) {
    system_init();
    //motor_startup_animation();
    //rez.fields.humidity = 0;
    //rez.fields.pressure = 0;
    //rez.fields.temperature = 0;
    uint16_t aux; 
    while(1){
        di();
        spi_resync();
        ei();        
        aux = read_temp(); //copiem valoarea cu intreruperile dezactivate
        di();
        temp = aux; //pentru a ne asigura ca nu se trimite o valoare
        //copiata pe jumatate 
        ei();
        //__delay_ms(1);
        aux = read_lgt();
        di();
        light = aux;
        ei();
        //__delay_ms(1);
        aux = read_pot();
        di();
        pot = aux;
        ei();
        //__delay_ms(1);
        if(spi_motor_updated){
            di(); //dezactivam intreruperile
            uint16_t speed = spi_motor_speed; //copiem valorile 
            uint8_t dir = spi_motor_dir;
            ei(); //PS. nu puteam dezactiva intreruperile direct pentru
            //motor_set_speed deoarece functia dureaza prea mult
            motor_set_speed(spi_motor_speed, spi_motor_dir);
            spi_motor_updated = 0;
        }
        if(raw_done){
            // 1. Create local working copies of raw data and output holders
            int32_t local_raw_temp     = raw.fields.temperature;
            int32_t local_raw_press    = raw.fields.pressure;
            int32_t local_raw_humidity = raw.fields.humidity;

            int16_t  out_temperature;
            uint16_t out_pressure;
            uint16_t  out_humidity;

            // 2. Intermediate registers for flat math execution (strictly 32-bit)
            int32_t  t_fine;
            int32_t  t_var1, t_var2, t_x;
            int32_t  h_x1, h_a, h_b, h_c;

            // =========================================================================
            // 3. TEMPERATURE MATH
            // =========================================================================
            t_x    = (local_raw_temp >> 4) - (int32_t)c.fields.T1;   // computed once, reused below

            t_var1 = (((local_raw_temp >> 3) - ((int32_t)c.fields.T1 << 1)) * (int32_t)c.fields.T2) >> 11;
            t_var2 = (((t_x * t_x) >> 12) * (int32_t)c.fields.T3) >> 14;

            t_fine = t_var1 + t_var2;
            out_temperature = (int16_t)((t_fine * 5L + 128L) >> 8);

            // =========================================================================
            // PRESSURE Math (Official Bosch 32-bit version)
            // out_pressure = pressure in 0.1 hPa (10135 = 1013.5 hPa)
            // =========================================================================

        
            int32_t var1, var2;
            uint32_t p;

            var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;

            var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) *
                   ((int32_t)c.fields.P6);

            var2 = var2 + ((var1 * ((int32_t)c.fields.P5)) << 1);

            var2 = (var2 >> 2) +
                   (((int32_t)c.fields.P4) << 16);

            var1 = (((c.fields.P3 *
                      (((var1 >> 2) * (var1 >> 2)) >> 13))
                      >> 3) +
                    ((((int32_t)c.fields.P2) * var1) >> 1))
                    >> 18;

            var1 = ((((32768 + var1)) *
                     ((int32_t)c.fields.P1)) >> 15);

            if (var1 == 0)
            {
                out_pressure = 0;      /* avoid division by zero */
            }
            else
            {
                p = (((uint32_t)(((int32_t)1048576)
                    - local_raw_press) - (uint32_t)(var2 >> 12))) * 3125U;

                if (p < 0x80000000U)
                {
                    p = (p << 1) / ((uint32_t)var1);
                }
                else
                {
                    p = (p / (uint32_t)var1) * 2U;
                }

                var1 = (((int32_t)c.fields.P9) *
                        ((int32_t)(((p >> 3) * (p >> 3)) >> 13)))
                        >> 12;

                var2 = (((int32_t)(p >> 2)) *
                        ((int32_t)c.fields.P8))
                        >> 13;

                p = (uint32_t)((int32_t)p +
                    ((var1 + var2 + c.fields.P7) >> 4));

                /* p is already in Pa */
                out_pressure = (uint16_t)(p / 10U);
            }
        

            // =========================================================================
            // 5. HUMIDITY MATH
            // =========================================================================
            h_x1 = t_fine - 76800L;

            h_a = ((int32_t)local_raw_humidity << 14)
                - ((int32_t)c.fields.H4 << 20)
                - ((int32_t)c.fields.H5 * h_x1);
            h_a = (h_a + 16384L) >> 15;

            h_b = (h_x1 * (int32_t)c.fields.H6) >> 10;
            h_c = (h_x1 * (int32_t)c.fields.H3) >> 11;
            h_b = (h_b * (h_c + 32768L)) >> 10;
            h_b = ((h_b + 2097152L) * (int32_t)c.fields.H2 + 8192L) >> 14;

            h_x1 = h_a * h_b;

            h_x1 = h_x1 - ((((h_x1 >> 15) * h_x1) >> 15) * (int32_t)c.fields.H1 >> 4);

            if (h_x1 < 0) {
                h_x1 = 0;
            } else if (h_x1 > 419430400L) {
                h_x1 = 419430400L;
            }

            // Bosch's h_v_x1_u32r>>12 = Q22.10 %RH; >>22 total gives plain %RH (0-100), fits uint8_t.
            // Modified to give xx.x% *10
            out_humidity = (uint16_t)((h_x1*5) >> 21);

            // =========================================================================
            // 6. ATOMIC OUT-PUSH
            // =========================================================================
            di();
            rez.fields.temperature = out_temperature;
            rez.fields.pressure    = out_pressure;
            rez.fields.humidity    = out_humidity;
            raw_done = 0;
            ei();
            rez_done = 1;
        }
        else{
            __delay_us(500); //small delay when not doing math
        }
    }
    return;
}