/*
 * File:   main.c
 * Author: Alex
 *
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
        //__delay_ms(1);
        if(spi_motor_updated){
            di(); //dezactivam intreruperile
            uint16_t speed = spi_motor_speed; //copiem valorile 
            uint8_t dir = spi_motor_dir;
            spi_motor_updated = 0;
            ei(); //PS. nu puteam dezactiva intreruperile direct pentru
            //motor_set_speed deoarece functia dureaza prea mult
            motor_set_target_speed(speed, dir);
        }
    }
    return;
}