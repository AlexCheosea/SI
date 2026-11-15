#define TMR0_RELOAD_VAL 112 // TMR0 numara de aici pana la overflow: tick = (256 - reload) x prescaler x 4 / Fosc
#define TMR0_SET        0b10000100
#define _XTAL_FREQ 18432000UL
#include <xc.h>
#include "init.h"
// Date folosite de spi_asm.s, in Bancul 3 (spi.c nu face parte din proiect)
volatile __bank(3) unsigned char spi_buf[SPI_BUF_SIZE];
volatile __bank(3) uint8_t spi_counter;
volatile __bank(3) uint8_t spi_delay_counter;
void system_init(void) {
    __delay_ms(50);
    buzzer_init();
    hmi_init();
    spi_init();
    uart_init();
    adc_init();

    OPTION_REG = TMR0_SET;  // OPTION_REG: T0CS = sursa Timer0, PSA = prescaler la Timer0 / WDT, PS = prescaler
    TMR0 = TMR0_RELOAD_VAL;
    INTCONbits.T0IE = 1;
    INTCONbits.PEIE = 1;    // intreruperile perifericelor (UART)
    INTCONbits.GIE = 1;
    hmi_print_string(" ");
    return;
}

// Starea buzzerului de parcare, folosita doar in intrerupere
static __bit buzzer_sounding;               // 1 = buzzerul suna acum
static uint8_t buzzer_prescaler = 10;       // tick-uri pe pasul buzzerului (10 ms)
static uint8_t buzzer_countdown = 0;        // timp ramas din faza curenta (x10 ms)

void __interrupt() isr(void){

    // Timer0: tick-ul de 1 ms
    if (T0IE && T0IF) {
        TMR0 = TMR0_RELOAD_VAL;
        hmi_update();

        if(buzzer_ms_ramase > 0){
            buzzer_ms_ramase--;
            if(buzzer_ms_ramase == 0){
                buzzer_off();
            }
        }
        // Buzzer de parcare, la fiecare pas: bip de buzzer_on_time, apoi
        // pauza de buzzer_off_time
        if(--buzzer_prescaler == 0){
            buzzer_prescaler = 10;
            if(buzzer_countdown > 0){
                buzzer_countdown--;
            }
            // Daca pauza ceruta s-a scurtat, nu mai asteptam pauza veche
            if(!buzzer_sounding && buzzer_countdown > buzzer_off_time){
                buzzer_countdown = buzzer_off_time;
            }
            if(buzzer_countdown == 0){
                if(buzzer_on_time == 0){                // oprit
                    if(buzzer_sounding){
                        buzzer_off();
                        buzzer_sounding = 0;
                    }
                } else if(!buzzer_sounding){            // incepe un bip
                    buzzer_on();
                    buzzer_sounding = 1;
                    buzzer_countdown = buzzer_on_time;
                } else if(buzzer_off_time != 0){        // bip terminat: pauza
                    buzzer_off();
                    buzzer_sounding = 0;
                    buzzer_countdown = buzzer_off_time;
                }                                       // altfel: sunet continuu
            }
        }
        if(sample_timer > 0){
            sample_timer--;
        }
        if(motor_voltage_timer > 0){
            motor_voltage_timer--;
        }
        if(motor_speed_report_timer > 0){
            motor_speed_report_timer--;
        }
        if(status_report_timer > 0){
            status_report_timer--;
        }
        if(position_report_timer > 0){
            position_report_timer--;
        }
        T0IF = 0;
    }
    // UART: octet primit
    if (RCIE && RCIF) {
        uart_rx_isr();
    }

    // UART: registrul de transmisie este liber
    if (TXIE && TXIF) {
        uart_tx_isr();
    }
}
