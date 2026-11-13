#include "spi.h"
#include "motor.h"
volatile PachetSPI_t rx_packet;
volatile unsigned char spi_index = 0;



volatile uint16_t spi_motor_speed = 0;
__bit spi_motor_dir;
__bit spi_motor_updated = 0;
void spi_init(void) {
    TRIS_SCK = 1;
    TRIS_SDI = 1;
    TRIS_SDO = 0;

    // ANSELH: 1 = pin analogic, 0 = digital (necesar pentru SDI / SDO)
    ANSELHbits.ANS9 = 0;
    ANSELHbits.ANS10 = 0;
    SSPEN = 0;
    SSPSTAT = 0b01000000; // SSPSTAT: SMP = momentul esantionarii, CKE = frontul de ceas al transmisiei

    // SSPCON: SSPEN = pornit, CKP = nivelul ceasului in repaus, SSPM = mod (master / slave, cu / fara SS)
    SSPCON = 0b00100101;

    SSPBUF = SPI_ACK; // primul octet trimis

    PIR1bits.SSPIF = 0;
    PIE1bits.SSPIE = 1;
    INTCONbits.PEIE = 1;
}
// Fara activitate pe magistrala (Timer0 a depasit): reporneste modulul SPI
void spi_resync(void) {
    if(INTCONbits.T0IF) {
        INTCONbits.T0IF = 0;

        spi_index = 0;
        SSPEN = 0;
        uint8_t dummy = SSPBUF;
        SSPCONbits.WCOL  = 0;
        SSPCONbits.SSPOV = 0;
        SSPEN = 1;
        SSPBUF = SPI_ACK;
        TMR0 = 0;
    }
}

// Octetul high al raspunsului, salvat odata cu cel low ca valoarea sa fie consistenta
static uint8_t spi_reply_hi;

// Apelata din ISR cand SSPIF este setat
void spi_isr(void) {
    uint8_t idx = spi_index;
    uint8_t rx_byte = SSPBUF;   // citirea goleste BF

    // SSPBUF se reincarca primul: master-ul asteapta doar pauza dintre octeti.
    // Raspunsul este mereu viteza curenta.
    if (idx == 0) {
        SSPBUF = (uint8_t)motor_current_sps;
        spi_reply_hi = (uint8_t)(motor_current_sps >> 8);
    } else if (idx == 1) {
        SSPBUF = spi_reply_hi;
    } else {
        SSPBUF = SPI_ACK;
    }
    PIR1bits.SSPIF = 0;
    TMR0 = 0;
    INTCONbits.T0IF = 0;

    rx_packet.raw[idx] = rx_byte;
    idx++;
    if (idx >= SPI_BUF_SIZE) {
        // Pachet complet
        idx = 0;
        if (rx_packet.cmd == CMD_SET_MOTOR_SPEED) {
            if ((uint16_t)rx_packet.val !=
                motor_target_steps_per_second) {
                spi_motor_speed = (uint16_t)rx_packet.val;
                spi_motor_updated = 1;
            }
        }
        else if (rx_packet.cmd == CMD_SET_MOTOR_DIR) {
            spi_motor_dir = (uint8_t)rx_packet.val;
            spi_motor_updated = 1;
        }
    }
    spi_index = idx;
}
