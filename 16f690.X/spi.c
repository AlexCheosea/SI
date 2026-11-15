#include "spi.h"
#include "motor.h"
volatile PachetSPI_t rx_packet;
volatile unsigned char spi_index = 0;



volatile uint16_t spi_motor_speed = 0;
__bit spi_motor_dir;
__bit spi_motor_updated = 0;
volatile uint16_t spi_speed_limit = 0;
__bit spi_speed_limit_updated = 0;
volatile int32_t spi_target = 0;
__bit spi_target_updated = 0;
__bit spi_link_lost = 0;
// Depasiri consecutive ale Timer0 fara octeti SPI; resetat la fiecare octet primit
static volatile uint8_t spi_timeout_count = 0;
// Tinta pe 32 de biti vine in doua pachete: LO se retine, HI o aplica doar
// daca pachetul anterior a fost chiar LO (un LO pierdut nu da o tinta gresita)
static uint16_t spi_target_lo;
static __bit spi_target_lo_valid = 0;
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
        spi_target_lo_valid = 0;

        // Legatura pierduta: motorul franeaza pana la 0 (motor_ramp_task), in
        // ambele moduri. Comenzile raman; urmatorul pachet complet reporneste.
        if (spi_timeout_count < SPI_TIMEOUT_STOP_COUNT) {
            spi_timeout_count++;
            if (spi_timeout_count == SPI_TIMEOUT_STOP_COUNT) {
                spi_link_lost = 1;
            }
        }
    }
}

// Octetul high al raspunsului, salvat odata cu cel low ca valoarea sa fie consistenta
static uint8_t spi_reply_hi;
// Partea de sus a pozitiei, retinuta la CMD_GET_POSITION_LO: cele doua
// jumatati sunt din acelasi moment
static uint8_t spi_position_latch[2];

// Apelata din ISR cand SSPIF este setat
void spi_isr(void) {
    uint8_t idx = spi_index;
    uint8_t rx_byte = SSPBUF;   // citirea goleste BF

    // SSPBUF se reincarca primul: master-ul asteapta doar pauza dintre octeti.
    // Dupa octetul 0 (comanda) se stie ce raspuns se cere. motor_position se
    // modifica doar in ISR-ul motorului, deci aici este consistenta.
    if (idx == 0) {
        if (rx_byte == CMD_GET_POSITION_LO) {
            SSPBUF = motor_position.b[0];
            spi_reply_hi = motor_position.b[1];
            spi_position_latch[0] = motor_position.b[2];
            spi_position_latch[1] = motor_position.b[3];
        } else if (rx_byte == CMD_GET_POSITION_HI) {
            SSPBUF = spi_position_latch[0];
            spi_reply_hi = spi_position_latch[1];
        } else {
            SSPBUF = (uint8_t)motor_current_sps;
            spi_reply_hi = (uint8_t)(motor_current_sps >> 8);
        }
    } else if (idx == 1) {
        SSPBUF = spi_reply_hi;
    } else {
        SSPBUF = SPI_ACK;
    }
    PIR1bits.SSPIF = 0;
    TMR0 = 0;
    INTCONbits.T0IF = 0;
    spi_timeout_count = 0;

    rx_packet.raw[idx] = rx_byte;
    idx++;
    if (idx >= SPI_BUF_SIZE) {
        // Pachet complet
        uint8_t lo_valid = spi_target_lo_valid;
        idx = 0;
        spi_link_lost = 0;
        spi_target_lo_valid = 0;
        if (rx_packet.cmd == CMD_SET_MOTOR_SPEED) {
            // Sensul vine din semn; aplicat la fiecare pachet, ca dupa un
            // reset al sclavului sensul sa fie refacut de urmatorul pachet
            int16_t velocity = rx_packet.val;
            if (velocity < 0) {
                spi_motor_dir = DIR_CCW;
                spi_motor_speed = (uint16_t)0U - (uint16_t)velocity;
            } else {
                spi_motor_dir = DIR_CW;
                spi_motor_speed = (uint16_t)velocity;
            }
            spi_motor_updated = 1;
        }
        else if (rx_packet.cmd == CMD_SET_SPEED_LIMIT) {
            spi_speed_limit = (uint16_t)rx_packet.val;
            spi_speed_limit_updated = 1;
        }
        else if (rx_packet.cmd == CMD_SET_TARGET_LO) {
            spi_target_lo = (uint16_t)rx_packet.val;
            spi_target_lo_valid = 1;
        }
        else if (rx_packet.cmd == CMD_SET_TARGET_HI) {
            if (lo_valid) {
                spi_target = (int32_t)(((uint32_t)(uint16_t)rx_packet.val << 16) |
                                       spi_target_lo);
                spi_target_updated = 1;
            }
        }
        else if (rx_packet.cmd == CMD_SET_MOTOR_RAMP) {
            // 0 ar bloca rampa; limitele le impune master-ul
            if (rx_packet.raw[1] != 0U) {
                motor_accel = rx_packet.raw[1];
            }
            if (rx_packet.raw[2] != 0U) {
                motor_decel = rx_packet.raw[2];
            }
        }
        // CMD_GET_POSITION_*: raspunsul a fost deja trimis
    }
    spi_index = idx;
}
