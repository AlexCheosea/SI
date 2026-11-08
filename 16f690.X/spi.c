#include "spi.h"
#include "motor.h"
volatile PachetSPI_t rx_packet;
volatile unsigned char spi_index = 0;



// Definirea bufferelor de iesire (Motor)
volatile uint16_t spi_motor_speed = 0;
__bit spi_motor_dir;
__bit spi_motor_updated = 0;
void spi_init(void) {
    // 1. Setare directie pini
    TRIS_SCK = 1;  // SCK (RB6) = Intrare
    TRIS_SDI = 1;  // SDI (RB4) = Intrare
    TRIS_SDO = 0;  // SDO (RC7) = Iesire

    // Dezactivam modulele analogice de pe pinii utilizati (AN10 si AN9)
    ANSELHbits.ANS9 = 0;  //Dezactiveaza analogic pe SDO
    ANSELHbits.ANS10 = 0; // Dezactiveaza analogic pe SDI
    // 2. Configurare SSPSTAT (CKE=1, potrivit cu Master ASM)
    SSPEN = 0;
    SSPSTAT = 0b01000000; 

    // 3. Configurare SSPCON
    // SSPEN = 1 (Pornire hardware SPI)
    // CKP = 0 (Ceas repaus pe LOW)
    // SSPM = 0101 (Mod SPI Slave, pin SS dezactivat - SS legat direct la GND)
    SSPCON = 0b00100101; 

    // Incarcam anticipat primul byte in buffer-ul hardware
    SSPBUF = SPI_ACK; // Byte generic de ACK

    // 4. Activare intreruperi Hardware SPI
    PIR1bits.SSPIF = 0;     
    PIE1bits.SSPIE = 1;     
    INTCONbits.PEIE = 1;    
}
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

// Octetul high al raspunsului, citit in acelasi ISR cu octetul low (la octetul 0)
// ca valoarea trimisa sa fie consistenta chiar daca main schimba viteza.
static uint8_t spi_reply_hi;

// Apelata din isr_handler doar cand SSPIF este setat.
void spi_isr(void) {
    uint8_t idx = spi_index;
    uint8_t rx_byte = SSPBUF;   // citirea goleste BF

    // 1. Reincarcam SSPBUF CAT MAI REPEDE: master-ul asteapta doar pauza dintre
    // octeti. Raspunsul este mereu motor_current_sps (master-ul ignora raspunsul
    // la CMD_SET_MOTOR_DIR), deci nu mai trebuie sa decodam comanda inainte.
    if (idx == 0) {
        SSPBUF = (uint8_t)motor_current_sps;              // Low Byte
        spi_reply_hi = (uint8_t)(motor_current_sps >> 8);
    } else if (idx == 1) {
        SSPBUF = spi_reply_hi;                             // High Byte
    } else {
        SSPBUF = SPI_ACK;                                  // pentru pachetul urmator
    }
    PIR1bits.SSPIF = 0;
    TMR0 = 0;
    INTCONbits.T0IF = 0;

    // 2. Restul procesarii (nu mai este pe calea critica)
    rx_packet.raw[idx] = rx_byte;
    idx++;
    if (idx >= SPI_BUF_SIZE) {
        // --- Pachet complet primit ---
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