#ifndef SPI_H
#define	SPI_H

#include <xc.h>
#include <stdint.h>

#define SPI_BUF_SIZE 3

// Pinii SPI
#define TRIS_SCK      TRISBbits.TRISB6  // ceas (intrare)
#define TRIS_SDI      TRISBbits.TRISB4  // date de la master
#define TRIS_SDO      TRISCbits.TRISC7  // date catre master
#define SPI_ACK           0xAA
// Failsafe: dupa atatea depasiri consecutive ale Timer0 fara niciun octet SPI,
// motorul franeaza pana la 0 (cu rampa; spi_link_lost). O depasire = 256 x 32 x 4 / Fosc
// = 4.096 ms la 8 MHz, deci 100 -> ~410 ms. Maxim 255. Trebuie sa fie peste
// pauza maxima dintre pachetele master-ului (MOTOR_SPEED_REPORT_TIMER = 150 ms).
#define SPI_TIMEOUT_STOP_COUNT 100U

// Pachetul SPI, identic cu cel al master-ului: comanda + valoare pe 16 biti
typedef union {
    volatile unsigned char raw[SPI_BUF_SIZE];
    struct {
        unsigned char cmd;
        int16_t val;
    };
} PachetSPI_t;

// Raspunsul (octetii 1-2) este viteza curenta, cu exceptia CMD_GET_POSITION_*.
typedef enum{
    CMD_SET_MOTOR_SPEED = 0x04, // mod viteza; val cu semn: pasi/s, > 0 = DIR_CW, < 0 = DIR_CCW
    CMD_SET_MOTOR_RAMP = 0x06,  // val: low = accelerare, high = franare
    CMD_SET_SPEED_LIMIT = 0x07, // mod pozitie: viteza maxima, pasi/s (fara semn)
    CMD_SET_TARGET_LO = 0x08,   // tinta (pasi, 32 de biti cu semn): cei 16 biti de jos
    CMD_SET_TARGET_HI = 0x09,   // cei 16 biti de sus; aplica tinta (doar imediat dupa LO) -> mod pozitie
    CMD_GET_POSITION_LO = 0x0A, // raspuns: pozitia, 16 biti de jos (retine si partea de sus)
    CMD_GET_POSITION_HI = 0x0B, // raspuns: partea de sus retinuta la CMD_GET_POSITION_LO
} SPI_Commands_t;
// Ultimele comenzi primite, scrise in ISR, citite in main cu intreruperile oprite
extern volatile uint16_t spi_motor_speed;
extern __bit spi_motor_dir;
extern __bit spi_motor_updated;
extern volatile uint16_t spi_speed_limit;
extern __bit spi_speed_limit_updated;
extern volatile int32_t spi_target;
extern __bit spi_target_updated;
// Legatura pierduta (SPI_TIMEOUT_STOP_COUNT depasiri): motorul franeaza pana
// la 0 in ambele moduri; sters de urmatorul pachet complet
extern __bit spi_link_lost;



void spi_init(void);
void spi_isr(void);
void spi_resync(void);

#endif	/* SPI_H */
