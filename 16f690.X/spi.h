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

// Pachetul SPI, identic cu cel al master-ului: comanda + valoare pe 16 biti
typedef union {
    volatile unsigned char raw[SPI_BUF_SIZE];
    struct {
        unsigned char cmd;
        int16_t val;
    };
} PachetSPI_t;

typedef enum{
    CMD_SET_MOTOR_SPEED = 0x04,
    CMD_SET_MOTOR_DIR = 0x05,
    CMD_SET_MOTOR_RAMP = 0x06,  // val: low = accelerare, high = franare
} SPI_Commands_t;
// Ultima comanda primita, scrisa in ISR
volatile uint16_t spi_motor_speed;
extern __bit spi_motor_dir;
__bit spi_motor_updated;



void spi_init(void);
void spi_isr(void);
void spi_resync(void);

#endif	/* SPI_H */
