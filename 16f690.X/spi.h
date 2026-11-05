#ifndef SPI_H
#define	SPI_H

#include <xc.h>
#include <stdint.h>

#define SPI_BUF_SIZE 3

// --- CONFIGURARE PINI CONFORM SCHEMEI ---
#define TRIS_SCK      TRISBbits.TRISB6  // Pin 11 (Intrare Ceas)
#define TRIS_SDI      TRISBbits.TRISB4  // Pin 13 (Intrare Date de la Master)
#define TRIS_SDO      TRISCbits.TRISC7  // Pin 9  (Iesire Date catre Master)
#define SPI_ACK           0xAA

// Structura union identica cu cea a Master-ului
typedef union {
    volatile unsigned char raw[SPI_BUF_SIZE];
    struct {
        unsigned char cmd;              // Primul octet este comanda
        int16_t val;    
    };
} PachetSPI_t;

typedef enum{
    CMD_SET_MOTOR_SPEED = 0x04,
    CMD_SET_MOTOR_DIR = 0x05,
} SPI_Commands_t;
// Buffere globale pentru transfer instant in ISR
volatile uint16_t spi_motor_speed;
extern __bit spi_motor_dir;
__bit spi_motor_updated;


// Prototipuri func?ii
void spi_init(void);
void spi_isr(void);
void spi_resync(void);

#endif	/* SPI_H */