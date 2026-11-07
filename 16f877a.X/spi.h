#ifndef SPI_H
#define	SPI_H

#include <xc.h>
#define SPI_BUF_SIZE 3  // Dimensiunea bufferului de transfer SPI (în octeți)
#define DIR_CW   1
#define DIR_CCW  0
// Bufferul fizic alocat in limbaj de asamblare
extern volatile unsigned char __section("spi_data_bank3") spi_buf[SPI_BUF_SIZE];

// Structura polimorfă de SPI_BUF_SIZE octeți, care poate fi interpretată în mai multe moduri
typedef union {
// 1. Perspectiva Driverului ASM: un masiv brut de SPI_BUF_SIZE octeți
volatile unsigned char raw[SPI_BUF_SIZE];

// 2. Perspectiva din C
struct {
    unsigned char cmd;              // Primul octet este mereu comanda
    int16_t val;    
    };
}PachetSPI_t;

// Macro pentru acces instant
#define SPI_PACKET  ((volatile PachetSPI_t*)spi_buf)

void spi_init(void);
void extern __far spi_transfer_packet(void);
//Comenzi SPI
typedef enum {
    CMD_SET_MOTOR_SPEED = 0x04,
    CMD_SET_MOTOR_DIR = 0x05
} SPI_Commands_t;






#endif	/* SPI_H */