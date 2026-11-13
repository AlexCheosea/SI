#ifndef SPI_H
#define	SPI_H

#include <xc.h>
#define SPI_BUF_SIZE 3  // octeti pe pachet
#define DIR_CW   1
#define DIR_CCW  0
// Bufferul este definit in spi_asm.s; __bank(n) trebuie sa corespunda cu
// class=BANKn de acolo (necesita -maddrqual=require).
extern volatile __bank(3) unsigned char spi_buf[SPI_BUF_SIZE];

// Pachetul SPI: octeti bruti pentru driverul ASM, comanda + valoare din C
typedef union {
volatile unsigned char raw[SPI_BUF_SIZE];

struct {
    unsigned char cmd;
    int16_t val;
    };
}PachetSPI_t;

#define SPI_PACKET  ((volatile PachetSPI_t*)spi_buf)

void spi_init(void);
void extern __far spi_transfer_packet(void);
typedef enum {
    CMD_SET_MOTOR_SPEED = 0x04,
    CMD_SET_MOTOR_DIR = 0x05
} SPI_Commands_t;






#endif	/* SPI_H */
