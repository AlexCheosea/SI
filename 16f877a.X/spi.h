#ifndef SPI_H
#define	SPI_H

#include <xc.h>
#define SPI_BUF_SIZE 3  // octeti pe pachet
#define SPI_ACK      0xAA   // primul octet al raspunsului unui sclav sincronizat
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
// Raspunsul (octetii 1-2) este viteza curenta, cu exceptia CMD_GET_POSITION_*.
typedef enum {
    CMD_SET_MOTOR_SPEED = 0x04, // mod viteza; val cu semn: pasi/s, > 0 = DIR_CW, < 0 = DIR_CCW
    CMD_SET_MOTOR_RAMP = 0x06,  // val: low = accelerare, high = franare
    CMD_SET_SPEED_LIMIT = 0x07, // mod pozitie: viteza maxima, pasi/s (fara semn)
    CMD_SET_TARGET_LO = 0x08,   // tinta (pasi, 32 de biti cu semn): cei 16 biti de jos
    CMD_SET_TARGET_HI = 0x09,   // cei 16 biti de sus; aplica tinta (doar imediat dupa LO) -> mod pozitie
    CMD_GET_POSITION_LO = 0x0A, // raspuns: pozitia, 16 biti de jos (sclavul retine partea de sus)
    CMD_GET_POSITION_HI = 0x0B  // raspuns: partea de sus retinuta la CMD_GET_POSITION_LO
} SPI_Commands_t;






#endif	/* SPI_H */
