#ifndef SPI_H
#define	SPI_H

#include <xc.h>
#define SPI_BUF_SIZE 3  // Dimensiunea bufferului de transfer SPI (în octeți)
#define DIR_CW   1
#define DIR_CCW  0
#define MATH_NOT_DONE 0x55AA
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
    CMD_READ_TEMP = 0x01,
    CMD_READ_POT = 0x02,
    CMD_READ_LIGHT = 0x03,
    CMD_SET_MOTOR_SPEED = 0x04,
    CMD_SET_MOTOR_DIR = 0x05,
    CMD_SEND_BME280_CALIB = 0x06,
    CMD_SEND_BME280_DATA = 0x07,
    CMD_READ_BME280_RES = 0x08,
} SPI_Commands_t;


typedef struct{
    int16_t temperature;
    uint16_t pressure;
    uint16_t humidity;
} BME280_rez_fields;
#define REZ_SIZE 6
typedef union{
    BME280_rez_fields fields;
    uint8_t bytes[REZ_SIZE];
} BME280_rez_t;
// 1. Unpacked Field Representation
typedef struct {
    int32_t temperature; // 4 bytes (20-bit value padded to 32-bit signed)
    int32_t pressure;    // 4 bytes (20-bit value padded to 32-bit signed)
    int32_t humidity;    // 4 bytes (16-bit value padded to 32-bit signed)
} BME280_Raw_Fields_t;

// 2. The Transmittable Union
#define RAW_SIZE 12
typedef union {
    BME280_Raw_Fields_t fields;
    uint8_t bytes[RAW_SIZE];       // 3 variables x 4 bytes each = 12 bytes total
} BME280_Raw_t;
#endif	/* SPI_H */