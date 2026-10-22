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
#define MATH_NOT_DONE 0x55AA

// Structura union identic? cu cea a Master-ului
typedef union {
    volatile unsigned char raw[SPI_BUF_SIZE];
    struct {
        unsigned char cmd;              // Primul octet este comanda
        int16_t val;    
    };
} PachetSPI_t;

typedef enum{
    CMD_READ_TEMP = 0x01,
    CMD_READ_POT = 0x02,
    CMD_READ_LIGHT = 0x03,
    CMD_SET_MOTOR_SPEED = 0x04,
    CMD_SET_MOTOR_DIR = 0x05,
    CMD_SEND_BME280_CALIB = 0x06,
    CMD_SEND_BME280_DATA = 0x07,
    CMD_READ_BME280_RES = 0x08,
} SPI_Commands_t;
// Buffere globale pentru transfer instant in ISR
extern volatile int16_t temp;
extern volatile uint8_t light;
extern volatile uint16_t pot;
volatile uint16_t spi_motor_speed;
extern __bit spi_motor_dir;
__bit spi_motor_updated;
#define BME280_CALIB_SIZE 33
typedef struct {
    uint16_t T1; 
    int16_t  T2, T3;
    uint16_t P1; 
    int16_t  P2, P3, P4, P5, P6, P7, P8, P9;
    uint8_t  H1, H3; 
    int16_t  H2, H4, H5; 
    int8_t   H6;
} BME280_Calib_Fields_t;


typedef union {
    BME280_Calib_Fields_t fields; // Access variables by name (e.g., cal.fields.T1)
    uint8_t bytes[BME280_CALIB_SIZE];            // Access data by byte index (e.g., cal.bytes[0])
} BME280_Calib_t;

#define REZ_SIZE 6
typedef struct{
    int16_t temperature;
    uint16_t pressure;
    uint16_t humidity;
} BME280_rez_fields;
typedef union{
    BME280_rez_fields fields;
    uint8_t bytes[REZ_SIZE];
} BME280_rez_t;
__bit   raw_done;
__bit   rez_done;
__bit   calib_done;

// 1. Unpacked Field Representation
typedef struct {
    int32_t temperature; // 4 bytes (20-bit value padded to 32-bit signed)
    int32_t pressure;    // 4 bytes (20-bit value padded to 32-bit signed)
    int32_t humidity;    // 4 bytes (16-bit value padded to 32-bit signed)
} BME280_Raw_Fields_t;

#define RAW_SIZE 12
// 2. The Transmittable Union
typedef union {
    BME280_Raw_Fields_t fields;
    uint8_t bytes[RAW_SIZE];       // 3 variables x 4 bytes each = 12 bytes total
} BME280_Raw_t;

// Prototipuri func?ii
void spi_init(void);
void spi_isr(void);
void spi_resync(void);

#endif	/* SPI_H */