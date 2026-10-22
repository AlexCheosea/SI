#ifndef BME280_H
#define BME280_H

#include <stdint.h>

// Configurarea Pinilor (Bit-Banging)
#define I2C_SDA_TRIS  TRISA4
#define I2C_SDA_PORT  RA4
#define I2C_SCL_TRIS  TRISA5
#define I2C_SCL_PORT  RA5
#define I2C_ACK  0
#define I2C_NACK 1

// ===============================
// BME280 Registers and addresses
// ===============================
#define BME280_CHIP_ID  0x60
#define BME280_ADDR_W   0xEC 
#define BME280_ADDR_R   0xED
#define BME280_REG_CTRL_HUM  0xF2 
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#define BME280_OSRS_TEMP_POS   5
#define BME280_OSRS_PRESS_POS  2
// ===============================
// BME280 OVERSAMPLING SETTINGS
// ===============================
#define BME280_OSRS_X1        0x01
#define BME280_OSRS_X2        0x02
#define BME280_OSRS_X4        0x03
#define BME280_OSRS_X8        0x04
#define BME280_OSRS_X16       0x05


// ===============================
// BME280 MODE SETTINGS
// ===============================
#define BME280_MODE_SLEEP     0x00
#define BME280_MODE_FORCED    0x01
#define BME280_MODE_NORMAL    0x03


// ===============================
// FILTER SETTINGS (CONFIG register)
// ===============================
#define BME280_FILTER_OFF     (0x00 << 2)
#define BME280_FILTER_X2      (0x01 << 2)
#define BME280_FILTER_X4      (0x02 << 2)
#define BME280_FILTER_X8      (0x03 << 2)
#define BME280_FILTER_X16     (0x04 << 2)


// ===============================
// STANDBY TIMES (NORMAL MODE ONLY)
// ===============================
#define BME280_STANDBY_0_5MS  (0x00 << 5)
#define BME280_STANDBY_62_5MS (0x01 << 5)
#define BME280_STANDBY_125MS  (0x02 << 5)
#define BME280_STANDBY_250MS  (0x03 << 5)
#define BME280_STANDBY_500MS  (0x04 << 5)
#define BME280_STANDBY_1000MS (0x05 << 5)
#define BME280_STANDBY_10MS   (0x06 << 5)
#define BME280_STANDBY_20MS   (0x07 << 5)

// ===============================
// BME280 WORKING SETTINGS
// ===============================
#define BME280_CTRL_HUM_OSRS   BME280_OSRS_X4

#define BME280_CTRL_MEAS_NORMAL \
    ((BME280_OSRS_X4 << BME280_OSRS_TEMP_POS) | \
     (BME280_OSRS_X4 << BME280_OSRS_PRESS_POS) | \
     BME280_MODE_NORMAL)

#define BME280_CONFIG_DEFAULT \
    (BME280_STANDBY_500MS | BME280_FILTER_X4)


uint8_t bme280_init(void);

//Structura de calibrare
// 1. Define the internal fields structure
typedef struct {
    uint16_t T1; 
    int16_t  T2, T3;
    uint16_t P1; 
    int16_t  P2, P3, P4, P5, P6, P7, P8, P9;
    uint8_t  H1, H3; 
    int16_t  H2, H4, H5; 
    int8_t   H6;
} BME280_Calib_Fields_t;
#define BME280_CALIB_SIZE 33 // Dimensiunea structurii e calibrare

// 2. Wrap it inside the union to expose the byte array
typedef union {
    BME280_Calib_Fields_t fields; // Access variables by name (e.g., cal.fields.T1)
    uint8_t bytes[BME280_CALIB_SIZE];            // Access data by byte index (e.g., cal.bytes[0])
} BME280_Calib_t;

// Citește absolut toate datele dintr-un singur apel (salvează masiv memorie și timp pe I2C)
// - temp: grade Celsius x 100 (ex: 2455 = 24.55 C)
// - press: Presiune în Pascali (Pa)
// - hum: Umiditate în format Q22.10 (împarte valoarea la 1024 pentru a obține %RH)

void bme280_read_raw(void);

#endif /* BME280_LITE_H */