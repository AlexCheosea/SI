#include "bme280.h"
#include "spi.h"
#include <xc.h>

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 18432000UL
#endif

// Definiții clare pentru lizibilitatea protocolului I2C
#define I2C_ACK  0
#define I2C_NACK 1




/* * DIRECTIVA CRITICĂ: __at(0x190)
 * Forțează compilatorul XC8 să aloce această structură (33 octeți) în Bancul 3.
 * Astfel, Bancul 0 este complet eliberat pentru C stack.
 */
BME280_Calib_t c __at(0x190); 

// --- Primitive I2C Inline (Economisesc memorie ROM) ---
static void i2c_delay(void) { __delay_us(4); }
static void i2c_start(void) { I2C_SDA_TRIS=1; I2C_SCL_TRIS=1; i2c_delay(); I2C_SDA_PORT=0; I2C_SDA_TRIS=0; I2C_SCL_PORT=0; I2C_SCL_TRIS=0; i2c_delay(); }
static void i2c_stop(void)  { I2C_SDA_PORT=0; I2C_SDA_TRIS=0; I2C_SCL_TRIS=1; i2c_delay(); I2C_SDA_TRIS=1; i2c_delay(); }

static void i2c_write(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) I2C_SDA_TRIS = 1; else { I2C_SDA_PORT = 0; I2C_SDA_TRIS = 0; }
        i2c_delay(); I2C_SCL_TRIS = 1; i2c_delay(); I2C_SCL_PORT = 0; I2C_SCL_TRIS = 0;
        data = (uint8_t)(data << 1);
    }
    I2C_SDA_TRIS = 1; i2c_delay(); I2C_SCL_TRIS = 1; i2c_delay(); I2C_SCL_PORT = 0; I2C_SCL_TRIS = 0; // Ignoră ACK
}

static uint8_t i2c_read(uint8_t ack_option) {
    uint8_t data = 0;
    I2C_SDA_TRIS = 1; i2c_delay();
    for (uint8_t i = 0; i < 8; i++) {
        I2C_SCL_TRIS = 1; i2c_delay();
        data = (uint8_t)((data << 1) | (I2C_SDA_PORT));
        I2C_SCL_PORT = 0; I2C_SCL_TRIS = 0; i2c_delay();
    }
    if (ack_option == I2C_NACK) I2C_SDA_TRIS = 1; else { I2C_SDA_PORT = 0; I2C_SDA_TRIS = 0; }
    i2c_delay(); I2C_SCL_TRIS = 1; i2c_delay(); I2C_SCL_PORT = 0; I2C_SCL_TRIS = 0; I2C_SDA_TRIS = 1;
    return data;
}

// --- Citirea Calibrării (Burst Read CORECTAT) ---
static void bme280_read_calib(void) {
    uint8_t rx[24];
    
    i2c_start(); i2c_write(BME280_ADDR_W); i2c_write(0x88);
    i2c_start(); i2c_write(BME280_ADDR_R);
    for(uint8_t i = 0; i < 23; i++) {
        rx[i] = i2c_read(I2C_ACK); 
    }
    rx[23] = i2c_read(I2C_NACK);   
    i2c_stop();
    
    c.fields.T1 = (uint16_t)((rx[1]<<8) | rx[0]); 
    c.fields.T2 = (int16_t)((rx[3]<<8) | rx[2]); 
    c.fields.T3 = (int16_t)((rx[5]<<8) | rx[4]);
    c.fields.P1 = (uint16_t)((rx[7]<<8) | rx[6]); 
    c.fields.P2 = (int16_t)((rx[9]<<8) | rx[8]); 
    c.fields.P3 = (int16_t)((rx[11]<<8) | rx[10]);
    c.fields.P4 = (int16_t)((rx[13]<<8)| rx[12]); 
    c.fields.P5 = (int16_t)((rx[15]<<8)| rx[14]); 
    c.fields.P6 = (int16_t)((rx[17]<<8)| rx[16]);
    c.fields.P7 = (int16_t)((rx[19]<<8)| rx[18]); 
    c.fields.P8 = (int16_t)((rx[21]<<8)| rx[20]); 
    c.fields.P9 = (int16_t)((rx[23]<<8)| rx[22]);

    i2c_start(); i2c_write(BME280_ADDR_W); i2c_write(0xA1);
    i2c_start(); i2c_write(BME280_ADDR_R);
    c.fields.H1 = i2c_read(I2C_NACK); 
    i2c_stop();

    i2c_start(); i2c_write(BME280_ADDR_W); i2c_write(0xE1);
    i2c_start(); i2c_write(BME280_ADDR_R);
    for(uint8_t i = 0; i < 6; i++) {
        rx[i] = i2c_read(I2C_ACK);
    }
    c.fields.H6 = (int8_t)i2c_read(I2C_NACK); 
    i2c_stop();

    c.fields.H2 = (int16_t)((rx[1]<<8) | rx[0]);
    c.fields.H3 = rx[2];
    c.fields.H4 = (int16_t)(((int16_t)rx[3] << 4) | (rx[4] & 0x0F));
    c.fields.H5 = (int16_t)(((int16_t)rx[5] << 4) | (rx[4] >> 4));
}

uint8_t bme280_init(void) {
    I2C_SDA_TRIS = 1; I2C_SCL_TRIS = 1; I2C_SDA_PORT = 0; I2C_SCL_PORT = 0;
    //1. Verifica daca este cipul
    i2c_start(); i2c_write(BME280_ADDR_W); i2c_write(0xD0);
    i2c_start(); i2c_write(BME280_ADDR_R);
    uint8_t id = i2c_read(I2C_NACK);
    i2c_stop();
    if (id != BME280_CHIP_ID) return 0;
    //2. Seteaza optiuni umiditate
    i2c_start(); i2c_write(BME280_ADDR_W); i2c_write(BME280_REG_CTRL_HUM);  i2c_write(BME280_CTRL_HUM_OSRS); i2c_stop(); 
    //3. Seteaza optiuni Standby si filtru
    i2c_start(); i2c_write(BME280_ADDR_W); i2c_write(BME280_REG_CONFIG);    i2c_write(BME280_CONFIG_DEFAULT); i2c_stop(); 
    //4. Seteaza Oversampling si trezeste cipul
    i2c_start(); i2c_write(BME280_ADDR_W); i2c_write(BME280_REG_CTRL_MEAS); i2c_write(BME280_CTRL_MEAS_NORMAL); i2c_stop(); 
    //5. Citeste valori calibrare
    bme280_read_calib();
    return 1;
}
BME280_Raw_t raw;
// --- Citirea Datelor Brute (Burst Read Secvențial) ---
void bme280_read_raw() {
    uint8_t rx[8];

    // Adresa de start pentru date este 0xF7 (BME280_REG_PRESS_MSB)
    i2c_start(); 
    i2c_write(BME280_ADDR_W); 
    i2c_write(0xF7); 
    
    // Repornire în mod Citire
    i2c_start(); 
    i2c_write(BME280_ADDR_R);
    
    // Citire secvențială a celor 8 regiştri de date (0xF7 - 0xFE)
    for(uint8_t i = 0; i < 7; i++) {
        rx[i] = i2c_read(I2C_ACK);
    }
    rx[7] = i2c_read(I2C_NACK); 
    i2c_stop();

    // Recompunere Presiune (20 biți: rx[0]=msb, rx[1]=lsb, rx[2]=xlsb)
    raw.fields.pressure = ((int32_t)rx[0] << 12) | ((int32_t)rx[1] << 4) | (rx[2] >> 4);

    // Recompunere Temperatură (20 biți: rx[3]=msb, rx[4]=lsb, rx[5]=xlsb)
    raw.fields.temperature = ((int32_t)rx[3] << 12) | ((int32_t)rx[4] << 4) | (rx[5] >> 4);

    // Recompunere Umiditate (16 biți: rx[6]=msb, rx[7]=lsb)
    raw.fields.humidity = ((int32_t)rx[6] << 8) | rx[7];
}