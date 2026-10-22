#ifndef MAIN_H
#define MAIN_H

#include <xc.h>
#include <stdint.h>
#include <string.h>

#include "init.h"
#include "bme280.h"
#include "buzzer.h"
#include "hmi.h"
#include "spi.h"
#include "uart.h"

#define _XTAL_FREQ 18432000
#define SEG_ROW_LEN  9
#define UART_ROW_LEN 16 
// EEPROM Memory Map Allocation
#define EE_ADDR_SPEED_LOW    0x00   // motor_speed low byte
#define EE_ADDR_SPEED_HIGH   0x01   // motor_speed high byte
#define EE_ADDR_VOLUME       0x02   // muzicuta_volum byte
#define EE_ADDR_PITCH        0x03   // Key Beep pitch
#define EE_ADDR_DUR_LOW      0x04   // Key Beep duration low byte
#define EE_ADDR_DUR_HIGH     0x05   // Key Beep duration high byte

#define EE_ADDR_FLAGS        0x06   // Packed bits addr
#define FLAG_SOUND           0x01   // Masks for the packed bits
#define FLAG_OCTAVA          0x02
#define FLAG_DIR             0x04
//BME280 calibration struct ===================================================
extern BME280_Calib_t c;
extern BME280_Raw_t raw;
// =============================================================================
// STRUCTURE DEFINITIONS FOR THE UNIFIED MENU ENGINE
// =============================================================================
typedef struct {
    const char* const* rows; // Pointer to an array of string pointers in Flash
    uint8_t num_rows;        // Exactly how many rows this menu or submenu has
} MenuDescriptor_t;

// =============================================================================
// MAIN STATES & SEVEN-SEGMENT MENUS (Left intact for HMI)
// =============================================================================
typedef enum {
    ST_MENU_METEO = 0,
    ST_MENU_SPI,
    ST_MENU_SETARI,
    ST_MENU_MUZICUTA,
    ST_MENU_UART,
    NUM_STATES
} MainState_t;

const char menu_seg[][SEG_ROW_LEN] = {
    "Meteo  >",
    "SPI    >",
    "Setari >",
    "Muzicuta",
    "Uart buf"
};

// Submenu Enums & Segment Text
typedef enum { TEMP_BME = 0, PRES_BME, HUM_BME, NUM_SUB_METEO } SubMeteo_t;
const char submenu_meteo[][SEG_ROW_LEN] = { "TEMP BME", "PRES BME", "HUM  BME" };

typedef enum { TEMP_SPI = 0, POT_SPI, LIGHT_SPI, MOTOR_SPEED, MOTOR_DIR, NUM_SUB_SPI } SubSpi_t;
const char submenu_spi[][SEG_ROW_LEN] = { "TEMP SPI", "POT  SPI", "LIGHT   ", "MOT  SPD", "MOT  DIR" };

typedef enum { TONE, DURATION, VOLUME, KEY_BEEP, SAVE, NUM_SUB_SET } SubSet_t;
const char submenu_setari[][SEG_ROW_LEN] = { "BZZ  TON", "BZZ  DUR", "VOLUME  ", "KEY BEEP", "SAVE SET" };

typedef enum { ERR_MENU, NUM_ERRORS } Error_t;
const char error_seg[][SEG_ROW_LEN] = { "Err menu" };

// =============================================================================
// OPTIMIZED UART MENUS (Tightly packed in Flash, no spaces/padding bytes)
// =============================================================================

// =============================================================================
// OPTIMIZED UART MENUS (Auto-numbered & Auto-padded at runtime)
// =============================================================================

// Main menu options (Exactly 11 characters so the '>' aligns perfectly to the right wall)
const char* const menu_uart_txt[] = {
    "Meteo     >", // Fits perfectly: |1.Meteo     >|
    "SPI       >", // Fits perfectly: |2.SPI       >|
    "Setari    >", // Fits perfectly: |3.Setari    >|
    "Audio grid",  // 10 chars -> auto-pads 1 space to the right wall
    "UART rxbuf"   // 10 chars -> auto-pads 1 space to the right wall
};

// Meteo Submenu (Adjusted to fit within the 11-character limit)
const char* const sub_meteo_uart_txt[] = {
    "Temp:--.- C", // 11 chars -> |1.Temp:--.- C|
    "Pres:-- hPa", // 11 chars -> |2.Pres:-- hPa|
    "Umid:--.- %"  // 11 chars -> |3.Umid:--.- %|
};

// SPI Submenu (Cleaned and adjusted to the 11-character limit)
const char* const sub_spi_uart_txt[] = {
    "Temp SPI",    // 8 chars  -> auto-padded dynamically
    "Pot [V]",  // 10 chars -> shortened "Potentio" to "Potent"
    "Light lvl", // 11 chars -> shortened "Luminozitate"
    "Motor spd",  // 10 chars -> shortened "viteza" to "vit."
    "Motor dir"   // 10 chars -> auto-padded dynamically
};

// Settings Submenu (Cleaned and adjusted to the 11-character limit)
const char* const sub_setari_uart_txt[] = {
    "Bzz Tone",
    "Bzz Time",  
    "Volume sys", 
    "Key sound",
    "Save set"
};

const char* const error_uart_txt[] = {
    "Eroare meniu"
};

// =============================================================================
// MASTER DESCRIPTOR ARRAYS (For seamless menuState indexing)
// =============================================================================
// Allows writing: uart_print_full_menu(submenu_descriptors[menuState]);
extern const MenuDescriptor_t master_menu_desc;
extern const MenuDescriptor_t submenu_descriptors[NUM_STATES];

// =============================================================================
// UNIONS & APP CONFIG STRUCTURES
// =============================================================================
typedef union {
    SubMeteo_t meteo;
    SubSpi_t spi;
    SubSet_t setari;
    uint8_t raw;
} SubMenu_t;

typedef enum {
    inMenu = 0,
    inSubMenu = 1,
    inApp = 2
} MeniuContext_t;



#endif /* MAIN_H */