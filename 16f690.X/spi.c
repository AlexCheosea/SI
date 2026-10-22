#include "spi.h"
#include "motor.h"
volatile PachetSPI_t rx_packet;
volatile PachetSPI_t tx_packet;
volatile unsigned char spi_index = 0;

// Definirea bufferelor de intrare (Senzori)
volatile int16_t temp = 0;
volatile uint8_t light = 0;
volatile uint16_t pot = 0;

// Definirea bufferelor de iesire (Motor)
volatile uint16_t spi_motor_speed = 0;
__bit spi_motor_dir;
__bit spi_motor_updated = 0;
volatile BME280_rez_t rez;
volatile BME280_Raw_t raw;
volatile BME280_Calib_t c;
void spi_init(void) {
    // 1. Setare direc?ie pini
    TRIS_SCK = 1;  // SCK (RB6) = Intrare
    TRIS_SDI = 1;  // SDI (RB4) = Intrare
    TRIS_SDO = 0;  // SDO (RC7) = Ie?ire

    // Dezactiv?m modulele analogice de pe pinii utiliza?i (AN10 ?i AN9)
    ANSELHbits.ANS9 = 0;  //Dezactiveaza analogic pe SDO
    ANSELHbits.ANS10 = 0; // Dezactiveaza analogic pe SDI
    // 2. Configurare SSPSTAT (CKE=1, potrivit cu Master ASM)
    SSPEN = 0;
    SSPSTAT = 0b01000000; 

    // 3. Configurare SSPCON
    // SSPEN = 1 (Pornire hardware SPI)
    // CKP = 0 (Ceas repaus pe LOW)
    // SSPM = 0101 (Mod SPI Slave, pin SS dezactivat - SS legat direct la GND)
    SSPCON = 0b00100101; 

    // Incarcam anticipat primul byte in buffer-ul hardware
    tx_packet.cmd = SPI_ACK; // Byte generic de ACK
    SSPBUF = tx_packet.raw[0];

    // 4. Activare �ntreruperi Hardware SPI
    PIR1bits.SSPIF = 0;     
    PIE1bits.SSPIE = 1;     
    INTCONbits.PEIE = 1;    
}
void spi_resync(void) {
    if(INTCONbits.T0IF) {
        INTCONbits.T0IF = 0;
        
        spi_index = 0;
        SSPEN = 0;
        uint8_t dummy = SSPBUF;
        SSPCONbits.WCOL  = 0;
        SSPCONbits.SSPOV = 0;
        SSPEN = 1;
        tx_packet.cmd = SPI_ACK;
        SSPBUF = tx_packet.raw[0];
        TMR0 = 0;
    }
}

static uint8_t byte_idx = 0;
__bit   raw_done = 0;
__bit   rez_done = 0;
__bit   calib_done = 0;
void spi_isr(void) {
    TMR0 = 0;
    
    INTCONbits.T0IF = 0;
    if (PIR1bits.SSPIF) {
        unsigned char rx_byte = SSPBUF; 
        
        rx_packet.raw[spi_index] = rx_byte;
        spi_index++;

        if (spi_index == 1) {
            // --- OCTETUL 0: Am primit comanda ---
            int16_t val_to_send = 0;
            
            // Evaluam comanda folosind noul enum
            switch((SPI_Commands_t)rx_byte) {
                case CMD_READ_TEMP:  val_to_send = temp;  break;
                case CMD_READ_POT:   val_to_send = pot;   break;
                case CMD_READ_LIGHT: val_to_send = light; break;
                case CMD_READ_BME280_RES:
                    if(rez_done){
                        if(byte_idx < REZ_SIZE){
                        val_to_send = (rez.bytes[byte_idx]<<8) | byte_idx;
                        byte_idx++;
                        }
                        else{
                            byte_idx = 0;
                            rez_done = 0;
                        }
                    }else
                        val_to_send = MATH_NOT_DONE;
                break;
                // Pentru comenzile de SET, Master-ul doar scrie. 
                // Returnam 0

                case CMD_SET_MOTOR_SPEED:
                case CMD_SET_MOTOR_DIR:
                case CMD_SEND_BME280_CALIB:                    
                case CMD_SEND_BME280_DATA:
                
                default:
                    val_to_send = 0;
                    break;
            }
            
            tx_packet.val = val_to_send;
            SSPBUF = tx_packet.raw[1]; // Incarcam Low Byte pentru Master
        } 
        else if (spi_index == 2) {
            // --- OCTETUL 1: Master-ul a citit Low Byte ---
            SSPBUF = tx_packet.raw[2]; // Incarcam High Byte pentru Master
        } 
        else if (spi_index >= SPI_BUF_SIZE) {
            // --- OCTETUL 2: Pachet complet primit si trimis ---
            spi_index = 0;
            
            // Verificam daca Master-ul a trimis o comanda de configurare motor
            // Datele primite sunt complet asamblate acum in rx_packet.val
            if (rx_packet.cmd == CMD_SET_MOTOR_SPEED) {
                spi_motor_speed = (uint16_t)rx_packet.val;
                spi_motor_updated = 1;
            } 
            else if (rx_packet.cmd == CMD_SET_MOTOR_DIR) {
                spi_motor_dir = (uint8_t)rx_packet.val;
                spi_motor_updated = 1;
            }
            else if (rx_packet.cmd == CMD_SEND_BME280_CALIB){
                if(rx_packet.raw[1] < BME280_CALIB_SIZE)
                    c.bytes[rx_packet.raw[1]] = rx_packet.raw[2];
                
                    
            }
            else if (rx_packet.cmd == CMD_SEND_BME280_DATA){
                if(rx_packet.raw[1] < RAW_SIZE)
                raw.bytes[rx_packet.raw[1]] = rx_packet.raw[2];
                
                if(rx_packet.raw[1] == RAW_SIZE-1)
                    raw_done = 1;
            }

            SSPBUF = tx_packet.raw[0]; // Resetam cu ACK pentru pachetul urmator
        }

        PIR1bits.SSPIF = 0; 
    }
}