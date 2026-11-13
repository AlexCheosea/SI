/*
 * File:   uart.c
 * Author: Alex
 *
 */


#include <xc.h>
#include "uart.h"
// Date folosite de uart_asm.s. Controlul + TX in Bancul 2, RX in Bancul 3:
// codul asm presupune bancurile (BANKSEL pe una, BANKMASK pe celelalte,
// BCF RP1 pentru Bancul 2 -> 0).
volatile __bank(2) uint8_t UART_RX_IDX;
volatile __bank(2) uint8_t UART_RX_STATUS;
volatile __bank(2) uint8_t UART_RX_LEN;
volatile __bank(2) uint8_t UART_TX_IDX;
volatile __bank(2) uint8_t UART_TX_LEN;
volatile __bank(2) uint8_t UART_TX_STATUS;
volatile __bank(2) uint8_t UART_TX_BUF[UART_BUF_SIZE];
volatile __bank(3) uint8_t UART_RX_BUF[UART_BUF_SIZE];
void uart_print_string(const char* str){
    uint8_t len = 0;
    char c;

    while(UART_TX_STATUS == UART_TX_BUSY); // asteapta linia anterioara
    if(UART_TX_STATUS == UART_TX_ERROR) {
        UART_TX_STATUS = UART_TX_IDLE;
    }
    // Transmisia este oprita aici, deci intreruperea TX nu atinge bufferul
    while((len < UART_BUF_SIZE - 1) && ((c = *str++) != '\0')){
        UART_TX_BUF[len++] = (unsigned char)c;
    }

    UART_TX_BUF[len++] = RX_MSG_TERMINATOR;

    UART_TX_LEN = len; // inclusiv terminatorul
    uart_send();
    return;
}

uint8_t uart_read_string(char *out_buf) {
    uint8_t len;

    len = uart_receive();
    if (len == 0)
        return 0;

    if (UART_RX_STATUS == UART_RX_ERROR) {
        uart_release();
        return 0;
    }

    // len include terminatorul '\n'; se copiaza fara el si se termina cu '\0'
    di();
    uint8_t data_len = (len > 0) ? (len - 1) : 0;
    if (data_len > UART_BUF_SIZE - 1)
        data_len = UART_BUF_SIZE - 1;

    for (uint8_t i = 0; i < data_len; i++)
        out_buf[i] = (char)UART_RX_BUF[i];

    out_buf[data_len] = '\0';
    ei();

    uart_release();

    return data_len;
}
uint8_t uart_receive(void) {
    if (UART_RX_STATUS == UART_RX_READY)
        return UART_RX_LEN;
    return 0;
}

void uart_release(void) {
    for(uint8_t i = 0; i < UART_RX_LEN; i++)
        UART_RX_BUF[i] = 0;
    UART_RX_STATUS = UART_RX_IDLE;

}
