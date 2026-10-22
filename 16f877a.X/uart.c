/*
 * File:   uart.c
 * Author: Alex
 *
 * Created on June 7, 2026, 10:42 PM
 */


#include <xc.h>
#include "uart.h"
void uart_print_string(const char* str){
    while(UART_TX_STATUS == UART_TX_BUSY); //Așteptăm finalizarea ultimei transmisii
    if(UART_TX_STATUS == UART_TX_ERROR) {
        // Resetăm starea de eroare
        UART_TX_STATUS = UART_TX_IDLE;
    }
    di();
    for(UART_TX_LEN = 0; (UART_TX_LEN < UART_BUF_SIZE - 1) && (str[UART_TX_LEN] != '\0'); UART_TX_LEN++){
        UART_TX_BUF[UART_TX_LEN] = (unsigned char)str[UART_TX_LEN];
    }
    
    // În acest punct, UART_TX_LEN indică EXACT prima poziție liberă din buffer
    // (care va fi maxim 15, adică ultima poziție din buffer)
    UART_TX_BUF[UART_TX_LEN] = RX_MSG_TERMINATOR; 
    
    // Incrementăm ca să includem și terminatorul în lungimea totală transmisă
    UART_TX_LEN++; 
    ei();
    uart_send();
    return;
}

uint8_t uart_read_string(char *out_buf) {
    uint8_t len;

    /* uart_receive() returns UART_RX_LEN when status == RX_STATE_READY, else 0 */
    len = uart_receive();
    if (len == 0)
        return 0;

    /* Error state: reset and bail */
    if (UART_RX_STATUS == UART_RX_ERROR) {
        uart_release();
        return 0;
    }

    /*
     * len includes the '\n' terminator written by the ASM driver.
     * Copy everything except that last byte, then null-terminate for C.
     */
    di();
    uint8_t data_len = (len > 0) ? (len - 1) : 0;
    if (data_len > UART_BUF_SIZE - 1)
        data_len = UART_BUF_SIZE - 1;

    for (uint8_t i = 0; i < data_len; i++)
        out_buf[i] = (char)UART_RX_BUF[i];

    out_buf[data_len] = '\0';
    ei();

    /* Release the ASM driver so it can accept the next packet */
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
