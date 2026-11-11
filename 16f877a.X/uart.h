#ifndef UART_H
#define UART_H
#include <stdint.h>
//===============================================================================
//Define-uri pentru constante
//===============================================================================
#define UART_RX_IDLE      0
#define UART_RX_ACTIVE    1
#define UART_RX_READY     2
#define UART_RX_ERROR     3
#define UART_TX_IDLE      0
#define UART_TX_BUSY      1
#define UART_TX_ERROR     3
#define UART_BUF_SIZE     16
#define RX_MSG_TERMINATOR 10
//===============================================================================
// VARIABILE EXTERNE (definite in uart_asm.s: control + TX_BUF in Bank 2,
// RX_BUF in Bank 3). __bank(n) ii spune compilatorului bancul (necesita
// -maddrqual=require); trebuie sa corespunda cu class=BANKn din uart_asm.s.
//===============================================================================
// Buffer-ul de recepție UART (Dimensiune fixă: 16 octeți)
extern volatile __bank(3) uint8_t UART_RX_BUF[UART_BUF_SIZE];
// Buffer-ul de transmisie UART (Dimensiune fixă: 16 octeți)
extern volatile __bank(2) uint8_t UART_TX_BUF[UART_BUF_SIZE];
// Lungimea mesajului primit în buffer-ul de recepție
extern volatile __bank(2) uint8_t UART_RX_LEN;
// Lungimea pachetului care urmează să fie trimis (Trebuie scrisă din C înainte de uart_send)
extern volatile __bank(2) uint8_t UART_TX_LEN;
// Variabila status curent transmisie
extern volatile __bank(2) uint8_t UART_TX_STATUS;
// Variabila status curent receptie
extern volatile __bank(2) uint8_t UART_RX_STATUS;
//===============================================================================
// PROTOTIPURI DE FUNCTII (Exportate din driverul de asamblare)
//===============================================================================

/**
 * @brief Inițializează modulul hardware UART
 * configurabil din assembly 
 */
extern void uart_init(void);

/**
 * @brief Subrutina ISR pentru tratarea întreruperii de transmisie (TX).
 * Trebuie apelată din ISR-ul principal din C când flag-ul TXIF este setat.
 */
extern void uart_tx_isr(void);

/**
 * @brief Subrutina ISR pentru tratarea întreruperii de recepție (RX).
 * Trebuie apelată din ISR-ul principal din C când flag-ul RCIF este setat.
 */
extern void uart_rx_isr(void);

/**
 * @brief Declanșează transmisia datelor aflate în UART_TX_BUF.
 * Înainte de apel, din C trebuie completat UART_TX_BUF și setat UART_TX_LEN.
 */
extern void uart_send(void);

//===============================================================================
// FUNCTII C (uart.c)
//===============================================================================

/**
 * @brief Verifică dacă s-a primit un mesaj complet (terminat cu '\n').
 * @return Lungimea mesajului, inclusiv terminatorul, dacă
 * UART_RX_STATUS == UART_RX_READY, altfel 0.
 */
uint8_t uart_receive(void);

/**
 * @brief Eliberează buffer-ul de recepție: îl golește și pune
 * UART_RX_STATUS = UART_RX_IDLE. Până atunci, octeții noi sunt ignorați.
 */
void uart_release(void);

/**
 * @brief Trimite un șir pe UART ca o linie: maxim UART_BUF_SIZE-1 (15)
 * caractere, la care se adaugă '\n'. Așteaptă terminarea liniei anterioare.
 * @param str Șirul de caractere de trimis.
 */
void uart_print_string(const char* str);

/**
 * @brief Reads a received UART string into a C buffer.
 * @param out_buf Destination buffer, must be at least UART_BUF_SIZE bytes.
 * @return Number of data bytes copied (excluding terminator), or 0 if not ready.
 */
uint8_t uart_read_string(char *out_buf);


#endif // UART_H