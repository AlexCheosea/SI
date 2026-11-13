#ifndef UART_H
#define UART_H
#include <stdint.h>
#define UART_RX_IDLE      0
#define UART_RX_ACTIVE    1
#define UART_RX_READY     2
#define UART_RX_ERROR     3
#define UART_TX_IDLE      0
#define UART_TX_BUSY      1
#define UART_TX_ERROR     3
#define UART_RX_BUF_SIZE  16    // RX_BUF_SIZE din uart_asm.s
#define UART_TX_BUF_SIZE  32    // TX_BUF_SIZE din uart_asm.s
#define RX_MSG_TERMINATOR 10
// Variabile definite in uart_asm.s; __bank(n) trebuie sa corespunda cu
// class=BANKn de acolo (necesita -maddrqual=require).
extern volatile __bank(3) uint8_t UART_RX_BUF[UART_RX_BUF_SIZE];
extern volatile __bank(2) uint8_t UART_TX_BUF[UART_TX_BUF_SIZE];
extern volatile __bank(2) uint8_t UART_RX_LEN;    // lungimea mesajului primit
extern volatile __bank(2) uint8_t UART_RX_IDX;    // caractere primite din linia in curs
extern volatile __bank(2) uint8_t UART_TX_LEN;    // de scris inainte de uart_send()
extern volatile __bank(2) uint8_t UART_TX_STATUS;
extern volatile __bank(2) uint8_t UART_RX_STATUS;

// Din uart_asm.s
extern void uart_init(void);
// Din ISR, cand TXIF / RCIF este setat
extern void uart_tx_isr(void);
extern void uart_rx_isr(void);
// Trimite UART_TX_LEN octeti din UART_TX_BUF
extern void uart_send(void);

// Lungimea mesajului primit (cu terminatorul) daca este complet, altfel 0
uint8_t uart_receive(void);

// Goleste bufferul de receptie; pana atunci octetii noi sunt ignorati
void uart_release(void);

// Trimite str ca o linie (maxim UART_TX_BUF_SIZE - 1 caractere + '\n');
// asteapta terminarea liniei anterioare
void uart_print_string(const char* str);

// Copiaza mesajul primit in out_buf (minim UART_RX_BUF_SIZE octeti), terminat cu '\0'.
// Intoarce numarul de caractere, 0 daca nu exista mesaj.
uint8_t uart_read_string(char *out_buf);


#endif // UART_H
