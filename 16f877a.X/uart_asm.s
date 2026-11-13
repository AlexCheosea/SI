#include <xc.inc>
RX_STATE_IDLE       EQU 0
RX_STATE_BUSY       EQU 1
RX_STATE_READY      EQU 2
RX_STATE_ERROR      EQU 3
RX_MSG_TERMINATOR   EQU 10
TX_MSG_TERMINATOR   EQU 10
TX_STATE_IDLE       EQU 0
TX_STATE_BUSY       EQU 1
TX_STATE_ERROR      EQU 3
RX_BUF_SIZE	    EQU 16           ; UART_RX_BUF_SIZE din uart.h
TX_BUF_SIZE	    EQU 32           ; UART_TX_BUF_SIZE din uart.h
; Date: control + TX in Bancul 2, RX in Bancul 3
GLOBAL _UART_RX_BUF, _UART_RX_LEN, _UART_RX_STATUS
GLOBAL _UART_TX_BUF, _UART_TX_LEN, _UART_TX_STATUS
; Variabilele sunt definite in uart.c (__bank(2) / __bank(3))
GLOBAL _UART_RX_IDX, _UART_TX_IDX

  
GLOBAL _uart_init, _uart_tx_isr, _uart_rx_isr, _uart_send
PSECT uart_init,abs, ovrld, class=CODE, delta=2
ORG 0x1E01

_uart_init:
    BANKSEL _UART_TX_STATUS
    CLRF    BANKMASK(_UART_TX_STATUS)
    CLRF    BANKMASK(_UART_RX_STATUS)
    CLRF    BANKMASK(_UART_TX_IDX)
    CLRF    BANKMASK(_UART_TX_LEN)
    BCF     INTCON, INTCON_GIE_POSN  
    BSF     STATUS, STATUS_RP0_POSN  
    BCF     STATUS, STATUS_RP1_POSN
    BSF     TRISC, 7                 ; RC7 = RX, RC6 = TX: USART cere TRIS = 1
    BSF     TRISC, 6                 
    BSF     TXSTA, TXSTA_BRGH_POSN   ; BRGH: generatorul de baud, 1 = viteza mare, 0 = mica
    MOVLW   2
    MOVWF   SPBRG                    ; SPBRG: baud = Fosc / (16 x (SPBRG + 1)) cu BRGH = 1 (x64 cu BRGH = 0)
    BCF     TXSTA, TXSTA_SYNC_POSN   ; SYNC: 0 = asincron, 1 = sincron
    
    BCF     STATUS, STATUS_RP0_POSN  
    BSF     RCSTA, RCSTA_SPEN_POSN   ; SPEN: portul serial (RC6 / RC7) pornit / oprit

    ; Receptie
    BCF     RCSTA, RCSTA_RX9_POSN    ; RX9: 1 = 9 biti, 0 = 8 biti
    BCF     RCSTA, RCSTA_CREN_POSN   ; CREN: receptie continua; 0 -> 1 sterge OERR
    BSF     RCSTA, RCSTA_CREN_POSN   

    ; Transmisie
    BSF     STATUS, STATUS_RP0_POSN  
    BCF     TXSTA, TXSTA_TX9_POSN    ; TX9: 1 = 9 biti, 0 = 8 biti
    BSF     TXSTA, TXSTA_TXEN_POSN   ; TXEN: transmisia pornita / oprita

    ; Intreruperi (TXIE se activeaza in uart_send)
    BSF     PIE1, PIE1_RCIE_POSN     
    BSF     INTCON, INTCON_PEIE_POSN
    RETURN

; ISR receptie
PSECT uart_code,abs, delta=2    
ORG 0x1E80
_uart_rx_isr:
    BANKSEL RCSTA
    BTFSC   RCSTA, RCSTA_OERR_POSN  ; overrun
    GOTO    uart_rx_overrun_err&0x7FF    
    BTFSC   RCSTA, RCSTA_FERR_POSN  ; framing
    GOTO    uart_rx_framing_err&0x7FF      

    ; Mesajul anterior inca necitit: octetul se arunca
    BANKSEL _UART_RX_STATUS
    MOVF    BANKMASK(_UART_RX_STATUS), W
    XORLW   RX_STATE_READY          
    BTFSC   STATUS, STATUS_Z_POSN   
    GOTO    uart_rx_discard&0x7FF           

    MOVLW   RX_STATE_BUSY           
    MOVWF   BANKMASK(_UART_RX_STATUS)     

    BSF     STATUS, STATUS_IRP_POSN ; IRP = 1: INDF adreseaza Bancurile 2 si 3
    MOVF    BANKMASK(_UART_RX_IDX), W     
    ADDLW   LOW(_UART_RX_BUF)       
    MOVWF   FSR                     

    BANKSEL RCREG
    MOVF    RCREG, W            
    MOVWF   INDF            

    ; Terminatorul incheie mesajul
    XORLW   RX_MSG_TERMINATOR       
    BTFSC   STATUS, STATUS_Z_POSN   
    GOTO    uart_rx_handle_t&0x7FF  

    ; Limita bufferului
    BANKSEL _UART_RX_IDX
    INCF    BANKMASK(_UART_RX_IDX), F
    MOVF    BANKMASK(_UART_RX_IDX), W
    SUBLW   RX_BUF_SIZE-1                      
    BTFSS   STATUS, STATUS_Z_POSN   
    RETURN                          

    ; Buffer plin: terminator fortat pe ultima pozitie
    INCF    FSR, F                  
    MOVLW   RX_MSG_TERMINATOR       
    MOVWF   INDF                    
    GOTO    uart_rx_msg_done&0x7FF  

uart_rx_handle_t:           
    BANKSEL _UART_RX_IDX
    INCF    BANKMASK(_UART_RX_IDX), F         
uart_rx_msg_done:
    MOVF    BANKMASK(_UART_RX_IDX), W
    MOVWF   BANKMASK(_UART_RX_LEN)             
    
    MOVLW   RX_STATE_READY          
    MOVWF   BANKMASK(_UART_RX_STATUS)            
    
    CLRF    BANKMASK(_UART_RX_IDX)             
    RETURN

uart_rx_discard:
    BANKSEL RCREG
    MOVF    RCREG, W                
    RETURN

uart_rx_overrun_err:                ; Bancul 0 selectat deja (RCSTA)
    BCF     RCSTA, RCSTA_CREN_POSN
    MOVF    RCREG, W
    MOVF    RCREG, W
    BSF     RCSTA, RCSTA_CREN_POSN

    BANKSEL _UART_RX_IDX
    CLRF    BANKMASK(_UART_RX_IDX)
    MOVLW   RX_STATE_ERROR
    MOVWF   BANKMASK(_UART_RX_STATUS)
    RETURN

uart_rx_framing_err:                ; Bancul 0 selectat deja (RCSTA)
    MOVF    RCREG, W
    BANKSEL _UART_RX_IDX
    CLRF    BANKMASK(_UART_RX_IDX)
    MOVLW   RX_STATE_ERROR
    MOVWF   BANKMASK(_UART_RX_STATUS)
    RETURN

; Porneste transmisia a UART_TX_LEN octeti din UART_TX_BUF
_uart_send:
    BANKSEL _UART_TX_STATUS ; transmisie in curs -> eroare
    MOVF    BANKMASK(_UART_TX_STATUS), W
    XORLW   TX_STATE_BUSY           
    BTFSC   STATUS, STATUS_Z_POSN   
    GOTO    uart_send_err&0x7FF     

    ; Lungime 0: nimic de trimis
    MOVF    BANKMASK(_UART_TX_LEN), F         
    BTFSC   STATUS, STATUS_Z_POSN   
    RETURN                          

    ; Lungime > TX_BUF_SIZE -> eroare
    MOVF    BANKMASK(_UART_TX_LEN), W         
    SUBLW   TX_BUF_SIZE                      
    BTFSS   STATUS, STATUS_C_POSN   
    GOTO    uart_send_err&0x7FF     

    MOVLW   TX_STATE_BUSY           
    MOVWF   BANKMASK(_UART_TX_STATUS)         
    CLRF    BANKMASK(_UART_TX_IDX)            

    ; Primul octet direct, restul din ISR
    MOVF    BANKMASK(_UART_TX_BUF), W         
    BCF	    STATUS, STATUS_RP1_POSN ; BANK 2-> BANK 0
    MOVWF   TXREG                   

    BSF	    STATUS, STATUS_RP1_POSN ; BANK 0 -> BANK 2
    INCF    BANKMASK(_UART_TX_IDX), F         

    BANKSEL PIE1
    BSF     PIE1, PIE1_TXIE_POSN    
    RETURN

uart_send_err:
    MOVLW   TX_STATE_ERROR          
    MOVWF   BANKMASK(_UART_TX_STATUS)     
    RETURN

; ISR transmisie
_uart_tx_isr:
    BANKSEL _UART_TX_STATUS
    ; Mesaj terminat?
    MOVF    BANKMASK(_UART_TX_IDX), W         
    SUBWF   BANKMASK(_UART_TX_LEN), W         
    BTFSC   STATUS, STATUS_Z_POSN   
    GOTO    uart_tx_msg_done&0x7FF  

    BSF     STATUS, STATUS_IRP_POSN ; IRP = 1: INDF adreseaza Bancurile 2 si 3
    MOVF    BANKMASK(_UART_TX_IDX), W     
    ADDLW   LOW(_UART_TX_BUF)       
    MOVWF   FSR                      

    MOVF    INDF, W                 
    BCF	    STATUS, STATUS_RP1_POSN ; BANK 2 -> BANK 0
    MOVWF   TXREG                   

    BSF	    STATUS, STATUS_RP1_POSN ; BANK 0 -> BANK 2
    INCF    BANKMASK(_UART_TX_IDX), F
    RETURN                          

uart_tx_msg_done:
    MOVLW   TX_STATE_IDLE
    MOVWF   BANKMASK(_UART_TX_STATUS)

    BANKSEL PIE1
    BCF     PIE1, PIE1_TXIE_POSN
    RETURN
END