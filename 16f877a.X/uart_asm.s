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
RX_BUF_SIZE	    EQU 16
TX_BUF_SIZE	    EQU 16
;===============================================================================
; SECTIUNEA DE DATE (RAM - BANK2 - BANK3)
;===============================================================================
GLOBAL _UART_RX_BUF, _UART_RX_LEN, _UART_RX_STATUS
GLOBAL _UART_TX_BUF, _UART_TX_LEN, _UART_TX_STATUS
PSECT uart_data_bank2,global,class=BANK2,space=1,noexec
; Exportăm variabilele către C (XC8 le va căuta cu underscore în față)
; Variabile de control
_UART_RX_IDX:    DS  1
_UART_RX_STATUS: DS  1
_UART_RX_LEN:    DS  1
_UART_TX_IDX:    DS  1
_UART_TX_LEN:    DS  1
_UART_TX_STATUS: DS  1
;Buffer transmisie
_UART_TX_BUF:    DS      16

;Bank 3
PSECT uart_data_bank3,global,class=BANK3,space=1,noexec
_UART_RX_BUF:    DS      16
  
;===============================================================================
; SECTIUNE DE COD
; ADRESA START : 0x1E01
;===============================================================================
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
    BSF     TRISC, 7                 
    BSF     TRISC, 6                 
    BSF     TXSTA, TXSTA_BRGH_POSN   
    ;-------------------------------------------;
    ;Valori pentru baudrate la Fosc = 18.432 MHz;
    ; Baudrate = 18432000 / (16 * (SPBRG+1))	;
    ; Baudrate 9600   -> SPBRG  = 119 (0x77)	;
    ; Baudrate 19200  -> SPBRG  = 59  (0x3B)	;
    ; Baudrate 38400  -> SPBRG  = 29  (0x1D)	;
    ; Baudrate 57600  -> SPBRG  = 19  (0x13)	;
    ; Baudrate 115200 -> SPBRG  = 9   (0x09)	;
    ; Baudrate 230400 -> SPBRG  = 4   (0x04)	;
    ; Baudrate 384000 -> SPBRG  = 2   (0x02)	;
    ;-------------------------------------------;
    MOVLW   9                     
    MOVWF   SPBRG
    BCF     TXSTA, TXSTA_SYNC_POSN   
    
    BCF     STATUS, STATUS_RP0_POSN  
    BSF     RCSTA, RCSTA_SPEN_POSN   

; Inițializare RX
    BCF     RCSTA, RCSTA_RX9_POSN    
    BCF     RCSTA, RCSTA_CREN_POSN   
    BSF     RCSTA, RCSTA_CREN_POSN   

; Inițializare TX
    BSF     STATUS, STATUS_RP0_POSN  
    BCF     TXSTA, TXSTA_TX9_POSN    
    BSF     TXSTA, TXSTA_TXEN_POSN   

; Inițializare întreruperi
    BSF     PIE1, PIE1_RCIE_POSN     
    ;BSF     PIE1, PIE1_TXIE_POSN     
    BSF     INTCON, INTCON_PEIE_POSN 
    
    BCF     STATUS, STATUS_RP0_POSN  
    RETURN

;===============================================================================
; ISR_UART_RX
;===============================================================================
PSECT uart_code,abs, delta=2    
ORG 0x1E80
_uart_rx_isr:
    BANKSEL RCSTA
    ; 1. Verificare eroare hardware de Overrun
    BTFSC   RCSTA, RCSTA_OERR_POSN 
    GOTO    uart_rx_overrun_err&0x7FF    
    ; 1b.Verificare eroare de framing
    BTFSC   RCSTA, RCSTA_FERR_POSN 
    GOTO    uart_rx_framing_err&0x7FF      

    ; 2. Verificăm dacă bufferul anterior este necitit (Status == RX_STATE_READY)
    BANKSEL _UART_RX_STATUS
    MOVF    BANKMASK(_UART_RX_STATUS), W
    XORLW   RX_STATE_READY          
    BTFSC   STATUS, STATUS_Z_POSN   
    GOTO    uart_rx_discard&0x7FF           

    ; 3. Forțăm starea în RX_STATE_BUSY
    MOVLW   RX_STATE_BUSY           
    MOVWF   BANKMASK(_UART_RX_STATUS)     

    ; 4. Configurare adresare indirectă (Poveștile cu FSR utilizează IRP)
    ; Deoarece buffers se află în BANK 2, setăm bitul IRP din STATUS pentru selecție Bank 2/3 în mod indirect
    BSF     STATUS, STATUS_IRP_POSN 
    MOVF    BANKMASK(_UART_RX_IDX), W     
    ADDLW   LOW(_UART_RX_BUF)       
    MOVWF   FSR                     

    ; 5. Citire octet din hardware și salvare în buffer indirect
    BANKSEL RCREG
    MOVF    RCREG, W            
    MOVWF   INDF            

    ; 6. Verificare '\n' (cod 10) -> Terminare naturală
    XORLW   RX_MSG_TERMINATOR       
    BTFSC   STATUS, STATUS_Z_POSN   
    GOTO    uart_rx_handle_t&0x7FF  

    ; 7. Incrementare și verificare limită buffer (index 15)
    BANKSEL _UART_RX_IDX
    INCF    BANKMASK(_UART_RX_IDX), F
    MOVF    BANKMASK(_UART_RX_IDX), W
    SUBLW   RX_BUF_SIZE-1                      
    BTFSS   STATUS, STATUS_Z_POSN   
    RETURN                          

    ; --- Buffer plin (Am ajuns la indexul 15, forțăm '\n' pe ultima poziție) ---
    INCF    FSR, F                  
    MOVLW   RX_MSG_TERMINATOR       
    MOVWF   INDF                    
    GOTO    uart_rx_msg_done&0x7FF  

uart_rx_handle_t:           
    BANKSEL _UART_RX_IDX
    INCF    BANKMASK(_UART_RX_IDX), F         
uart_rx_msg_done:
    ; Salvare lungime finală pachet pentru C
    MOVF    BANKMASK(_UART_RX_IDX), W
    MOVWF   BANKMASK(_UART_RX_LEN)             
    
    ; Setăm Starea = RX_STATE_READY
    MOVLW   RX_STATE_READY          
    MOVWF   BANKMASK(_UART_RX_STATUS)            
    
    ; Resetăm indexul intern pentru următoarea transmisie
    CLRF    BANKMASK(_UART_RX_IDX)             
    RETURN

uart_rx_discard:
    BANKSEL RCREG
    MOVF    RCREG, W                
    RETURN

uart_rx_overrun_err:
    BANKSEL _UART_RX_IDX
    CLRF    BANKMASK(_UART_RX_IDX)    
    BANKSEL RCSTA
    BCF     RCSTA, RCSTA_CREN_POSN  
    MOVF    RCREG, W        
    MOVF    RCREG, W                
    BSF     RCSTA, RCSTA_CREN_POSN  
    
    BANKSEL _UART_RX_STATUS
    MOVLW   RX_STATE_ERROR          
    MOVWF   BANKMASK(_UART_RX_STATUS)     
    RETURN

uart_rx_framing_err:        
    BANKSEL _UART_RX_IDX
    CLRF    BANKMASK(_UART_RX_IDX)
    BANKSEL RCREG
    MOVF    RCREG, W                
    BANKSEL _UART_RX_STATUS
    MOVLW   RX_STATE_ERROR          
    MOVWF   BANKMASK(_UART_RX_STATUS)         
    RETURN

;===============================================================================
; UART_SEND - Inițiere Transmisie
;===============================================================================
_uart_send:
    ; 1. Verificăm dacă driverul este deja ocupat (Status == TX_STATE_BUSY)
    BANKSEL _UART_TX_STATUS ; BANK 2
    MOVF    BANKMASK(_UART_TX_STATUS), W
    XORLW   TX_STATE_BUSY           
    BTFSC   STATUS, STATUS_Z_POSN   
    GOTO    uart_send_err&0x7FF     

    ; 2. Verificăm dacă lungimea este validă (să nu fie 0)
    MOVF    BANKMASK(_UART_TX_LEN), F         
    BTFSC   STATUS, STATUS_Z_POSN   
    RETURN                          

    ; 3. Verificăm limita maximă a bufferului (Lungimea trebuie să fie <= 16)
    MOVF    BANKMASK(_UART_TX_LEN), W         
    SUBLW   TX_BUF_SIZE                      
    BTFSS   STATUS, STATUS_C_POSN   
    GOTO    uart_send_err&0x7FF     

    ; 4. Pregătim stările pentru transmisie
    MOVLW   TX_STATE_BUSY           
    MOVWF   BANKMASK(_UART_TX_STATUS)         
    CLRF    BANKMASK(_UART_TX_IDX)            

    ; 5. Trimitem direct primul octet (index 0)
    MOVF    BANKMASK(_UART_TX_BUF), W         
    BCF	    STATUS, STATUS_RP1_POSN ; BANK 2-> BANK 0
    MOVWF   TXREG                   

    ; 6. Inițializăm indexul direct la 1 pentru ISR
    BSF	    STATUS, STATUS_RP1_POSN ; BANK 0 -> BANK 2
    INCF    BANKMASK(_UART_TX_IDX), F         

    ; 7. Activăm întreruperea hardware TX (TXIE în PIE1 -> Bank 1)
    BANKSEL PIE1 ; BANK 2 -> BANK 1
    BSF     PIE1, PIE1_TXIE_POSN    
    RETURN

uart_send_err:
    ;BANKSEL _UART_TX_STATUS ; BANK 2 -> BANK 2
    MOVLW   TX_STATE_ERROR          
    MOVWF   BANKMASK(_UART_TX_STATUS)     
    RETURN

;===============================================================================
; ISR_UART_TX - Rutina de Întrerupere pentru Transmisie
;===============================================================================    
_uart_tx_isr:
    BANKSEL _UART_TX_STATUS ;BANK 2
    ; 1. Verificăm dacă am terminat de trimis tot mesajul
    MOVF    BANKMASK(_UART_TX_IDX), W         
    SUBWF   BANKMASK(_UART_TX_LEN), W         
    BTFSC   STATUS, STATUS_Z_POSN   
    GOTO    uart_tx_msg_done&0x7FF  

    ; 2. Configurare adresare indirectă pentru următorul octet (BANK 2/3 prin IRP=1)
    BSF     STATUS, STATUS_IRP_POSN         
    MOVF    BANKMASK(_UART_TX_IDX), W     
    ADDLW   LOW(_UART_TX_BUF)       
    MOVWF   FSR                      

    ; 3. Citim octetul din buffer și îl împingem în hardware (TXREG -> Bank 0)
    MOVF    INDF, W                 
    BCF	    STATUS, STATUS_RP1_POSN ; BANK 2 -> BANK 0
    MOVWF   TXREG                   

    ; 4. Incrementăm indexul pentru următorul caracter
    BSF	    STATUS, STATUS_RP1_POSN ; BANK 0 -> BANK 2
    INCF    BANKMASK(_UART_TX_IDX), F
    RETURN                          

uart_tx_msg_done:
    MOVLW   TX_STATE_IDLE
    MOVWF   BANKMASK(_UART_TX_STATUS)

    ; Oprim întreruperile TX (PIE1 se află în Bank 1)
    BANKSEL PIE1 ; BANK 2 -> BANK 1
    BCF     PIE1, PIE1_TXIE_POSN
    RETURN
END