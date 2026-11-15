#include <xc.inc>
SPI_BUF_SIZE EQU 3



GLOBAL _spi_init, _spi_transfer_packet
GLOBAL _spi_buf, _spi_counter, _spi_delay_counter

; Date in Bancul 3
; Date in Bancul 3, definite in init.c
PSECT spi_code,abs, ovrld, class=CODE, delta=2
ORG 0x1FBA
; SPI hardware, master
_spi_init:
    BANKSEL TRISC
    ; TRIS: 1 = intrare, 0 = iesire
    BCF     TRISC, 3                    ; RC3 = SCK
    BSF     TRISC, 4                    ; RC4 = SDI
    BCF     TRISC, 5                    ; RC5 = SDO

    MOVLW   0b01000000
    MOVWF   SSPSTAT                     ; SSPSTAT: SMP = momentul esantionarii, CKE = frontul de ceas al transmisiei
    
    BANKSEL SSPCON
    MOVLW   0b00100000
    MOVWF   SSPCON                      ; SSPCON: SSPEN = pornit, CKP = nivelul ceasului in repaus, SSPM = mod si ceas (Fosc/4..64, slave)
    RETURN

; Trimite si primeste _spi_buf (blocant)
_spi_transfer_packet:
    BSF     STATUS, STATUS_IRP_POSN     ; IRP = 1: INDF adreseaza Bancurile 2 si 3
    BANKSEL _spi_counter
    MOVLW   SPI_BUF_SIZE
    MOVWF   BANKMASK(_spi_counter)          
    MOVLW   _spi_buf & 0xFF             ; FSR = 8 biti de jos; bitul 9 vine din IRP
    MOVWF   FSR                         

spi_loop:
    MOVF    INDF, W                     ; octetul de trimis
    BANKSEL SSPBUF              
    MOVWF   SSPBUF                      

spi_wait_bf:
    BANKSEL SSPSTAT
    BTFSS   SSPSTAT, SSPSTAT_BF_POSN    ; BF = 1: octet transferat
    GOTO    spi_wait_bf&0x7FF                 
    
    BANKSEL SSPBUF
    MOVF    SSPBUF, W                    
    
    MOVWF   INDF                        ; octetul primit inlocuieste pe cel trimis
    
    INCF    FSR, F                      
    ; Fara pauza dupa ultimul octet
    BANKSEL _spi_counter
    DECFSZ  BANKMASK(_spi_counter), F
    GOTO    spi_delay
    RETURN

spi_delay: ; pauza intre octeti, pentru raspunsul sclavului (Bancul 3 selectat deja)
    ; ~3 Tcy pe iteratie: 0x7F -> ~380 Tcy = ~83 us la 18.432 MHz. Sclavul
    ; (8 MHz, 2 MIPS) reincarca SSPBUF in ISR, posibil dupa un pas de motor
    ; (contorul de pozitie pe 32 de biti): ~90 Tcy ai sclavului = ~45 us.
    MOVLW   0x9F
    BANKSEL _spi_delay_counter
    MOVWF   BANKMASK(_spi_delay_counter)
spi_delay_inner:
    DECFSZ  BANKMASK(_spi_delay_counter), F
    GOTO    spi_delay_inner&0x7FF
    GOTO    spi_loop&0x7FF

END