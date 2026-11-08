#include <xc.inc>
SPI_BUF_SIZE EQU 3



; Exportăm funcțiile și variabila buffer către mediul C
GLOBAL _spi_init, _spi_transfer_packet
GLOBAL _spi_buf

;===============================================================================
; SECTIUNEA DE DATE (RAM - ALOCATĂ ÎN BANC 3)
;===============================================================================
PSECT spi_data_bank3,global,class=BANK3,space=1,noexec

_spi_buf:       DS SPI_BUF_SIZE    ; Buffer de SPI_BUF_SIZE octeți, alocat în Bancul 3
_spi_counter:   DS 1		   ; Contor intern mutat tot în Bancul 3
_spi_delay_counter: DS 1
;===============================================================================
; SECTIUNEA DE COD
;===============================================================================
PSECT spi_code,abs, ovrld, class=CODE, delta=2
ORG 0x1FBA
;-------------------------------------------------------------------------------
; INIȚIALIZARE SPI HARDWARE (MASTER MODE, FOSC/16)
;-------------------------------------------------------------------------------
_spi_init:
    BANKSEL TRISC
    BCF     TRISC, 3                    ; RC3/SCK = Ieșire (Master Ceas)
    BSF     TRISC, 4                    ; RC4/SDI = Intrare (Date de la Slave)
    BCF     TRISC, 5                    ; RC5/DO  = Ieșire (Date către Slave)

    ; Configurare SSPSTAT (Banc 1)
    CLRF    SSPCON
    BANKSEL SSPSTAT
    MOVLW   0b01000000                  ; SMP=0, CKE=1
    MOVWF   SSPSTAT
    
    ; Activăm modulul hardware în Banc 0
    BANKSEL SSPCON
    MOVLW   0b00100000                  ; SSPEN=1, CKP=0, Master Fosc/4
    MOVWF   SSPCON
    RETURN

;-------------------------------------------------------------------------------
; TRANSFER PACHET - blocant pana la trimiterea intregului pachet
;-------------------------------------------------------------------------------
_spi_transfer_packet:
    ; 1. CRUCIAL: Setăm IRP = 1 pentru ca INDF să poată accesa Bancurile 2 și 3
    BSF     STATUS, STATUS_IRP_POSN   
    ; 2. Mergem în Bancul 3 pentru a configura contorul de buclă
    BANKSEL _spi_counter
    MOVLW   SPI_BUF_SIZE        ; Resetăm bucla la dimensiunea bufferului
    MOVWF   BANKMASK(_spi_counter)          
    ; 3. Încărcăm adresa de start a bufferului în FSR
    ; FSR preia doar cei 8 biți de jos ai adresei, bitul 9 fiind mapat prin IRP=1
    MOVLW   _spi_buf & 0xFF             
    MOVWF   FSR                         

spi_loop:
    ; Pasul 1: Citim octetul curent din buffer prin adresare indirectă.
    MOVF    INDF, W                     ; W = _spi_buf[FSR]
    ; Pasul 2: Scriem în SSPBUF (aflat în Bancul 0)
    BANKSEL SSPBUF              
    MOVWF   SSPBUF                      

spi_wait_bf:
    ; Pasul 3: Verificăm flag-ul BF în SSPSTAT (aflat în Bancul 1)
    BANKSEL SSPSTAT
    BTFSS   SSPSTAT, SSPSTAT_BF_POSN    ; Așteptăm până când transmisia/recepția s-a terminat
    GOTO    spi_wait_bf&0x7FF                 
    
    ; Pasul 4: Citim rezultatul venit de la sclav din SSPBUF (Bancul 0)
    BANKSEL SSPBUF
    MOVF    SSPBUF, W                    
    
    ; Pasul 5: Salvăm datele recepționate înapoi în buffer-ul din Bancul 3
    ; Registrul INDF ignoră complet faptul că tocmai am fost în Bancul 0, folosind IRP=1
    MOVWF   INDF                        
    
    ; Pasul 6: Incrementăm pointerul FSR pentru următorul element din masiv
    INCF    FSR, F                      
    ; Decrementăm contorul înainte de a introduce delay-ul. Astfel, nu
    ; întârziem după ultimul octet, înaintea următorului pachet.
    BANKSEL _spi_counter
    DECFSZ  BANKMASK(_spi_counter), F
    GOTO    spi_delay
    RETURN

spi_delay: ;Delay intre octeti pentru a da timp de raspuns
    BANKSEL _spi_delay_counter
    MOVLW   0x2F
    MOVWF   BANKMASK(_spi_delay_counter)
spi_delay_inner:
    DECFSZ  BANKMASK(_spi_delay_counter), F
    GOTO    spi_delay_inner&0x7FF
    ; Continuăm cu următorul octet după delay.
    GOTO    spi_loop&0x7FF

END