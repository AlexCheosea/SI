#include <xc.inc>
GLOBAL _buzzer_init, _buzzer_on, _buzzer_off
GLOBAL _buzzer_get_note_value



GLOBAL _temp_var    
PSECT buzzer_data,global,class=BANK3,space=1,noexec
; Variabila temporara refolosita (a nu se refolosi in intreruperi)
_temp_var:    DS  1    
    
;===============================================================================
; SECTIUNEA DE COD - Modul CCP1 in mod PWM + Timer2 pe 8 biti
;===============================================================================

    
PSECT buzzer, abs, ovrld,class = CODE, delta=2
ORG 0x1F80
_buzzer_init:
    BSF     STATUS, STATUS_RP0_POSN     ; Selectam Banc 1
    BCF     STATUS, STATUS_RP1_POSN
    BCF     TRISC, 2                    ; Setam RC2 (CCP1) ca IEȘIRE
    MOVLW   0xB7			;Frecventa = 1568 Hz
    MOVWF   PR2
    BCF     STATUS, STATUS_RP0_POSN     ; Revenim in Banc 0

    MOVLW   0x0C                        ; Setam modul PWM in CCP1CON
    MOVWF   CCP1CON
    
    CLRF    CCPR1L                      ; Initializam cu Duty Cycle 0 (Mute)
    
    MOVLW   0b00000110                  ; Timer2 ON, Prescaler 1:16
    MOVWF   T2CON

    RETURN

_buzzer_on:
    BANKSEL CCP1CON
    MOVLW   0x0C
    IORWF   CCP1CON,F
    MOVWF   CCP1CON
    RETURN

_buzzer_off:
    BANKSEL CCP1CON
    MOVLW   0x30
    ANDWF   CCP1CON,F
    BCF     PORTC, 2                    ; Siguranță tranzistor OFF
    RETURN

_buzzer_get_note_value:
    BANKSEL _temp_var			
    MOVWF   BANKMASK(_temp_var)		; Salvam indexul
    MOVLW   HIGH(_get_note_pr2)		; Pregatim saltul
    MOVWF   PCLATH
    MOVF    BANKMASK(_temp_var), W	; Citim indexul
    CALL    _get_note_pr2&0x7FF
    RETURN

;-------------------------------------------------------------------------------
; TABELA DE LOOKUP PENTRU REGISTRUL PR2 (8-biti) 
; Calculata cu formula: PR2 = (288000 / Frecventa) - 1
; Incepe de la RE6 (1175 Hz), limitata inferior de pragul de 1125 Hz al Timer2
;-------------------------------------------------------------------------------
_get_note_pr2:
    ADDWF   PCL, F                      ; Jump indexat
    RETLW   0xF4                        ; 0:  RE6   (1175 Hz)
    RETLW   0xE6                        ; 1:  RE#6  (1245 Hz)
    RETLW   0xD9                        ; 2:  MI6   (1319 Hz)
    RETLW   0xCD                        ; 3:  FA6   (1397 Hz)
    RETLW   0xC2                        ; 4:  FA#6  (1480 Hz)
    RETLW   0xB7                        ; 5:  SOL6  (1568 Hz)
    RETLW   0xAC                        ; 6:  SOL#6 (1661 Hz)
    RETLW   0xA3                        ; 7:  LA6   (1760 Hz)
    RETLW   0x99                        ; 8:  LA#6  (1865 Hz)
    RETLW   0x91                        ; 9:  SI6   (1976 Hz)
    RETLW   0x89                        ; 10: DO7   (2093 Hz)
    RETLW   0x81                        ; 11: DO#7  (2217 Hz)
    RETLW   0x7A                        ; 12: RE7   (2349 Hz)
    RETLW   0x73                        ; 13: RE#7  (2489 Hz)
    RETLW   0x6C                        ; 14: MI7   (2637 Hz)
    RETLW   0x66                        ; 15: FA7   (2794 Hz)
    RETLW   0x60                        ; 16: FA#7  (2960 Hz)
    RETLW   0x5B                        ; 17: SOL7  (3136 Hz)
    RETLW   0x56                        ; 18: SOL#7 (3322 Hz)
    RETLW   0x51                        ; 19: LA7   (3520 Hz)
    RETLW   0x4C                        ; 20: LA#7  (3729 Hz)
    RETLW   0x48                        ; 21: SI7   (3951 Hz)
    RETLW   0x44                        ; 22: DO8   (4186 Hz)
    RETLW   0x40                        ; 23: DO#8  (4435 Hz)
    RETLW   0x3C                        ; 24: RE8   (4699 Hz)

END
    