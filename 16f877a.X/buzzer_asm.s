#include <xc.inc>
GLOBAL _buzzer_init, _buzzer_on, _buzzer_off
GLOBAL _buzzer_get_note_value



GLOBAL _temp_var    
; Variabila temporara (nu se foloseste in intreruperi)
; definita in buzzer.c
; Buzzer: CCP1 in mod PWM, perioada din Timer2

    
PSECT buzzer, abs, ovrld,class = CODE, delta=2
ORG 0x1F80
_buzzer_init:
    BSF     STATUS, STATUS_RP0_POSN     ; Banc 1
    BCF     STATUS, STATUS_RP1_POSN
    BCF     TRISC, 2                    ; RC2 = iesirea CCP1 (TRIS: 1 = intrare, 0 = iesire)
    MOVLW   0xB7
    MOVWF   PR2                         ; PR2: perioada PWM (Timer2 numara 0..PR2)
    BCF     STATUS, STATUS_RP0_POSN     ; Banc 0

    MOVLW   0x0C
    MOVWF   CCP1CON                     ; CCP1CON: CCP1M = modul (captura, comparare, PWM), DC1B = bitii de jos ai factorului de umplere
    
    CLRF    CCPR1L                      ; CCPR1L: bitii de sus ai factorului de umplere
    
    MOVLW   0b00000110
    MOVWF   T2CON                       ; T2CON: TMR2ON, prescaler (1, 4, 16), postscaler (1..16)

    RETURN

_buzzer_on:
    BANKSEL CCP1CON
    MOVLW   0x0C                        ; CCP1CON<3:0> = CCP1M, modul CCP1
    IORWF   CCP1CON,F                   ; DC1B<5:4> raman neschimbati
    RETURN

_buzzer_off:
    BANKSEL CCP1CON
    MOVLW   0x30                        ; CCP1CON<3:0> = CCP1M (modul), <5:4> = DC1B
    ANDWF   CCP1CON,F
    BCF     PORTC, 2                    ; pinul la 0: tranzistorul blocat
    RETURN

_buzzer_get_note_value:
    BANKSEL _temp_var			
    MOVWF   BANKMASK(_temp_var)		; indexul notei
    MOVLW   HIGH(_get_note_pr2)		; pagina tabelului
    MOVWF   PCLATH
    MOVF    BANKMASK(_temp_var), W
    CALL    _get_note_pr2&0x7FF
    RETURN

; Tabelul PR2 al notelor: PR2 = Fosc / (4 x 16 x f) - 1 = 288000 / f - 1
; (prescaler 1:16). PR2 <= 255, deci f >= ~1125 Hz.
_get_note_pr2:
    ADDWF   PCL, F                      ; salt indexat
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
    