#include <xc.inc>
SEG_EN          EQU 7	  
BAR_GRN         EQU 1
BAR_RED         EQU 0

;===============================================================================
; SECTIUNEA DE DATE (RAM - MUTATĂ INTEGRAL ÎN BANK 3: 0x190 - 0x1EF)
;===============================================================================
PSECT udata_bank3,global,class=BANK3,space=1,noexec

GLOBAL _bar_red_low, _bar_red_high, _bar_grn_high, _bar_grn_low  
GLOBAL _dp_pos
GLOBAL _disp_buf
GLOBAL _btn_i0event
_seg_index:     DS 1 
_disp_cnt:      DS 1 
_bar_red_low:   DS 1 
_bar_red_high:  DS 1
_bar_grn_low:   DS 1      
_bar_grn_high:  DS 1 
_dp_pos:	    DS 1     ; Locatie digit point
    
_disp_buf:       DS 8   ; Buffer mutat în Bank 3
_btn_i0state:    DS 1   
_btn_i0prev:     DS 1   
_btn_i0event:    DS 1   

    
;===============================================================================
; DISPLAY DRIVER CODE
;===============================================================================
PSECT hmi_code,abs, ovrld, class=CODE, delta=2
ORG 0x1D00

GLOBAL _hmi_init, _hmi_update


_hmi_init: 
    BANKSEL TRISD               ; Regiștrii TRIS sunt în Bank 1
    MOVLW   0x00                     
    MOVWF   TRISD               ;PORT D iesire      
    MOVWF   TRISB               ;PORT B iesire      
    MOVWF   TRISE               ;PORT E iesire      
    MOVLW   0b00000110          ;Toate porturile digitale      
    MOVWF   ADCON1              ; ADCON1 este tot în Bank 1
    MOVLW   0b00010000          ;RA4 input      
    IORWF   TRISA, F		;RA4 input
    BCF     TRISC, 0                  
    BCF     TRISC, 1 
    ;BANKSEL CMCON		;tot banc 1
    MOVLW   0x07
    MOVWF   CMCON
    ; Trecem în Bank 3 pentru inițializarea variabilelor din RAM
    BANKSEL _btn_i0state
    CLRF    BANKMASK(_btn_i0state)
    CLRF    BANKMASK(_btn_i0prev)
    CLRF    BANKMASK(_btn_i0event)
    MOVLW   9                         
    MOVWF   BANKMASK(_disp_cnt)
    RETURN   

display_segment:
    MOVLW   HIGH(font_table_low)
    MOVWF   PCLATH
    BANKSEL PORTC               ; Porturile fizice sunt în Bank 0
    BCF     PORTC, BAR_RED 
    BCF     PORTC, BAR_GRN            
    
    BANKSEL _seg_index          ; Citim din Bank 3
    MOVF    BANKMASK(_seg_index), W         
    CALL    font_table_low&0x7FF; Tabela ASCII->SEG                
    
    BANKSEL PORTD               ; Afișăm în Bank 0
    MOVWF   PORTD
    
    MOVLW   HIGH(font_table_high)
    MOVWF   PCLATH
    
    BANKSEL _seg_index          ; Citim din Bank 3
    MOVF    BANKMASK(_seg_index), W            
    CALL    font_table_high&0x7FF; Tabela ASCII->SEG                
    
    BANKSEL PORTB               ; Afișăm în Bank 0
    MOVWF   PORTB
    ;Adauga digit point peste un carater arbitrar
    BANKSEL _dp_pos
    MOVF    BANKMASK(_dp_pos), W
    XORWF   BANKMASK(_disp_cnt), W
    BTFSS   STATUS, STATUS_Z_POSN
    GOTO    dp_off
    BANKSEL PORTB
    BCF     PORTB, 6
dp_off:

; Scanare butoane (Variabile în Bank 3, Porturi în Bank 0) ---------------------
    BANKSEL _disp_cnt           ; Suntem în Bank 3
    MOVLW   HIGH(mask_pressed)
    MOVWF   PCLATH
    MOVF    BANKMASK(_disp_cnt), W
    CALL    mask_pressed&0x7FF
    
    BANKSEL PORTA               ; Mergem în Bank 0 să citim pinul fizic
    BTFSS   PORTA, 4                
    GOTO    not_pressed_I0&0x7FF
pressed_I0: 
    BANKSEL _btn_i0state        ; Înapoi în Bank 3 pentru salvare date
    IORWF   BANKMASK(_btn_i0state), F          
    GOTO    event_I0&0x7FF
not_pressed_I0:
    BANKSEL _btn_i0state        ; Înapoi în Bank 3
    XORLW   0xFF                    
    ANDWF   BANKMASK(_btn_i0state), F          
event_I0: 
    ; Rămânem în Bank 3 (toate variabilele de mai jos sunt în aceeași bancă)
    COMF    BANKMASK(_btn_i0prev), W
    ANDWF   BANKMASK(_btn_i0state), W
    IORWF   BANKMASK(_btn_i0event), F          
    MOVF    BANKMASK(_btn_i0state), W          
    MOVWF   BANKMASK(_btn_i0prev)

    GOTO    display_end&0x7FF

mask_pressed:
    ADDWF   PCL, F
    RETLW   0b00000001    ; 0
    RETLW   0b00000010    ; 1
    RETLW   0b00000100    ; 2
    RETLW   0b00001000    ; 3
    RETLW   0b00010000    ; 4
    RETLW   0b00100000    ; 5
    RETLW   0b01000000    ; 6
    RETLW   0b10000000    ; 7

display_bar_red:
    BANKSEL PORTB               ; Pini (Bank 0)
    BCF     PORTB, SEG_EN             
    BCF     PORTC, BAR_GRN          
    
    BANKSEL _bar_red_low        ; Date (Bank 3)
    MOVF    BANKMASK(_bar_red_low), W          
    BANKSEL PORTD               ; Pini (Bank 0)
    MOVWF   PORTD 
    
    BANKSEL _bar_red_high       ; Date (Bank 3)
    MOVF    BANKMASK(_bar_red_high), W         
    BANKSEL PORTB               ; Pini (Bank 0)
    MOVWF   PORTB 
    BSF     PORTC, BAR_RED          
    GOTO    display_end&0x7FF

display_bar_green:
    BANKSEL PORTB               ; Pini (Bank 0)
    BCF     PORTB, SEG_EN            
    BCF     PORTC, BAR_RED          
    
    BANKSEL _bar_grn_low        ; Date (Bank 3)
    MOVF    BANKMASK(_bar_grn_low), W          
    BANKSEL PORTD               ; Pini (Bank 0)
    MOVWF   PORTD 
    
    BANKSEL _bar_grn_high       ; Date (Bank 3)
    MOVF    BANKMASK(_bar_grn_high), W         
    BANKSEL PORTB               ; Pini (Bank 0)
    MOVWF   PORTB 
    BSF     PORTC, BAR_GRN          
    GOTO    display_end&0x7FF

_hmi_update:
    BANKSEL _disp_cnt        ;Ne asiguram ca suntem in Bank 3   
    MOVF    BANKMASK(_disp_cnt), W
    XORLW   8
    BTFSC   STATUS, STATUS_Z_POSN
    GOTO    display_bar_green&0x7FF

    MOVF    BANKMASK(_disp_cnt), W
    XORLW   9
    BTFSC   STATUS, STATUS_Z_POSN
    GOTO    display_bar_red&0x7FF

    ; cnt = 0..7
    MOVF    BANKMASK(_disp_cnt), W
    BANKSEL PORTE               ; PORTE este în Bank 0
    BCF	    PORTB, SEG_EN	; Disable la display cand schimbam digit
    MOVWF   PORTE                                             
    
    ; =========================================================================
    ; ADRESARE INDIRECTĂ PENTRU PIC16F877A (BANK 3)
    ; Deoarece _disp_buf este în Bank 3 (zona 0x180 - 0x1FF), pointerul hardware 
    ; FSR (care are doar 8 biți) are nevoie de bitul IRP din STATUS setat pe 1.
    ; =========================================================================
    BSF     STATUS, STATUS_IRP_POSN   ; IRP = 1 -> INDF va accesa Bank 2 și Bank 3!
    
    BANKSEL _disp_cnt                 ; Trecem în Bank 3 ca să citim contorul
    MOVF    BANKMASK(_disp_cnt), W
    ADDLW   LOW(_disp_buf)            ; Adunăm offset-ul la adresa de bază a buffer-ului
    MOVWF   FSR                       ; Încărcăm pointerul în FSR

    MOVF    INDF, W                   ; Hardware-ul citește acum direct din Bank 3!
    MOVWF   BANKMASK(_seg_index)      ; Salvăm în variabila din Bank 3
    
    BCF     STATUS, STATUS_IRP_POSN   ; Resetăm IRP = 0 (Opțional, dar recomandat ca bune practici)
    GOTO    display_segment&0x7FF

display_end:  
    BANKSEL _disp_cnt               ; Control contor în Bank 3
    MOVF    BANKMASK(_disp_cnt), F  ;Plimbam _disp_cnt pentru a altera Z
    BTFSC   STATUS, STATUS_Z_POSN   
    GOTO    reload_counter&0x7FF
    DECF    BANKMASK(_disp_cnt), F              
    RETURN
reload_counter:
    MOVLW   9
    MOVWF   BANKMASK(_disp_cnt)
    RETURN

;===============================================================================
; TABELA PORTD - LOW Aliniata la pagina
;===============================================================================
PSECT font_low,abs,delta=2 ;plasam tabela in ultima pagina de memorie
ORG 0x1F00
font_table_low:
    ADDWF PCL,F
ORG 0x1F21
    RETLW 0b00000000 ; (space)
    RETLW 0b00000110 ; EXCLAM
    RETLW 0b01000010 ; QUOTE
    RETLW 0b01001110 ; HASH
    RETLW 0b01101101 ; DOLLAR
    RETLW 0b11100100 ; PERCENT
    RETLW 0b01011001 ; AMP
    RETLW 0b01000000 ; APOSTROPHE
    RETLW 0b10000000 ; LPAREN
    RETLW 0b00000000 ; RPAREN
    RETLW 0b11000000 ; STAR
    RETLW 0b01000000 ; PLUS
    RETLW 0b00000000 ; COMMA
    RETLW 0b00000000 ; MINUS
    RETLW 0b00000000 ; DOT
    RETLW 0b10000000 ; SLASH
    RETLW 0b10111111 ; 0
    RETLW 0b10000110 ; 1
    RETLW 0b00011011 ; 2
    RETLW 0b00001111 ; 3
    RETLW 0b00100110 ; 4
    RETLW 0b00101001 ; 5
    RETLW 0b00111101 ; 6
    RETLW 0b00000111 ; 7
    RETLW 0b00111111 ; 8
    RETLW 0b00101111 ; 9
    RETLW 0b01000000 ; COLON
    RETLW 0b01000000 ; SEMICOLON
    RETLW 0b10000000 ; LT
    RETLW 0b00001000 ; EQUAL
    RETLW 0b00000000 ; GT
    RETLW 0b00000011 ; QUESTION
    RETLW 0b01111011 ; AT
    RETLW 0b00110111 ; A
    RETLW 0b01001111 ; B
    RETLW 0b00111001 ; C
    RETLW 0b01001111 ; D
    RETLW 0b00111001 ; E
    RETLW 0b00110001 ; F
    RETLW 0b00111101 ; G
    RETLW 0b00110110 ; H
    RETLW 0b01001001 ; I
    RETLW 0b00011110 ; J
    RETLW 0b10110000 ; K
    RETLW 0b00111000 ; L
    RETLW 0b10110110 ; M
    RETLW 0b00110110 ; N
    RETLW 0b00111111 ; O
    RETLW 0b00110011 ; P
    RETLW 0b00111111 ; Q
    RETLW 0b00110011 ; R
    RETLW 0b00101101 ; S
    RETLW 0b01000001 ; T
    RETLW 0b00111110 ; U
    RETLW 0b10110000 ; V
    RETLW 0b00110110 ; W
    RETLW 0b10000000 ; X
    RETLW 0b00101110 ; Y
    RETLW 0b10001001 ; Z
    RETLW 0b00111001 ; LBRACKET
    RETLW 0b00000000 ; BACKSLASH
    RETLW 0b00001111 ; RBRACKET
    RETLW 0b00000000 ; CARET
    RETLW 0b00001000 ; UNDERSCORE
    RETLW 0b00000000 ; BACKTICK
    RETLW 0b00011000 ; a
    RETLW 0b00111000 ; b
    RETLW 0b00011000 ; c
    RETLW 0b00001110 ; d
    RETLW 0b00011000 ; e
    RETLW 0b10000000 ; f
    RETLW 0b10001110 ; g
    RETLW 0b00110000 ; h
    RETLW 0b00000000 ; i
    RETLW 0b01010000 ; j
    RETLW 0b11000000 ; k
    RETLW 0b00110000 ; l
    RETLW 0b00010100 ; m
    RETLW 0b00010000 ; n
    RETLW 0b00011100 ; o
    RETLW 0b00110000 ; p
    RETLW 0b10000110 ; q
    RETLW 0b00010000 ; r
    RETLW 0b00001000 ; s
    RETLW 0b00111000 ; t
    RETLW 0b00011100 ; u
    RETLW 0b00010000 ; v
    RETLW 0b00010100 ; w
    RETLW 0b10000000 ; x
    RETLW 0b01001110 ; y
    RETLW 0b00001000 ; z
    RETLW 0b00001001 ; LBRACE
    RETLW 0b01000000 ; PIPE
    RETLW 0b10001001 ; RBRACE
    RETLW 0b10000000 ; TILDE
;===============================================================================
; TABELA PORTB - HIGH Aliniata la pagina
;===============================================================================
PSECT font_high,abs,delta=2
ORG 0x1E00
font_table_high:
    ADDWF PCL,F
ORG 0x1E21

    RETLW 0b11000000 ; (space)
    RETLW 0b11000000 ; EXCLAM
    RETLW 0b11000000 ; QUOTE
    RETLW 0b11010101 ; HASH
    RETLW 0b11010101 ; DOLLAR
    RETLW 0b11111111 ; PERCENT
    RETLW 0b11110010 ; AMP
    RETLW 0b11000000 ; APOSTROPHE
    RETLW 0b11000010 ; LPAREN
    RETLW 0b11101000 ; RPAREN
    RETLW 0b11111111 ; STAR
    RETLW 0b11010101 ; PLUS
    RETLW 0b11001000 ; COMMA
    RETLW 0b11010001 ; MINUS
    RETLW 0b10000000 ; DOT
    RETLW 0b11001000 ; SLASH
    RETLW 0b11001000 ; 0
    RETLW 0b11000000 ; 1
    RETLW 0b11010001 ; 2
    RETLW 0b11000001 ; 3
    RETLW 0b11010001 ; 4
    RETLW 0b11010010 ; 5
    RETLW 0b11010001 ; 6
    RETLW 0b11000000 ; 7
    RETLW 0b11010001 ; 8
    RETLW 0b11010001 ; 9
    RETLW 0b11000100 ; COLON
    RETLW 0b11001000 ; SEMICOLON
    RETLW 0b11010010 ; LT
    RETLW 0b11010001 ; EQUAL
    RETLW 0b11101001 ; GT
    RETLW 0b11000101 ; QUESTION
    RETLW 0b11000001 ; AT
    RETLW 0b11010001 ; A
    RETLW 0b11000101 ; B
    RETLW 0b11000000 ; C
    RETLW 0b11000100 ; D
    RETLW 0b11010000 ; E
    RETLW 0b11010000 ; F
    RETLW 0b11000001 ; G
    RETLW 0b11010001 ; H
    RETLW 0b11000100 ; I
    RETLW 0b11000000 ; J
    RETLW 0b11010010 ; K
    RETLW 0b11000000 ; L
    RETLW 0b11100000 ; M
    RETLW 0b11100010 ; N
    RETLW 0b11000000 ; O
    RETLW 0b11010001 ; P
    RETLW 0b11000010 ; Q
    RETLW 0b11010011 ; R
    RETLW 0b11010001 ; S
    RETLW 0b11000100 ; T
    RETLW 0b11000000 ; U
    RETLW 0b11001000 ; V
    RETLW 0b11001010 ; W
    RETLW 0b11101010 ; X
    RETLW 0b11010001 ; Y
    RETLW 0b11001000 ; Z
    RETLW 0b11000000 ; LBRACKET
    RETLW 0b11100010 ; BACKSLASH
    RETLW 0b11000000 ; RBRACKET
    RETLW 0b11001010 ; CARET
    RETLW 0b11000000 ; UNDERSCORE
    RETLW 0b11100000 ; BACKTICK
    RETLW 0b11010100 ; a
    RETLW 0b11010010 ; b
    RETLW 0b11010001 ; c
    RETLW 0b11001001 ; d
    RETLW 0b11011000 ; e
    RETLW 0b11010101 ; f
    RETLW 0b11000001 ; g
    RETLW 0b11010100 ; h
    RETLW 0b11000100 ; i
    RETLW 0b11001000 ; j
    RETLW 0b11000110 ; k
    RETLW 0b11000000 ; l
    RETLW 0b11010101 ; m
    RETLW 0b11010100 ; n
    RETLW 0b11010001 ; o
    RETLW 0b11110000 ; p
    RETLW 0b11000001 ; q
    RETLW 0b11010000 ; r
    RETLW 0b11000011 ; s
    RETLW 0b11010000 ; t
    RETLW 0b11000000 ; u
    RETLW 0b11001000 ; v
    RETLW 0b11001010 ; w
    RETLW 0b11101010 ; x
    RETLW 0b11000001 ; y
    RETLW 0b11011000 ; z
    RETLW 0b11111000 ; LBRACE
    RETLW 0b11000100 ; PIPE
    RETLW 0b11000011 ; RBRACE
    RETLW 0b11011001 ; TILDE


END