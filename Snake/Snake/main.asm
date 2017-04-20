;
; Snake.asm
;
; Created: 2017-04-20 13:31:08
; Author : a16chrsa
;

.DEF rTemp  = r16
.DEF rLoop	= r17
.DEF rLoop2	= r18

.ORG 0x0000
	jmp init

init:
    // Sätt stackpekaren till högsta minnesadressen
    ldi rTemp, HIGH(RAMEND)
    out SPH, rTemp
    ldi rTemp, LOW(RAMEND)
    out SPL, rTemp

	ldi rTemp, 0x3F
	out DDRB, rTemp
	ldi rTemp, 0xF
	out DDRC, rTemp
	ldi rTemp, 0xFC
	out DDRD, rTemp




loop:
	ldi rTemp, 0b00000001 ; 0b0000001
	out PORTC, rTemp
	ldi rTemp, 0b01000000 ; 0b
	out PORTD, rTemp
	ldi rTemp, 0b00000000 ; 0b00111111
	out PORTB, rTemp


	; två
	ldi rTemp, 0b00000000 ; 0b0000001
	out PORTC, rTemp
	ldi rTemp, 0b00000100 ; 0b
	out PORTD, rTemp
	ldi rTemp, 0b00000100 ; 0b00111111
	out PORTB, rTemp
	jmp loop