;
; Snake.asm
;
; Created: 2017-04-20 13:31:08
; Author : a16chrsa
;

.DEF rTemp		= r16
.DEF rArraySize = r17
.DEF rRow		= r18
.DEF rOutputB	= r19
.DEF rOutputC	= r20
.DEF rOutputD	= r21
.DEF rRowSelect	= r22

.DSEG
; Datasegment för lysdiodarrayen
dMatrix: .BYTE 8

.CSEG

.ORG 0x0000
	jmp init
.ORG INT_VECTORS_SIZE

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

	jmp main


print:
	ldi ZL, low(dMatrix)
	ldi ZH, high(dMatrix)
	ldi rArraySize, 8
	ldi rRowSelect, 0x1
	print_loop:
		ld	rRow, Z+
		; Column output
		ldi rOutputB, 0b11111100
		and rOutputB, rRow
		lsr rOutputB
		lsr rOutputB
		
		ldi rOutputD, 0b00000011
		and rOutputD, rRow
		; move to top of D register
		lsl rOutputD
		lsl rOutputD
		lsl rOutputD
		lsl rOutputD
		lsl rOutputD
		lsl rOutputD

		; add row counter high bits
		mov rTemp, rRowSelect
		lsr rTemp
		lsr rTemp
		andi rTemp, 0b00111100
		or  rOutputD, rTemp

		; row counter low bits
		mov rOutputC, rRowSelect
		andi rOutputC, 0b00001111
		
		; output information
		out PORTB, rOutputB
		out PORTC, rOutputC
		out PORTD, rOutputD

		lsl rRowSelect
		dec rArraySize

		brne print_loop

	ret


main:
	ldi ZL, low(dMatrix)
	ldi ZH, high(dMatrix)

	ldi rTemp, 0x55
	st	Z+, rTemp
	ldi rTemp, 0x00
	st	Z+, rTemp
	ldi rTemp, 0x00
	st  Z+, rTemp
	ldi rTemp, 0x00
	st  Z+, rTemp
	ldi rTemp, 0x00
	st	Z+, rTemp
	ldi rTemp, 0x00
	st	Z+, rTemp
	ldi rTemp, 0xaa
	st  Z+, rTemp
	ldi rTemp, 0x00
	st  Z, rTemp

	loop:
		
		call print

		jmp loop