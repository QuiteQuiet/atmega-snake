;
; Snake.asm
;
; Created: 2017-04-20 13:31:08
; Author : a16chrsa
;
.DEF rITemp1	= r1
.DEF rITemp2	= r2
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

.ORG 0x002A
	jmp adc_complete

.ORG INT_VECTORS_SIZE

init:
    // Sätt stackpekaren till högsta minnesadressen
    ldi rTemp, HIGH(RAMEND)
    out SPH, rTemp
    ldi rTemp, LOW(RAMEND)
    out SPL, rTemp

	; Global interrupt enable
	sei

	; ADC config
	ldi rTemp, 0b01100100 ; (AVCC with external capacitor at AREF pin, Left adjusted result)
	sts ADMUX, rTemp
	ldi rTemp, 0b10001111 ; (Enable ADC, 128 Prescaler value, Enable Interrupt)
	sts ADCSRA, rTemp
	
	; input / output pins
	ldi rTemp, 0x3F
	out DDRB, rTemp
	ldi rTemp, 0xF
	out DDRC, rTemp
	ldi rTemp, 0xFC
	out DDRD, rTemp

	jmp main

adc_complete:
	ldi ZL, low(dMatrix)
	ldi ZH, high(dMatrix)
	lds rITemp1, ADCH
	lds rITemp2, ADMUX
	sbrc rITemp2, MUX0
	st Z, rITemp1
	sbrs rITemp2, MUX0
	std Z+1, rITemp1
	ldi r25, 0x01
	eor rITemp2, r25
	sts ADMUX, rITemp2
	lds r25, ADCSRA
	ori r25, (1<<ADSC)
	sts ADCSRA, r25 ; start next ADC
	reti

print:
	ldi ZL, low(dMatrix)
	ldi ZH, high(dMatrix)
	ldi rArraySize, 8
	ldi rRowSelect, 0x1
	print_loop:
		ld	rRow, Z+
		; Column output high bits
		ldi rOutputB, 0 ; clear register
		bst rRow, 5
		bld rOutputB, 0
		bst rRow, 4
		bld rOutputB, 1
		bst rRow, 3
		bld rOutputB, 2
		bst rRow, 2
		bld rOutputB, 3
		bst rRow, 1
		bld rOutputB, 4
		bst rRow, 0
		bld rOutputB, 5

		; Column output low bits
		ldi rOutputD, 0 ; clear register
		bst rRow, 7
		bld rOutputD, 6
		bst rRow, 6
		bld rOutputD, 7

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

	lds rTemp, 0x00
	st	Z+, rTemp
	lds rTemp, 0x00
	st	Z+, rTemp
	ldi rTemp, 0x00
	st  Z+, rTemp
	ldi rTemp, 0x00
	st  Z+, rTemp
	ldi rTemp, 0x00
	st	Z+, rTemp
	ldi rTemp, 0x00
	st	Z+, rTemp
	ldi rTemp, 0x00
	st  Z+, rTemp
	ldi rTemp, 0x00
	st  Z, rTemp

	lds rTemp, ADCSRA
	ori rTemp, (1<<ADSC)
	sts ADCSRA, rTemp ; start first ADC

	loop:
		
		call print

		jmp loop