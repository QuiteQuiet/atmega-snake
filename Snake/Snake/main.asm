;
; Snake.asm
;
; Created: 2017-04-20 13:31:08
; Author : a16chrsa
;
.DEF rITemp1	= r1
.DEF rITemp2	= r2
.DEF rITemp3	= r3
.DEF rITemp4	= r4
.DEF rTimerCount = r5
.DEF rLength	= r6
.DEF rDirection	= r7
.DEF rOldSnakePos = r8
.DEF rTemp		= r16
.DEF rArraySize = r17
.DEF rRow		= r18
.DEF rOutputB	= r19
.DEF rOutputC	= r20
.DEF rOutputD	= r21
.DEF rRowSelect	= r22
.DEF rSnakeX	= r23
.DEF rSnakeY	= r24

; constants
.EQU MAX_LENGTH    = 25

.DSEG
; Datasegment för lysdiodarrayen
dMatrix: .BYTE 8
; Snake array
dSnake: .BYTE MAX_LENGTH + 1

.CSEG

.ORG 0x0000
	jmp init

.ORG 0x0020
	jmp t0_overflow

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

	; Timer0 config
	ldi rTemp, 0b00000100 ; (Timer0 enabled, /1024 clock cycle)
	out TCCR0B, rTemp
	ldi rTemp, (1<<TOIE0) ; (Enable Timer0 interrupt)
	sts TIMSK0, rTemp

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

; ADC interrupt subroutine
adc_complete:
	ldi ZL, low(dMatrix)
	ldi ZH, high(dMatrix)
	lds rITemp1, ADCH
	lds rITemp2, ADMUX

	ldi r25, 0x20
	cp rITemp1, r25
	brsh over_0x20
	ldi r25, 0
	cbr r25, 1 ; RÄKNAR FRÅN ETT!!!?!?!?!?!?!?
	mov r10, r25
	rjmp get_axis

	over_0x20:
	ldi r25, 0xE0
	cp rITemp1, r25
	brlo move_end
	ldi r25, 0
	sbr r25, 1
	mov r10, r25

	get_axis:
	ldi r25, 0
	sbrs rITemp2, MUX0
	sbr r25, 2
	or r10, r25
	mov rDirection, r10

	move_end:
	ldi r25, 0x01
	eor rITemp2, r25
	sts ADMUX, rITemp2
	lds r25, ADCSRA
	ori r25, (1<<ADSC)
	sts ADCSRA, r25 ; start next ADC
	reti

; Timer0 interrupt subroutine
t0_overflow:
	inc rTimerCount
	breq t0_do_stuff
	reti
	t0_do_stuff:
	ldi XL, low(dMatrix)
	ldi XH, high(dMatrix)
	ldi r23, 8
	t0_clear_loop:
		ld r24, X
		clr r24
		st X+, r24
		dec r23
		brne t0_clear_loop
	ldi XL, low(dMatrix)
	ldi XH, high(dMatrix)
	ldi YL, low(dSnake)
	ldi YH, high(dSnake)
	mov rITemp1, rLength
	t0_snake_loop:
		ld rITemp2, Y
		; move snake head position
		mov rOldSnakePos, rITemp2
		sbrs rDirection, 1
		swap rITemp2
	
		ldi r25, 0x7
		and rITemp2, r25
		sbrc rDirection, 0
		dec rITemp2
		sbrs rDirection, 0
		inc rITemp2
		ldi r25, 0x7
		and rITemp2, r25
		ld r25, Y
		sbrs rDirection, 1
		rjmp x_pos_change
		andi r25, 0xF0
		or rITemp2, r25
		rjmp new_pos_store

		x_pos_change:
		andi r25, 0x0F
		swap rITemp2
		or rITemp2, r25

		new_pos_store:
		st Y+, rITemp2

		mov rSnakeX, rITemp2
		swap rSnakeX
		andi rSnakeX, 0x07
		mov rSnakeY, rITemp2
		andi rSnakeY, 0x07
		movw Z, X
		add ZL, rSnakeY
		lds rITemp3, SREG
		sbrc rITemp3, 3 ; check if overflow flag is set
		inc ZH
		ld rITemp4, Z
		ldi r25, 0x80
		cpi rSnakeX, 1
		brlo update_position
		t0_find_xpos:
			lsr r25
			dec rSnakeX
			brne t0_find_xpos
		update_position:
		or rITemp4, r25
		st Z, rITemp4
		dec rITemp1
		brne t0_snake_loop
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
	ldi XL, low(dSnake)
	ldi XH, high(dSnake)
	; start the snake in the center
	ldi rTemp, 0x36
	st X, rTemp
	ldi rTemp, 0x1
	mov rLength, rTemp

	ldi ZL, low(dMatrix)
	ldi ZH, high(dMatrix)

	ldi rTemp, 0x00
	st	Z+, rTemp
	mov rTimerCount, rTemp
	mov rDirection, rTemp
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