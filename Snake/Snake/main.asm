;
; Snake.asm
;
; Created: 2017-04-20 13:31:08
; Author : a16chrsa
;
.DEF rITemp1	= r1
.DEF rITemp2	= r2
.DEF rTimerCount = r3
.DEF rLength	= r4
.DEF rOldSnakePos = r5
.DEF rTemp		= r16
.DEF rArraySize = r17
.DEF rSnakeX	= r17
.DEF rSnakeY	= r18
.DEF rRow		= r18
.DEF rSnakePart	= r19
.DEF rOutputB	= r19
.DEF rOutputC	= r20
.DEF rIndex		= r20
.DEF rOutputD	= r21
.DEF rRowSelect	= r22
.DEF rPosition	= r23
.DEF rDirection	= r24

; constants
.EQU MAX_LENGTH    = 25

.DSEG
; Datasegment för lysdiodarrayen
dMatrix: .BYTE 8
; Snake array
; The snake array is stored as a byte per body piece where each index is the
; X, Y position of that body part on the board. For example if dSnake[0] is 0x34
; it means that the piece is at (0 indexed) row 4 and column 3 in the matrix.
; Each coordinate use one nibble for its location.
; Example dSnake: {0x34, 0x24, 0x14} (three long, head at 0x34 and turned towards the right)
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
	mov r25, rDirection
	andi r25, 0xFC
	cbr r25, 1 ; RÄKNAR FRÅN ETT!!!?!?!?!?!?!?
	mov r10, r25
	rjmp get_axis

	over_0x20:
	ldi r25, 0xE0
	cp rITemp1, r25
	brlo move_end
	mov r25, rDirection
	andi r25, 0xFC
	sbr r25, 1
	mov r10, r25

	get_axis:
	mov r25, rDirection
	andi r25, 0xFC
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
	ldi ZL, low(dMatrix)
	ldi ZH, high(dMatrix)
	inc rTimerCount
	st Z, rTimerCount
	breq t0_return
	sbr rDirection, 5 ; mark that 255 timer0 overflows have happened by setting this bit
	std Z+1, rDirection
	t0_return:
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
	ldi rTemp, 0x34
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
		sbrs rDirection, 4
		jmp loop
	game_logic:
		ldi XL, low(dMatrix)
		ldi XH, high(dMatrix)
		ldi rRow, 8
		array_clear_loop:
			ld rTemp, X
			clr rTemp
			st X+, rTemp
			dec rRow
			brne array_clear_loop
		ldi XL, low(dMatrix)
		ldi XH, high(dMatrix)
		ldi YL, low(dSnake)
		ldi YH, high(dSnake)
		mov rIndex, rLength
		move_snake_loop:
			ld rSnakePart, Y
			; TODO: jump if not head

			; move snake head position
			mov rOldSnakePos, rSnakePart
			; swap if we want the x position, skip if we don't
			sbrs rDirection, 1
			swap rSnakePart

			; increment and decrement the head based on what the rDirection register say
			andi rSnakePart, 0x7
			sbrc rDirection, 0
			dec rSnakePart
			sbrs rDirection, 0
			inc rSnakePart
			andi rSnakePart, 0x7
			; load coordinates for the snake again (if x position changed, we need the old y again)
			ld rPosition, Y
			sbrs rDirection, 1
			rjmp x_pos_change

			; y-position changed
			andi rPosition, 0xF0
			or rSnakePart, rPosition
			rjmp new_pos_store

			; x-position changed
			x_pos_change:
			andi rPosition, 0x0F
			swap rSnakePart
			or rSnakePart, rPosition
			rjmp new_pos_store

			; move every other part that isn't the head
			; TODO

			new_pos_store:
			st Y+, rSnakePart

			; set matrix bit where this snake part is at
			mov rSnakeX, rSnakePart
			swap rSnakeX
			andi rSnakeX, 0x07
			mov rSnakeY, rSnakePart
			andi rSnakeY, 0x07
			; copy index 0 of the matrix to Z from X to keep the
			; reference to X clean for future operations
			movw Z, X
			add ZL, rSnakeY		; add the y position to the matrix index to select row
			lds rTemp, SREG
			sbrc rTemp, 3 		; check if overflow flag is set to compensate for addition
			inc ZH 				; compensate so we're in the right memory space if overflow DID happen
			ld rPosition, Z
			ldi rTemp, 0x80
			; check if the snake part is in index 0
			cpi rSnakeX, 1
			brlo update_position
			find_xpos:
				lsr rTemp
				dec rSnakeX
				brne find_xpos

			update_position:
			or rPosition, rTemp
			st Z, rPosition
			; comtinue or conclude the loop
			dec rIndex
			brne move_snake_loop
		cbr rDirection, 4	; we've now dealth with the last overflow that happend and are ready for a new one
		jmp loop