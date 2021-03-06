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
.DEF rDirection	= r5
.DEF rOldSnakePos = r6
.DEF rPart		= r7
.DEF rHeadPos	= r8
.DEF rApple		= r9
.DEF rTick		= r10
.DEF rRandom	= r11
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
; Matrix for LED-array
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
    // Set stackpointer to highest memoryaddress 
    ldi rTemp, HIGH(RAMEND)
    out SPH, rTemp
    ldi rTemp, LOW(RAMEND)
    out SPL, rTemp

	; Global interrupt enable
	sei

	; Timer0 config
	ldi rTemp, 0b00000100 ; (Timer0 enabled, /256 clock cycle)
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
	; do some random generation 
	lds rITemp1, ADCL
	swap rITemp1
	lsr rITemp1
	mul rITemp1, rITemp1
	mov rITemp1, r0
	add rRandom, rITemp1

	; deal with movement
	lds rITemp1, ADCH
	lds rITemp2, ADMUX

	ldi r25, 0x20
	cp rITemp1, r25
	brsh over_0x20
	ldi r25, 0
	cbr r25, 1
	mov r15, r25
	rjmp get_axis

	over_0x20:
	ldi r25, 0xE0
	cp rITemp1, r25
	brlo move_end
	ldi r25, 0
	sbr r25, 1
	mov r15, r25

	get_axis:
	ldi r25, 0
	sbrs rITemp2, MUX0
	sbr r25, 2
	or r15, r25
	mov r25, rDirection
	andi r25, 0b11111100
	or r15, r25
	mov rDirection, r15

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
	; mark that 255 timer0 overflows have happened
	ldi r25, 0xFF
	mov rTick, r25
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
	st X+, rTemp
	ldi rTemp, 0x34
	st X+, rTemp
	ldi rTemp, 0x34
	st X+, rTemp
	ldi rTemp, 0x3
	mov rLength, rTemp
	ldi rTemp, 0x21
	mov rApple, rTemp

	ldi ZL, low(dMatrix)
	ldi ZH, high(dMatrix)

	ldi rTemp, 0x00
	st	Z+, rTemp
	; Set TimerCount and Direction registers to start values
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
		sbrs rTick, 1
		jmp loop

		ldi XL, low(dMatrix)
		ldi XH, high(dMatrix)
		ldi rArraySize, 8
		; Clearing array
		matrix_clear_loop:
			ld rTemp, X
			clr rTemp
			st X+, rTemp
			dec rArraySize
			brne matrix_clear_loop
		ldi XL, low(dMatrix)
		ldi XH, high(dMatrix)
		ldi YL, low(dSnake)
		ldi YH, high(dSnake)
		mov rArraySize, rLength
		move_snake_loop:
			ld rPart, Y

			; jump if not head
			cpse rArraySize, rLength
			jmp move_snake_part

			; move snake head position
			mov rOldSnakePos, rPart
			; swap if we want the x position, skip if we don't
			sbrs rDirection, 1
			swap rPart
	
			; increment and decrement the head based on what the rDirection register say
			ldi rTemp, 0x7
			and rPart, rTemp
			sbrc rDirection, 0
			dec rPart
			sbrs rDirection, 0
			inc rPart
			ldi rTemp, 0x7
			and rPart, rTemp
			; load coordinates for the snake again (if x position changed, we need the old y again)
			ld rTemp, Y
			sbrs rDirection, 1
			jmp x_pos_change

			; y-position changed
			andi rTemp, 0xF0
			or rPart, rTemp
			mov rHeadPos, rPart ; store new head position for collision
			jmp apple_collision

			; x-position changed
			x_pos_change:
			andi rTemp, 0x0F
			swap rPart
			or rPart, rTemp
			mov rHeadPos, rPart ; store new head position for collision
			jmp apple_collision

			move_snake_loop_middle_jump_back:
			jmp move_snake_loop

			; test apple collision
			apple_collision:
			cpse rApple, rHeadPos
			jmp new_pos_store
			inc rLength
			; generate new apple position
			mov rTemp, rTimerCount
			add rTemp, rRandom
			andi rTemp, 0x77
			mov rApple, rTemp

			mov rApple, rTemp
			jmp new_pos_store

			; move every other part that isn't the head
			move_snake_part:
			mov rTemp, rPart
			mov rPart, rOldSnakePos
			mov rOldSnakePos, rTemp

			; collision detection
			cpse rHeadPos, rPart
			jmp new_pos_store

			; snake head collided with a body part
			jmp init

			new_pos_store:
			st Y+, rPart

			; set matrix bit where this snake part is at
			mov rSnakeX, rPart
			swap rSnakeX
			andi rSnakeX, 0x07
			mov rSnakeY, rPart
			andi rSnakeY, 0x07
			; copy index 0 of the matrix to Z from X to keep the
			; reference to X clean for future operations
			movw Z, X
			add ZL, rSnakeY		; add the y position to the matrix index to select row
			lds rTemp, SREG
			sbrc rTemp, 3		; check if overflow flag is set to compensate for addition
			inc ZH				; compensate so we're in the right memory space if overflow DID happen
			ld rRow, Z
			ldi rTemp, 0x80
			; check if the snake part is in index 0
			cpi rSnakeX, 1
			brlo update_position
			part_find_xpos:
				lsr rTemp
				dec rSnakeX
				brne part_find_xpos
			update_position:
			or rRow, rTemp
			st Z, rRow
			; comtinue or conclude the loop
			dec rArraySize
			brne move_snake_loop_middle_jump_back

		; print apple position
		mov rSnakeX, rApple
		swap rSnakeX
		andi rSnakeX, 0x07
		mov rSnakeY, rApple
		andi rSnakeY, 0x07
		; copy index 0 of the matrix to Z from X to keep the
		; reference to X clean for future operations
		movw Z, X
		add ZL, rSnakeY		; add the y position to the matrix index to select row
		lds rTemp, SREG
		sbrc rTemp, 3		; check if overflow flag is set to compensate for addition
		inc ZH				; compensate so we're in the right memory space if overflow DID happen
		ld rRow, Z
		ldi rTemp, 0x80
		; check if the snake part is in index 0
		cpi rSnakeX, 1
		brlo apple_update_position
		apple_find_xpos:
			lsr rTemp
			dec rSnakeX
			brne apple_find_xpos
		apple_update_position:
		or rRow, rTemp
		st Z, rRow

		; we've now dealth with the last overflow that happend and are ready for a new one
		ldi rTemp, 0x00
		mov rTick, rTemp
		jmp loop