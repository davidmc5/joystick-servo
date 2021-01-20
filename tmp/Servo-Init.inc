; This file needs to be included at the start of the main program, 

; after the interrupt code at org 04H of the .asm file,
; and after the initial configuration settings and ; with the following command:
; #INCLUDE Servo-Init.inc

;Requires the following variables in unbanked register CBLOCK 070h:


;************************************************************************************************* 
; SERVOS INITIALIZATION
;************************************************************************************************* 


	;configure timer0 prescaler
	banksel OPTION_REG
	movlw	B'00000111'		;Configure max prescale of 256 for Timer0, bits 0-2 = 111
	movwf	OPTION_REG


	;This is just for an initial idle time before the ISR starts the servo pulses loop
	banksel TMR0
	movlw	TIME_18ms_INT 	;Set initial idle time.	 
	movwf	TMR0

	;Initialize servo pulse pins
	call SERVO_Stop

	;set servo pins as outputs
	BANK1
	bcf	TRISA, 0
	bcf	TRISA, 2
	bcf	TRISA, 4
	bcf	TRISA, 5
	bcf	TRISC, 4
	bcf	TRISC, 5
	bcf	TRISC, 6
	bcf	TRISC, 7

	;start sequence with idle time for ISR
	bsf IdleFlag 		;execute idle time on next interrupt (all servos off)

	bsf INTCON, T0IE	;unmask/enable Timer0 Overflow Interrupt
