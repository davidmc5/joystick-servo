; This file needs to be included at the start of the main program, 

; after the interrupt code at org 04H of the .asm file,
; and after the initial configuration settings and ; with the following command:
; #INCLUDE Joystick-Init.inc

;to adjust the count value of the RC, adjust the folowing three controls:

; 1 - timer preescaler:			T1CON,T1CKPS0 and T1CON,T1CKPS1
; 2 - voltage reference Vref:	VRCON - 4 least signif digits
; 3 - voltage divider range:	VRCON, VRR


;Requires the following variables in unbanked register CBLOCK 070h:

;UART_Temp	;Variable for UART subroutine 
;Delay1 


;*************************************************************************************************
; TIMER 1 INITIALIZATION
;*************************************************************************************************

	;timer 1 is used in combination with comparator 2 to determine the charge time of the capacitors

	;configure timer 1 parameters
	banksel	T1CON			;bank 0
	bcf		T1CON, TMR1ON	;stop timer 1. Set this bit to start counting.

	bcf		T1CON, TMR1CS	;use as clock Fosc/4 (1us per clock)

	;next two bits set preescaler dividing factor
	;joystick tested with 10:4
	; possible divide values: 00:1, 01:2, 10:4, 11:8
	bsf		T1CON,T1CKPS0	
	bsf		T1CON,T1CKPS1

	bsf		T1CON, TMR1GE	;start/stop counting controlled by comp 2 output
	bcf		T1CON, T1GINV	;count while comp 2 output is low. Stop counting when high.
	
 
	;disable timer 1 interrupts (not currently using them). 
	;Interrupt is trigered by comparator when cap reaches charge level).
	banksel	PIR1			;bank 0
	bcf		PIR1, TMR1IF	;clear interrupt flag
	banksel	PIE1			;bank 1
	bcf		PIE1, TMR1IE	;mask out timer 1 interrupt
	

;************************************************************************************************* 
; COMPARATOR 2 INITIALIZATION
;************************************************************************************************* 

	; The comparator 2 is used to determine when the capacitor charge has reached the reference voltage
	; The time to reach that voltage is measured by timer 1
	; The 4 capacitors (4 axis) are multiplexed into comparator 2.

	;set joystick pins as analog inputs
	banksel	ANSEL			
	bsf	ANSEL, ANS1		;Set port ANS1 as analog - JOY_AXIS_1 (RA1 - pin 18)
	bsf	ANSEL, ANS5		;Set port ANS1 as analog - JOY_AXIS_1 (RC1 - pin 15)
	bsf	ANSEL, ANS6		;Set port ANS1 as analog - JOY_AXIS_1 (RC2 - pin 14)
	bsf	ANSEL, ANS7		;Set port ANS1 as analog - JOY_AXIS_1 (RC3 - pin 7)

	;enable output drivers to capacitor pins to discharge when pin is set to zero
	;changing this setting to an input will allow the capacitors to charge
	banksel TRISA
	bcf		TRISA,1		;set pin as an output (RA1)
	bcf		TRISC,1		;set pin as an output (RC1)
	bcf		TRISC,2		;set pin as an output (RC2)
	bcf		TRISC,3		;set pin as an output (RC3)



	;write a zero to the capacitor pins to discharge when the ports are configured as digital outputs
	;to charge, set the port as input to allow current from the joystick pot into the capacitor.
	;these are the 4 comparator pin inputs conected to the 4 joystick axis resistors.
	banksel PORTA
	bcf	PORTA, 1			;input from joystick axis 1(RA1 - pin 18)
	bcf	PORTC, 1			;input from joystick axis 2(RC1 - pin 15)
	bcf	PORTC, 2			;input from joystick axis 3(RC2 - pin 14)
	bcf	PORTC, 3			;input from joystick axis 4(RC3 - pin 7)

	;set gate control for timer1
	banksel	CM2CON1			;bank 2
	bcf		CM2CON1, T1GSS	;enable timer 1 to increment based on comp 2 output
	bsf		CM2CON1, C2SYNC	;latch comp2 output with falling edge of timer 1


	;set voltage reference Vref to detect capacitor charge level
	;max voltage that can be selected is 3.75v (0Fh) (@VDD = 5.0v)
	; Maximum all 4 LSB = 1 (range: 00-0F)
	banksel	VRCON			;bank 2
	movlw	0Bh				;to set voltage ref for joystick
	;movlw	0Fh				;test
	movwf	VRCON
	bcf		VRCON, VRR	;	;set voltage range. 1=lower voltage, 0=higher voltage
	bsf		VRCON, C2VREN	;power voltage divider to supply selected Vref for comp 2 (input +)
	bsf		VRCON, VP6EN	;disable zero voltage reference on Vref. (see sect 8.10.3 data sheet) 

	;configure comparator 2
	banksel CM2CON0			;bank 2
	clrf	CM2CON0			;disable comp output pins
	bsf		CM2CON0, C2POL
	bsf		CM2CON0, C2ON	;enable comparator 2
	bsf		CM2CON0, C2R	;select internal voltage reference
	banksel SRCON			;bank 3
	clrf	SRCON			;disable SR LATCH functionality


	;initialize number of joystick RC axis sensing
	movlw Num_Axis			;set the number of axis to sense
	movwf Current_Axis 		;valid values 1-4


;\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\start pot sensing. not needed if servo will start it.

	call Start_Pot_Sensing
