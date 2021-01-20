 title  "Joystick - Controlling multiple Servos with a joystick sidewinder 3D Pro (PIC16F690)
#define _version “1.00”
;
; NOTE: For enabling mux channel 1, undo changes on subroutines below: charge cap, discharge cap and set channel
; this change is to bypass channel 1 since the same pin 18 is being used by the programmer and is preventing
; the capacitor to charge correctly while the programmer is plugged.

; Update History:

; 6/19/15 - adding UART code (non interrupt) to view counters and debug
; This works. it measures the charging time of capactor to voltage reference
; sends the 2 byte count as FF FF in ascii to the UART 

;joystickTest_2 - tested with 2 axis. Can be extended to a max of 4 axis defining Num_Axis = 4
;the max number of axis to measure is limited by the number of mux comparator channels (4) of the PIC.
;completly done via via comparator 2 interrupts
; no scaling yet.
; If joystick resistors are not connected, PIN input will be floating and never reach the Vref
; required to trigger an interrupt.
;
; Author
; David Martin 
; June 19, 2015
;
;  Hardware Notes:
;   PIC16F690 Using 4 MHz Internal Clock

;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
; To scale/change the count of Timer 1 for a variation of resistance:

;1) change timer 1 clock pre-scaler T1CON,T1CKPS<1:0>
;2) change comparator 2 voltage reference VRCON,VR<3:0>
;3) change comparator 2 voltage range bit VRCON,VRR
;4) change capacitor
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

;to make it easier to reuse code for each module:

; move initialization rutines for each module to a header file
; move subrutines to a header file
; move interrupts to a header file

;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;TO DO

;2 - control 8 servos (4 with joystick and 4 external)
;3 - sense 4 switches from sidewinder
;4 - output all data to uart
;5 - control servos remotely via sockets between two RPi

;3 - add scaling and offset via button
;4 - X-Y axis centering via button

;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



; Set the default (undeclared) radix as decimal. 
  LIST R=DEC	; (use H for hex, i.e., 50H)

;------------------------------------------------------
;------------------------------------------------------
; Device Specification
 #INCLUDE "p16f690.inc"	;Include file with device's register and config bit definitions
 ;#INCLUDE <p16f690.inc>	;Include file with device's register and config bit definitions

; Configuration settings fuses (see definitions in p16f690.inc file)

 __CONFIG _INTOSCIO & _WDT_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _BOR_OFF & _IESO_OFF & _FCMEN_OFF

;_INTOSCIO			Use internal oscillator with no ext clock in/out (RA4 and RA5 pins function as I/O)
;_INTOSC			Use internal oscillator with no ext clock in (RA5 is a I/O) but Ext Clock out on R4.
;_WDT_OFF			Watch Dog Timer Off
;_PWRTE_OFF			Power-up Timer Off
;_MCLRE_OFF			MCLR pin function is digital input, MCLR internally tied to VDD
;_CP_OFF			Program memory code protection is disabled
;_BOR_OFF			Brown-out reset Off
;_IESO_OFF			Internal-External Oscilator Switchover mode is disabled
;_FCMEN_OFF			Fail-Safe Clock Monitor is disabled (switches to internal osc if ext fails)


;------------------------------------------------------
;DEFINE VARIABLES (use movwf to store to a reg . Use movf to retrieve from a reg or movlw literal
;------------------------------------------------------
;Shadow/Unbanked Variables (16 bytes). Data memory for interrupt routine. No Need for "banksel".

	CBLOCK 070h

		; General
		W_Temp			; variable used for context saving (W register) for interrupts
		STATUS_Temp		; variable used for context saving (STATUS register) for interrupts
		Delay1, Delay2	;used for time delay
		Delay1_Temp  	;temp variable during interrupt stack swap
		temp			;used for temporary storage

		; Servo Control
		nextServoAddr	; Holds the address of the next servo position (used for indirect addressing)
		nextServoPin	; Control pin for next servo
		servoCount		; starts with the number of servos and when decrements to zero, enable idle time for next interrupt
		IntFlags		; Used by Timer 0 interrupt routine. (See bit definitions for info on individual flags)

		;UART Control
		UART_Temp		;used by UART to send data to display

		;Joystick Control
		Current_Axis	;holds the current axis being timed	
 	ENDC


; Bank 0 Variables (80 bytes)- Use "banksel <variable_name>" before addressing these registers
	CBLOCK 020h
		servo1pos		;Servo 1 desired position (0 - 255)
		servo2pos		;Servo 2 desired position (0 - 255)
		servo3pos		;Servo 3 desired position (0 - 255)
		servo4pos		;Servo 4 desired position (0 - 255)
		servo5pos		;Servo 5 desired position (0 - 255)
		servo6pos		;Servo 6 desired position (0 - 255)
		servo7pos		;Servo 7 desired position (0 - 255)
		servo8pos		;Servo 8 desired position (0 - 255)

		Count_1H, Count_1L		;counter axis 1 (2 bytes) 
		Count_2H, Count_2L		;counter axis 2 (2 bytes) 
		Count_3H, Count_3L		;counter axis 3 (2 bytes) 
		Count_4H, Count_4L		;counter axis 4 (2 bytes)


	ENDC


; Bank 1 Variables (80 bytes)- Use "banksel variable_name" before addressing these registers
	CBLOCK 0A0h
		;variable name
 	ENDC


; Bank 2 Variables (80 bytes)- Use "banksel variable_name" before addressing these registers
	CBLOCK 120h
		;variable name
 	ENDC



;------------------------------------------------------
;DEFINE LITERALS (Assembler substitutions: use movlw to retrieve its defined value at assembly time)
;------------------------------------------------------
; JOYSTICK PORT/PIN DEFINITIONS

; Joystick Control
; these are the 4 comparator pin inputs conected to the 4 joystick axis resistors
;set as input to start charging and set as output (with a zero) to discharge
;Use BANK1
#define CAP_1 		TRISA,1			;input from joystick axis 1(RA1 - pin 18)
#define CAP_2 		TRISC,1			;input from joystick axis 2(RC1 - pin 15)
#define CAP_3 		TRISC,2			;input from joystick axis 3(RC2 - pin 14)
#define CAP_4 		TRISC,3			;input from joystick axis 4(RC3 - pin 7)

#define Num_Axis	4				;number of joystick axis to sense (can't be greater than 4!)

;SERVOS PORT/PIN DEFINITIONS

#define SERVO_ADDR 	020h

#define SERVO_1		PORTA, 0
#define SERVO_2		PORTA, 2
#define SERVO_3		PORTA, 4
#define SERVO_4		PORTA, 5
#define SERVO_5		PORTC, 4
#define SERVO_6		PORTC, 5
#define SERVO_7		PORTC, 6
#define SERVO_8		PORTC, 7

#define NUMSERVOS	8			;number of servos to control (1-8)

#define IdleFlag	IntFlags, 0	;1= idle time, all pulses off; 0= servos' pulse loop on (one at a time)

#define ServoPos servo8pos	;FOR TEST ONLY
;------------------------------------------------------
; Constants Definitions

;Timer 0 Interrupts (Used in the Interrupt Service Routine)
; The PIC clock frequency (or instruction cycle)is = (Freq Oscilator) / 4
; With the internal oscilator frequency set for 4MHz, the clock frequency is = (Osc freq)/4 = 1 MHz (or 1 us per instruction cycle).
; The timer0 period is = Instruction Cycle (1us) x Prescaler (256) x (remaining count to overflow: 256 - TMR0 )

; for pulse (with preescaler = 256)
;TIME_0.5ms_INT	equ	254d			
;TIME_0.8ms_INT	equ	253d
;TIME_1ms_INT	equ	252d		;left			
;TIME_1.3ms_INT	equ	251d
;TIME_1.5ms_INT	equ	250d		;middle 
;TIME_1.8ms_INT	equ	249d			
;TIME_2ms_INT	equ	248d		;right

; for pulse (with preescaler = 8)
TIME_0.5ms_INT	equ	160			
TIME_1.5ms_INT	equ	80		
TIME_2ms_INT	equ	0	

; for idle (with preescaler = 256)
TIME_40ms_INT	equ	100d		;seems like 40 ms delay is ok too. Can add more servos @ 2 ms each	
TIME_18ms_INT	equ	186d		;Starting at 186, TMR0 overflows after 70 additional clocks: 70 + 186 = 256
									;256 (prescaler) * 70 (TMR0) ~ 18 ms

;------------------------------------------------------

;------------------------------------------------------
;------------------------------------------------------
;MACRO DEFINITIONS

;------------------------------------------------------
DELAY macro
	
	clrf	Delay1
	clrf	Delay2
;Loop:
	decfsz	Delay1,f
	goto	$-1		;goto loop:
	decfsz	Delay2,f
	goto	$-3 	;goto loop:
	
	endm

;------------------------------------------------------
BANK0 macro
	bcf	STATUS, RP0
	bcf	STATUS, RP1
	endm

BANK1 macro
	bsf	STATUS, RP0
	bcf	STATUS, RP1
	endm

BANK2 macro
	bcf	STATUS, RP0
	bsf	STATUS, RP1
	endm

BANK3 macro
	bsf	STATUS, RP0
	bsf	STATUS, RP1
	endm



;------------------------------------------------------
;macro to facilitate switch structure (used in subroutines)
;it jumps to the specified 'label' if 'register' matches 'literal'
;otherwise it continues with the next line after JIFEQ.
JIFEQ   MACRO   register,literal,label 
        movlw   literal 
        xorwf   register,w 
        btfsc	STATUS, Z 
         goto   label
        ENDM 

;------------------------------------------------------
;FOR TESTING JOYSTICK ONLY
;print 4-axis counters to UART
;used in Display_Pos subroutine

DisplPos MACRO register
		;MSB - digit 1
		BANK0					;bank 0
		swapf	register, W		;get MOST significant half byte into W
		call	ascii_lookup	;uart hex-ascii conversion
		call send				;uart send
		;MSB - digit 0
		BANK0					;bank 0
		movf	register, W		;get LEAST significant half byte into W
		call	ascii_lookup
		call send
		endm

;------------------------------------------------------
;------------------------------------------------------



 PAGE ;Inserts a page eject into the listing file.

;**********************************************************************
;RESET_VECTOR    
 	org 0h       	; processor reset vector memory address
	goto    start	; go to beginning of the program

;**********************************************************************
;INT_VECTOR
	org 04H       				; start of interrupt vector memory address
	#include interrupts.inc		; Interrupt Service Rutine


;**********************************************************************

start:

;**********************************************************************
; GENERAL CONFIGURATION SETINGS
;**********************************************************************

	; If an interrupt occurs during the execution of the instruction to disable the interrupts (bcf GIE)
	; the PIC could still sjump to the interrupt service routine. Upon returning with RETFIE,
	; the GIE flag will get set again. We need to verify the GIE flag remains cleared.
	clrf	INTCON			; disable all interrupts
	btfsc	INTCON, GIE		; check if GIE is still disabled
	goto	start			; if not yet disable, keep trying

	; mask all peripheral interrupts
	clrf	PIE1
	clrf	PIE2 ; bit PIE2,C2IE is for comparator 2

	;clear all peripheral interrupt flags
	banksel	PIR1			;bank 0
	clrf	PIR1			
	clrf	PIR2

	;set internal oscillators at 4 MHz
	banksel	OSCCON
	movlw	61h		;Sets internal oscilator for clock at 4 MHz (default)	
	movwf	OSCCON 

	;set initially all analog pins as digital I/O
	banksel	ANSEL			
	clrf	ANSEL		;Set ports ANS0-7 to digital
	clrf	ANSELH		;Set ports ANS8-11 to digital


;************************************************************************************************* 
; MODULE INITIALIZATION

#INCLUDE Joystick-Init.inc	
#INCLUDE Servo-Init.inc
#INCLUDE UART-Init.inc

;*************************************************************************************************
;*************************************************************************************************
	
	banksel INTCON
	bsf		INTCON, PEIE	;enable all unmasked peripheral interrupts
	bsf 	INTCON, GIE		;Enable Interrupts

;*************************************************************************************************
;*************************************************************************************************
;*************************************************************************************************
;*************************************************************************************************
;MAIN PROGRAM
;*************************************************************************************************

main:

	;goto display

	BANK0
;	movlw TIME_0.8ms_INT
;	movlw TIME_2ms_INT

    movf	Count_1L, W
	;movlw	TIME_2ms_INT ; delete this
	movwf	servo6pos

    movf	Count_2L, W
	;movlw	TIME_2ms_INT ; delete this
	movwf	servo8pos

    movf	Count_3L, W
	;movlw	TIME_2ms_INT ; delete this
	movwf	servo7pos

    movf	Count_4L, W
	;movlw	TIME_2ms_INT ; delete this
	movwf	servo5pos

	;DisplPos Count_3L
	;movlw  0x0D ; CR 
	;call send

	goto 	main



loop1:
	call Display_Pos	;display values of pots via UART
	call SERVO_Test1
	goto loop1


display:
	call SERVO_Test1
	;call Display_Pos	;display values of pots via UART
	goto display


;--------------------------------------------------------------
;UART Test
loop    call receive            ; waits FOREVER until a char is in W
        call send               ; send the char just received in W
        goto loop 



;**********************************************************************
;**********************************************************************
;**********************************************************************
;**********************************************************************
; SUBROUTINES
;**********************************************************************

#INCLUDE UART-Subroutine.inc	;UART SUBROUTINES

;**********************************************************************
; COMPARATOR SUBROUTINES
;**********************************************************************

CAP_Charge:
		BANK1
        JIFEQ   Current_Axis, 1, CapC1
        JIFEQ   Current_Axis, 2, CapC2
        JIFEQ   Current_Axis, 3, CapC3
        JIFEQ   Current_Axis, 4, CapC4
		;set here an error code if did not match any.
CapC1: 
		bsf		CAP_1
;		bsf		CAP_2

        return 

CapC2: 
		bsf		CAP_2
        return 

CapC3: 
		bsf		CAP_3
        return 

CapC4: 
		bsf		CAP_4
        return 

;---------------------------------------------------------------------

CAP_Discharge:

		BANK1
		bcf		CAP_1
		bcf		CAP_2
		bcf		CAP_3
		bcf		CAP_4
		return

;---------------------------------------------------------------------

SET_Channel:
	;select next input channel for comp to match current axis cap.
	;0:00(pin 18), 1:01(pin 15), 2:10(pin 14), 3:11(pin 7)

		BANK2
        JIFEQ   Current_Axis, 1, Comp1
        JIFEQ   Current_Axis, 2, Comp2
        JIFEQ   Current_Axis, 3, Comp3
        JIFEQ   Current_Axis, 4, Comp4
		;set here an error code if did not match any.

Comp1: 
		bcf		CM2CON0, C2CH0	
;		bsf		CM2CON0, C2CH0	;	
		bcf		CM2CON0, C2CH1  

        return 

Comp2: 
		bsf		CM2CON0, C2CH0	
		bcf		CM2CON0, C2CH1
        return 
Comp3: 
		bcf		CM2CON0, C2CH0	
		bsf		CM2CON0, C2CH1
        return 

Comp4: 
		bsf		CM2CON0, C2CH0	
		bsf		CM2CON0, C2CH1
        return 

;---------------------------------------------------------------------
;SAVE_Pos:
STORE_Count:
		BANK0
        JIFEQ   Current_Axis, 1, Cnt1
        JIFEQ   Current_Axis, 2, Cnt2
        JIFEQ   Current_Axis, 3, Cnt3
        JIFEQ   Current_Axis, 4, Cnt4
		;set here an error code if did not match any.
Cnt1: 
		movf	TMR1H, W	;save high byte
		movwf	Count_1H 
		movf	TMR1L, W	;save low byte
		movwf	Count_1L
       return 

Cnt2: 
		movf	TMR1H, W	;save high byte
		movwf	Count_2H 
		movf	TMR1L, W	;save low byte
		movwf	Count_2L
        return 

Cnt3: 
		movf	TMR1H, W	;save high byte
		movwf	Count_3H 
		movf	TMR1L, W	;save low byte
		movwf	Count_3L
        return 

Cnt4: 
		movf	TMR1H, W	;save high byte
		movwf	Count_4H 
		movf	TMR1L, W	;save low byte
		movwf	Count_4L
        return 

;---------------------------------------------------------------------
Stop_Pot_Sensing:


	banksel	T1CON			;bank 0
	bcf		T1CON, TMR1ON	;stop timer 1

	banksel	PIE2
	bcf		PIE2, C2IE		;disable comp 2 interrupt

	;discharge all capacitors (interrupts should be disbled first)
	call CAP_Discharge

	return

;---------------------------------------------------------------------
Start_Pot_Sensing: 

	call SET_Channel	;select next input channel to measure

	;wait mux settling time (around 10 us)
	movlw	3
	movwf	Delay1  
	decfsz	Delay1,f
	goto	$-1	


;\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
	;clear Comp2 interrupt flag
	banksel PIR2
	bcf	PIR2, C2IF 
;////////////////////////////////////////////

	;clear timer 1
	banksel	TMR1L
	clrf	TMR1L		;clear LSB
	clrf	TMR1H		;clear MSB

	;remove any comp mistmatch (could triger an initial interrupt) 
	banksel CM2CON0			;bank 2
	movf	CM2CON0, W		;reading this register clears comp change mismatch


	;enable Comp2 interrupts
	banksel	PIE2
	bsf		PIE2, C2IE		;enable comp 2 interrupt

	;allow next capacitor charge
	call CAP_Charge

	;start timer 1
	banksel	T1CON			;bank 0
	bsf		T1CON, TMR1ON	;

	return

;---------------------------------------------------------------------



;ONLY FOR TESTING JOYSTICK
;print 4-axis counters to UART
;see also DisplPos macro - Display_Pos:

Display_Pos:

	DisplPos Count_1H
	DisplPos Count_1L

    movlw  ' '				;space
    call send 

	DisplPos Count_2H
	DisplPos Count_2L

    movlw  ' '				;space
    call send 

	DisplPos Count_3H
	DisplPos Count_3L

    movlw  ' '				;space
    call send 

	DisplPos Count_4H
	DisplPos Count_4L

    movlw  ' '				;space
    call send 

	;send line separator for formating
	movlw  0x0D ; CR 
	call send
	
	return


;**********************************************************************
; SERVO SUBROUTINES
;**********************************************************************

SERVO_Start:
		BANK0
        JIFEQ   nextServoPin, 1, Servo1
        JIFEQ   nextServoPin, 2, Servo2
        JIFEQ   nextServoPin, 3, Servo3
        JIFEQ   nextServoPin, 4, Servo4
        JIFEQ   nextServoPin, 5, Servo5
        JIFEQ   nextServoPin, 6, Servo6
        JIFEQ   nextServoPin, 7, Servo7
        JIFEQ   nextServoPin, 8, Servo8
		;set here an error code if did not match any.
		return	;Do nothing. servoCount was NOT between 1 and 8

Servo1: 
		bsf	SERVO_1
        return 

Servo2: 
		bsf	SERVO_2
        return 

Servo3: 
		bsf	SERVO_3
        return 

Servo4: 
		bsf	SERVO_4
        return 

Servo5: 
		bsf	SERVO_5
        return 

Servo6: 
		bsf	SERVO_6
        return 

Servo7: 
		bsf	SERVO_7
        return 

Servo8: 
		bsf	SERVO_8
        return 


;---------------------------------------------------------------------
SERVO_Stop:
	BANK0
	bcf	SERVO_1
	bcf	SERVO_2
	bcf	SERVO_3
	bcf	SERVO_4
	bcf	SERVO_5
	bcf	SERVO_6
	bcf	SERVO_7
	bcf	SERVO_8
	return


;**********************************************************************


;**********************************************************************


SERVO_Test1

	BANK0
;
	movlw	TIME_0.5ms_INT 	;left	 
	movwf	ServoPos
	DELAY
	DELAY
	DELAY
	DELAY
	movlw	TIME_1.5ms_INT 	;middle	 
	movwf	ServoPos
	DELAY
	DELAY
	DELAY
	DELAY
	movlw	TIME_2ms_INT 	;right	 
	movwf	ServoPos
	DELAY
	DELAY
	DELAY
	DELAY
	movlw	TIME_1.5ms_INT 	;middle	 
	movwf	ServoPos
	DELAY
	DELAY
	DELAY
	DELAY
	return
;**********************************************************************
SERVO_Test2

	BANK0
;
	movlw	TIME_0.5ms_INT 	;left	 
	movwf	servo1pos
	movwf	servo2pos
	movwf	servo3pos
	movwf	servo4pos
	movwf	servo5pos
	movwf	servo6pos
	movwf	servo7pos
	movwf	servo8pos
	DELAY
	DELAY
	movlw	TIME_1.5ms_INT 	;middle	 
	movwf	servo1pos
	movwf	servo2pos
	movwf	servo3pos
	movwf	servo4pos
	movwf	servo5pos
	movwf	servo6pos
	movwf	servo7pos
	movwf	servo8pos

	DELAY
	DELAY
	movlw	TIME_2ms_INT 	;right	 
	movwf	servo1pos
	movwf	servo2pos
	movwf	servo3pos
	movwf	servo4pos
	movwf	servo5pos
	movwf	servo6pos
	movwf	servo7pos
	movwf	servo8pos

	DELAY
	DELAY
	movlw	TIME_1.5ms_INT 	;middle	 
	movwf	servo1pos
	movwf	servo2pos
	movwf	servo3pos
	movwf	servo4pos
	movwf	servo5pos
	movwf	servo6pos
	movwf	servo7pos
	movwf	servo8pos

	DELAY
	DELAY
	return
;**********************************************************************
;**********************************************************************

	END