;Joystick_Int.inc
;============================================================
;COMPARATOR 2
;============================================================

;this interrupt is used to measure the time of the RC of the joystick axis

;1) read pin counter
;2) update correct counter with timer 1
;3) discharge current cap
;4) increment pin counter
;5) select next input pin
;6) start charging cap

	call STORE_Count	;save 2-byte timer count to axis registers
	call CAP_Discharge 	;timer count is saved. OK to discharge current capacitor

	decfsz	Current_Axis,F 	;select next axis PIN to measure
	 goto 	next_input 		;if axis sequence # is not zero
	movlw	Num_Axis		;if zero, reload the initial number of axis
	movwf	Current_Axis	;reset next pin number 


	; disable axis sensing after a complete loop
	; this will be reactivated by servo routine
;this is to allow timely servo pulse refreshes. 
;No need to read pots faster than servo refreshes.

	call Stop_Pot_Sensing
	goto Exit_Joystick_ISR

next_input:

	call Start_Pot_Sensing

Exit_Joystick_ISR:


;============================================================
