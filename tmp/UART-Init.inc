; This file needs to be included at the start of the main program, 

; after the interrupt code at org 04H of the .asm file,
; and after the initial configuration settings and ; with the following command:
; #INCLUDE UART-Init.inc

;Requires the following variables in unbanked register CBLOCK 070h:

;UART_Temp	;Variable for UART subroutine 
;Delay1 

;************************************************************************************************* 
; UART INITIALIZATION
;************************************************************************************************* 

	; Configure Baud Rate = 9600, No Parity, 1 Stop Bit 
	banksel SPBRG		; Bank 1
    movlw	25d         ; 9600 bps 
    movwf	SPBRG		; we only need the lower two bytes for 25 (SPBRGH not used)

	; configure transmiter (still in bank 1)
	bcf	TXSTA, SYNC	; Asynchronous mode
	bsf	TXSTA, BRGH	; High baud rate
	bsf	TXSTA, TXEN	; transmit enabled
	bcf	TXSTA, TX9	; 8-bit transmission (= no parity and 1 stop bit)
	bcf	BAUDCTL, BRG16	; using 8 bits (SPBRG) for encoding baud rate. Ignore SPBRGH bits.


	; configure receiver
    banksel	RCSTA		; bank 0
	bcf		RCSTA, SYNC	; set asynchronous mode
	bsf		RCSTA, CREN	; enable reception
	bsf		RCSTA, SPEN	; enable usart module and set RX pin as input and TX pin as output

	; wait until uart is settled
    clrf 	Delay1 
	decfsz 	Delay1,F 
    goto 	$-1

    ; flush receive buffer (FIFO buffer holds only two characters. Flush 3 times to clear)
	banksel	RCREG
    movf 	RCREG,W 
    movf 	RCREG,W 
    movf 	RCREG,W

	;dissable UART interrupts. currently using loops instead of interrupts.
	banksel	PIE1		;bank 1
	bcf		PIE1, TXIE	;mask out transmit interrupt
	bcf		PIE1, RCIE	;mask out receive interrupt
	banksel	PIR1		;bank 0
	bcf		PIR1, TXIF	;clear TX interrupt flag
	bcf		PIR1, RCIF	;clear RX interrupt flag
	
