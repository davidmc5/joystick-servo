; This file needs to be included at the end of the end of the .asm file, 
; just before the 'END' statement with the following command:
; #INCLUDE UART-Subroutine.inc

;Requires the following variable in unbanked register CBLOCK 070h:
;
;UART_Temp	;Variable for UART subroutine

;**********************************************************************
;UART SUBROUTINES
;**********************************************************************

; ------------------------------------------- 
; UART RECEIVE AND SAVE CHARACTER INTO W 
; ------------------------------------------- 
; This routine does not return until a character is received. 
; 
receive	
		banksel	PIR1
		btfss 	PIR1,RCIF         ; check for received data 
        goto 	$-1

		banksel RCREG
        movf 	RCREG,W            ; save received data in W 
        return 
; 
; ------------------------------------------------------------- 
; UART TRANSMIT CHARACTER IN  W  
; ------------------------------------------------------------- 
; 
send    
		banksel TXREG			; bank 0
		movwf TXREG             ; put character in W into TX register

		banksel TXSTA 			;bank 1         	 
		btfss TXSTA,TRMT        ; wait for 1 => character transmission is complete.
        goto $-1
        return 
	

;**********************************************************************
; Conversion HEX to ASCII
;**********************************************************************

ascii_lookup
        ANDLW   0Fh     	;Strip off upper half byte which will be used as the table offset
		movwf	UART_Temp	;save it temporarily since we need to use W

		MOVLW	HIGH Table	;get high address of table
		MOVWF	PCLATH		;and load the high bits of the program counter buffer	

		MOVLW	LOW	Table	;get low byte of table address
		addwf	UART_Temp,w	; and add the offset
		btfsc	STATUS,C	;if low table address plus offset rolled over 255 (genertated carry)...
		incf	PCLATH,f	;increment the high address
		movwf	PCL			;and jump to the offset position in the table

Table:
        RETLW   '0'  
        RETLW   '1' 
        RETLW   '2'      
        RETLW   '3'          
        RETLW   '4'     
        RETLW   '5'       
        RETLW   '6'        
        RETLW   '7'   
        RETLW   '8'     
        RETLW   '9'  
        RETLW   'A'       
        RETLW   'B'    
        RETLW   'C'
        RETLW   'D'  
        RETLW   'E'  
        RETLW   'F'   

;**********************************************************************
