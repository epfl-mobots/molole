.include "p33fxxxx.inc"

; assembler file for the HSYNC interrupt
.global	__T4Interrupt
__T4Interrupt:
; /!\ interrupt nesting, push.s allowed only once !
		push.s
		push w4

		bclr 	IFS1,#T4IF		; clear interrupt flag
		clr 	TMR4			; clear timer

		; for fine syncronisation
		nop


		mov __po6030k_buffer, w1
		mov #__po6030k_line_conf, w2
		mov #1, w3				; used for comparaisons
		mov __po6030k_port, w4
		cp0 __po6030k_slow_path
		bra NZ, slow_path

restart_loop:
		cp.b w3,[w2++]	
		bra Z, take_it 	; if w2 == 1		
		bra LT, end_line ; if w2 == 2
		; w2 == 0
		nop
		nop
		nop
		bra restart_loop
take_it:
		mov.b [w4], w0
		nop
		mov.b w0,[w1++]	
		bra restart_loop

end_line:
		mov w1, __po6030k_buffer
		inc __po6030k_current_row
		mov __po6030k_current_row,w0
		cp __po6030k_row
		bra NZ, go_out
		; tell others we are ready
		mov #1, w0
		mov w0, __po6030k_img_ready	
		; disable ourself by calling a C function ! Dirty hack, but works(tm)
		pop w4
		pop.s
		goto __po6030k_disable_hsync	
		
go_out:
		pop w4
		pop.s
		retfie
		
slow_path:
restart_loop_slow:
		cp.b w3,[w2++]	
		bra Z, take_it_slow 	; if w2 == 1		
		bra LT, end_line ; if w2 == 2
		; w2 == 0
		nop
		nop
		nop

		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop

		bra restart_loop_slow
take_it_slow:
		mov.b [w4], w0
		nop
		
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop

		mov.b w0,[w1++]	
		bra restart_loop_slow

