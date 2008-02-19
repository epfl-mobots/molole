.include "p33Fxxxx.inc"


;	Molole - Mobots Low Level library
;	An open source toolkit for robot programming using DsPICs
;	
;	Copyright (C) 2008 Stephane Magnenat <stephane at magnenat dot net>,
;	Mobots group (http://mobots.epfl.ch), Robotics system laboratory (http://lsro.epfl.ch)
;	EPFL Ecole polytechnique federale de Lausanne (http://www.epfl.ch)
;	
;	See AUTHORS for more details about other contributors.
;	
;	This program is free software: you can redistribute it and/or modify
;	it under the terms of the GNU General Public License as published by
;	the Free Software Foundation, either version 3 of the License, or
;	(at your option) any later version.
;
;	This program is distributed in the hope that it will be useful,
;	but WITHOUT ANY WARRANTY; without even the implied warranty of
;	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;	GNU General Public License for more details.
;	
;	You should have received a copy of the GNU General Public License
;	along with this program.  If not, see <http://www.gnu.org/licenses/>.



; assembler file for the HSYNC interrupt
.global	__T4Interrupt
__T4Interrupt:
; /!\ interrupt nesting, push.s allowed only once !
		push.s
		push w4
		bclr 	IFS1,#T4IF		; clear interrupt flag
		clr 	TMR4			; clear timer

		nop

		mov __po3030k_buffer, w1
		mov #__po3030k_line_conf, w2
		mov #1, w3				; used for comparaisons
		mov __po3030k_port, w4
		cp0 __po3030k_slow_path
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
;		lsr w0,#8,w0	
		nop
		mov.b w0,[w1++]	
		bra restart_loop

end_line:
		mov w1, __po3030k_buffer
		inc __po3030k_current_row
		mov __po3030k_current_row,w0
		cp __po3030k_row
		bra NZ, go_out
		; tell others we are ready
		mov #1, w0
		mov w0, __po3030k_img_ready		
		; disable ourself by calling a C function ! Dirty hack, but works(tm)
		pop w4
		pop.s 
		goto __po3030k_disable_hsync
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
;		lsr w0,#8,w0	
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

