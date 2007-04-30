/*

CAN bootloader for PIC 24/33
This file is part of Molole (Mobots low level library)
Copyright (C) 2006 - 2007 Philippe Rétornaz <philippe dot retornaz at epfl dot ch>
Copyright (C) 2007 Valentin Longchamp <valentin dot longchamp at a3 dot epfl dot ch>
Copyright (C) 2007 Stéphane Magnenat <stephane at magnenat dot net>
Laboratory of Robotics Systems 1 - EPFL

See authors.txt in top level directory for more details about other contributors.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, 
USA.

*/

.include cpuinc

; PIC have SRAM, datasheet say that it keep data if Vdd > 2.8V
; Moreover a PDF by microchip say the ram is untouched from a software 
; reset or a MCLR reset 

config __FOSCSEL, FNOSC_FRCPLL ; FRC Oscillator with PLL

; Clock Switching and Fail Safe Clock Monitor is disabled 
; Primary Oscillator Mode: Disabled
; OSC2 Pin Function: OSC2 is Clock Output
config __FOSC, FCKSM_CSDCMD & OSCIOFNC_OFF  & POSCMD_NONE

; 40 MHz CPU, if you change this, be sure to check CAN timing configurations
; (can_c1cfg1_conf and can_c1cfg2_conf)
.equ FCQ, 40000000

; page size ( in nb of instructions )
.equ FLASH_PAGE_INSTRUCTION_COUNT, 512
; page size ( in stupid dspic address )
.equ FLASH_PAGE_SIZE, 1024

.equ FLASH_ERASE_PAGE_CODE, 0x4042
.equ FLASH_PROG_ROW_CODE, 0x4001

; row size ( in nb of instructions )
.equ FLASH_ROW_INSTRUCTION_COUNT, 64

.equ can_config_mode, 0x400
.equ can_normal_mode, 0x0

.equ can_answer_OK, 0x0
.equ can_answer_INVALID_SIZE, 0x1
.equ can_answer_PROGRAM_FAILED, 0x2
.equ can_answer_NOT_PROGRAMMING, 0x3

.equ can_CMD_RESET, 0x8000			; ask the bootloader to reset
.equ can_CMD_READ_PAGE, 0x8001		; ask the bootloader to read a page and send it back
.equ can_CMD_WRITE_PAGE, 0x8002		; ask the bootloader to write a page
.equ can_CMD_PAGE_DATA, 0x8003		; CAN message containing data from host
.equ can_PUSH_DESCRIPITON, 0x8004	; CAN message containing bootloader description
.equ can_PUSH_PAGE_DATA, 0x8005		; CAN message containing data to host
.equ can_PUSH_ACK, 0x8006			; CAN message containing return value

.equ can_MSG_SHORT, 0x300
.equ can_MSG_PHY_TYPE_MASK, 0x700

.equ BOOTLOADER_VERSION, 0x1

.bss

; Workaround bug, so we can see the other variables when debugging with ICD2
_workaround_mplab_ICD2_bug:	.space 1024

; Memory for a page when flashing
page_buf: 		.space FLASH_PAGE_INSTRUCTION_COUNT * 4 

; Additional variables
can_sid:		.space 2
page_lowptr:	.space 2

; CAN buffer in DMA area
.section canbuf,bss,dma
can_buf:		.space 64

.text
.global __reset

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Configuration page                   ;
; DO NOT CHANGE THE ORDER AND OFFSETS  ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.section confpage,code,address(FLASH_END - FLASH_PAGE_SIZE - FLASH_PAGE_SIZE)

configuration_page:

can_id_conf:
.word NODE_ID

can_c1cfg1_conf:
; SWJ = 11 (4x Tq)
; BRP = 000000 (Tq = (2 * (0 + 1)) / Fcan))
; Fcan MUST be 40 MHz
.word 0x00C0

can_c1cfg2_conf:
; WAKFIL = 1 (CAN bus line can be used for wakeup)
; SEG2PH = 101 (Phase 2 length is 6x Tq)
; SEG2PHTS = 1 (Phase 2 time fully programmable)
; SAM = 1 (bus line is sampled three times)
; SEG1PH = 100 (Phase 1 length is 5x Tq)
; PRSEG = 100 (Prop seg is 5x Tq)
; Sync + Prop + Phase 1 + Phase 2 = 20 Tq => 1 Mbps
.word 0x45E4

can_enable_latch_addr:
; LATD ( TRIS == LAT - 4 )
.word 0x02D6

can_enable_mask:
; bit 4 is cleared
.word 0xFFEF

pll_pllfbd_conf:
; PLLDIV = M = 130
.word 0x0080

pll_clkdiv_conf:
; ROI = 0, interrupts have no effect on the DOZEN bit
; DOZE = 0, processor clock reduction ratio = 1
; DOZEN = 0 no processor clock reduction
; FRCDIV = 0 , FRC divided by 1
; PLLPOST = 0, N2 = 2
; PLLPRE = 4, N1 = 6
.word 0x0004

pll_osctun_conf:
; FRC internal = 7.3728
.word 0

low_wait_conf:
.word 65535
high_wait_conf:
.word (FCY/(65536*6))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Bootloader Code start
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; w0, w1, w2: scratch registers
; w3: CAN TX frame ptr
; w4: CAN RX frame ptr
; w5: page ptr in data memory
; w6: current state
; w7: number of received packets
; w8, w9, w10: scratch register
; w11: reserved for flashing operation

.section bootloader,code,address(FLASH_END - FLASH_PAGE_SIZE)
__reset:
	; Setup stack pointer
	mov #__SP_init, W15
	mov #__SPLIM_init, W0
	mov W0, SPLIM
	nop
	nop
	
	; Init pll, get the conf from program memory
	mov #tblpage(pll_pllfbd_conf), w0
	mov w0, _TBLPAG
	mov #tbloffset(pll_pllfbd_conf), w0
	tblrdl [w0++], w1 				; pllfbd
	mov w1, PLLFBD
	tblrdl [w0++], w1				; clkdiv
	mov w1, CLKDIV
	tblrdl [w0] , w1 				; osctun
	mov w1, OSCTUN
	
	; Wait on pll stabilisation
pll_stab_init:
	btss OSCCON,  #5
	bra pll_stab_init
	
	; Init CAN
	mov #can_config_mode, w0
	rcall setup_can_runlevel
	
	bset C1CTRL1, #11 		; Fcan = FCY
	
	mov #tblpage(can_enable_latch_addr), w0
	mov w0, _TBLPAG
	mov #tbloffset(can_enable_latch_addr), w0
	tblrdl [w0++], w1 		; can latch addr
	tblrdl [w0], w2			; can bit mask
	mov [w1], w8			; get the latch value
	and w8, w2, w8
	mov w8, [w1]			; clear the correct LATCH bit according to mask
	sub #4, w1			; get the tris addr
	mov [w1], w8
	and w8, w2, w8
	mov w8, [w1]			; clear the correct TRIS bit according to mask
	
	; Load CAN config
	mov #tblpage(can_id_conf), w0
	mov w0, _TBLPAG
	mov #tbloffset(can_id_conf), w0
	tblrdl [w0++], w1
	
	; Build physical CAN sid
	ior #can_MSG_SHORT, w1
	com w1, w1
	sl w1, #2, w1			; standard configuration, normal frame
	mov w1, can_sid			; can SID
	
	; Load others CAN configuration registers
	tblrdl [w0++] ,w1 		; C1CFG1
	mov w1, C1CFG1
	tblrdl [w0], w1 		; C1CFG2
	mov w1, C1CFG2
	
	; Configure Filter 
	bset C1CTRL1, #0 		; C1CTRL1.WIN = 1
	
	; Enable only Filter 0
	clr C1FEN1
	bset C1FEN1, #0
	
	; Set Receive Filter for SHORT MESSAGES
	mov #can_MSG_SHORT, w0
	com w0, w0
	sl w0, #5, w0			; EXIDE bit is cleared by sl #5
	mov w0, C1RXF0SID

	; Clear EID Filter
	mov #0, w0
	mov w0, C1RXF0EID
	
	; Set Receive Mask
	mov #can_MSG_PHY_TYPE_MASK, w0
	sl w0, #5, w0			; MIDE bit is cleared by sl #5
	bset w0, #3			; so we set MIDE
	mov w0, C1RXM0SID

	; Clear EID Mask
	mov #0, w0
	mov w0, C1RXM0EID

	; Use Mask 0 with Filter 0
	bclr C1FMSKSEL1, #0
	bclr C1FMSKSEL1, #1
	
	; Configure Receive and Transmit Buffers

	; When Filter 0 hits, fill buffer 1
	mov #1, w0		
	mov w0, C1BUFPNT1
			
	; Buffer 0 is a TX Buffer (and buffer 1 RX)
	mov #0x0080, w0
	mov w0, C1TR01CON
	
	; 4 can buffer in DMA
	clr C1FCTRL
	
	bclr C1CTRL1, #0	; C1CTRL1.WIN = 0
	
	clr C1RXFUL1
	clr C1RXOVF1
	
	; Configure DMA channels
	mov #0x0020, w0
	mov w0, DMA0CON
	mov #C1RXD, w0
	mov w0, DMA0PAD
	mov #7, w0
	mov w0, DMA0CNT
	mov w0, DMA1CNT
	mov #34, w0
	mov w0, DMA0REQ
	mov #dmaoffset(can_buf), w0
	mov w0, DMA0STA
	mov w0, DMA1STA
	mov #0x2020, w0
	mov w0, DMA1CON
	mov #C1TXD, w0
	mov w0, DMA1PAD
	mov #70, w0
	mov w0, DMA1REQ
	bset DMA1CON, #15
	bset DMA0CON, #15

	; Prepare can TX frame header
	mov #can_buf, w3
	mov can_sid, w0
	mov w0, [w3]		; sid
	clr w0
	mov w0, [w3+2]		; eid

	add w3,#16,w4		; setup can RX ptr
	clr w6				; status == 0

	clr C1INTF
	
	; Enable CAN "interrupt" ( but not enabled in interrupt controller )
	bset C1INTE, #1
	bset C1INTE, #0

	; Initialization phase complete !
	mov #can_normal_mode, w0
	rcall setup_can_runlevel
	
	; Send description
	
	; Set DLC
	mov #8, w1
	mov w1, [w3+4]
	
	; Set aseba message type
	mov #can_PUSH_DESCRIPITON, w1
	mov w1, [w3+6]	 ; set data[0]
	
	; Set page size
	mov #(FLASH_PAGE_INSTRUCTION_COUNT * 4), w1
	mov w1, [w3+8]	 ; set data[1]
	
	; Set page start
	mov #0, w1
	mov w1, [w3+10]	 ; set data[2]
	
	; Set page count
	mov #((FLASH_END / FLASH_PAGE_SIZE) - 2), w1	; we forbide bootloader overwriting
	mov w1, [w3+12]	 ; set data[3]
	
	; Send description
	rcall send_can_packet

	; Setup timeout
	; one non-successfull loop need 6 cycles
	; ( each 65536 loop, it need 4 more cycles )
	; 40Mips, 1 sec == 40'000'000 cycles
	; FCY/(6 * 65536 + 4*x) = x => 4x*x + 6*65536x - FCY = 0
	; since assembler doesn't know the sqrt() operator, we 
	; ignore the "4 more cycles"
	; See the confpage definition.
	mov #tblpage(low_wait_conf), w2
	mov w2, _TBLPAG
	mov #tbloffset(low_wait_conf), w2
	tblrdl [w2++], w0
	tblrdl [w2++], w1

	; Wait on incoming CAN packet ( RX buffer 1 )
wait_loop:
	btsc C1RXFUL1, #1		; 2
	bra got_can_packet
	dec w0, w0				; 1
	bra z, out_cnt			; 1 ( 2 )
	bra wait_loop			; 2
out_cnt:
	dec w1,w1				; 1
	bra Z, timeout			; 1
	bra wait_loop			; 2
	
timeout:
	reset

	; Depending on aseba type od incoming packet, we take a different action
got_can_packet:
	; Is it for us ?
	mov [w4+8], w0
	cp w0, #NODE_ID
	bra NZ, wait_can_packet
	
	; If yes, parse aseba message type
	mov [w4+6], w0
	
	mov #can_CMD_PAGE_DATA, w1
	cp w0, w1
	bra Z, cmd_page
	
	mov #can_CMD_RESET, w1
	cp w0, w1
	bra Z, cmd_reset
	
	mov #can_CMD_READ_PAGE, w1
	cp w0, w1
	bra Z, cmd_readpage
	
	mov #can_CMD_WRITE_PAGE, w1
	cp w0, w1
	bra Z, cmd_writepage

	; Others aseba messages are ignored
	bra wait_can_packet

;;;;;;;;;;;;
; Reset Command
;;;;;;;;;;;;
cmd_reset:
	mov #can_answer_OK, w0
	rcall send_error_code
	reset

;;;;;;;;;;;;
; Readpage Command
;;;;;;;;;;;;
cmd_readpage:
	; Check DLC
	mov [w4 + 4], w0
	subr w0, #6 ,w0
	bra NZ, error_dlc
	
	; Page number from data[2]
	mov [w4 + 10], w0
	
	; Page address high 8 bits ((pageNumber<< 10) >> 16)
	lsr w0, #6, w1
	mov w1, TBLPAG
	
	; Page address low 16 bits to w2 ((pageNumber<< 10) & 0xFFFF)
	sl w0, #10, w2
	
	; Set DLC
	mov #6, w0
	mov w0, [w3 + 4]
	
	; Set aseba message type
	mov #can_PUSH_PAGE_DATA, w0
	mov w0, [w3 + 6]	; set data[0]
	
	; Loop boundary
	mov #FLASH_PAGE_INSTRUCTION_COUNT, w7
	
readpage_loop:
	; Set w8 points to data[1]
	mov w3, w8
	add w8, #8, w8
	
	; Instruction
	tblrdl [w2], [w8++]
	tblrdh [w2++], [w8++]
	
	; Send instruction
	rcall send_can_packet
	
	; If not sent all instructions yet, loop
	dec w7, w7
	bra NZ, readpage_loop
	
	; Everything went fine
	mov #can_answer_OK, w0
	rcall send_error_code
	
	bra wait_can_packet

error_dlc:
	mov #can_answer_INVALID_SIZE, w0
	rcall send_error_code
	bra wait_can_packet

;;;;;;;;;;;;
; Write page Command
;;;;;;;;;;;;
cmd_writepage:
	; Check DLC
	mov [w4 + 4], w0
	subr w0, #6, w0
	bra NZ, error_dlc
	
	; Page number from data[2]
	mov [w4 + 10], w0
	
	; Page address high 8 bits ((pageNumber << 10) >> 16)
	lsr w0, #6, w1
	mov w1, TBLPAG
	
	; Page address low 16 bits to w2 ((pageNumber << 10) & 0xFFFF)
	sl w0, #10, w2
	
	; Go into page mode
	mov #FLASH_PAGE_INSTRUCTION_COUNT, w7		; number of times cmd_page should be called per page
	mov w2, page_lowptr
	mov #page_buf, w5

	bra wait_can_packet


;;;;;;;;;;;;
; Page loading command
;;;;;;;;;;;;
cmd_page:
	; Check if we are actually programming something
	cp w7, #0
	bra Z, error_not_prog
	
	; Read the next 4 bytes
	mov [w4+10], w0		; from data[2]
	mov w0, [w5++]
	mov [w4+12], w0		; from data[3]
	mov w0, [w5++]
	
	; If we have not finished page
	dec w7, w7
	bra NZ, wait_can_packet
	
	; If we have read the whole page
	rcall do_flash_page
	cp w0, #0
	bra Z, prog_ok
	
	; Set DLC
	mov #6, w2
	mov w2, [w3+4]
	
	; Set aseba message type
	mov #can_PUSH_ACK, w2
	mov w2, [w3+6]	 	; set data[0]
	
	; Set error code
	mov w0, can_answer_PROGRAM_FAILED
	mov w0, [w3+8]  ; set data[1]
	mov w1, [w3+10] ; set data[2]
	
	rcall send_can_packet
	bra wait_can_packet

prog_ok:
	mov #can_answer_OK, w0
	rcall send_error_code
	bra wait_can_packet

error_not_prog:
	mov #can_answer_NOT_PROGRAMMING, w0
	rcall send_error_code
	bra wait_can_packet

; programm a flash page ( 512 instr. )
; and verify it's correct
; in: nothing
; out: w0: the result
;      w1: the failing low address ( if failed )
; modify: w0,w1,w2,w5,w9,w10,w8
do_flash_page:
	; Erase the page
	
	; Setup flash for erase operation
	mov #FLASH_ERASE_PAGE_CODE, w0
	mov w0, NVMCON

	; Dummy write to select the page to erase
	mov page_lowptr, w0
	tblwtl w0, [w0]
	
	; Do the real flash page erase
	rcall do_key_seq

	; Prepare to write rows
	
	; Setup flash for row write
	mov #FLASH_PROG_ROW_CODE, w0
	mov w0, NVMCON
	
	; Setup pointers
	mov page_lowptr, w11
	mov #(FLASH_PAGE_INSTRUCTION_COUNT / FLASH_ROW_INSTRUCTION_COUNT), w10
	mov #page_buf, w5
	
dfp_prog_loop_page:
	; Row instruction counter
	mov #FLASH_ROW_INSTRUCTION_COUNT, w9
	
dfp_prog_loop_row:
	; Write into flash from buffer
	tblwtl [w5++], [w11]
	tblwth [w5++], [w11++]

	; Decrement and check row counter
	dec w9, w9
	bra NZ, dfp_prog_loop_row
	
	; Do the real flash write
	rcall do_key_seq
	
	; Decrement and check page counter
	dec w10, w10
	bra NZ, dfp_prog_loop_page
	
; now verify the whole thing
	mov #page_buf, w5	; reset page buf ptr
	mov page_lowptr, w0

	mov #FLASH_PAGE_INSTRUCTION_COUNT, w10

dfp_verify_loop:
	; Compare low word
	mov [w5++], w1
	tblrdl [w0], w2
	cpseq w1, w2
	bra dfp_error_prog
	
	; Compare high byte
	mov [w5++], w1
	tblrdh [w0++], w2
	cpseq w1, w2
	bra dfp_error_prog
	
	; Decrement and check instruction counter
	dec w10, w10
	bra NZ, dfp_verify_loop
	
	retlw #can_answer_OK, w0
	
dfp_error_prog:
	dec w0, w1
	retlw #can_answer_PROGRAM_FAILED, w0



; Do the key sequence and start the flash programming
; in: nothing
; out: nothin
; modify: w0,w1
do_key_seq:
	disi #5
	mov #0x55, w0
	mov w0, NVMKEY
	mov #0xaa, w1
	mov w1, NVMKEY
	bset NVMCON, #WR
	nop
	nop
	return


;;;;;;;;;;;;
; Utilities functions
;;;;;;;;;;;;

; Wait until a new CAN packet arrives
wait_can_packet:
	bclr C1RXFUL1, #1
_wait_can_packet:
	btss C1RXFUL1, #1
	bra _wait_can_packet
	bra got_can_packet


; Send an error code on can
; in: w0: error code
; out: nothing
; modify w1
send_error_code:
	; Set DLC
	mov #4, w1
	mov w1, [w3+4]
	
	; Set aseba message type
	mov #can_PUSH_ACK, w1
	mov w1, [w3+6]	 ; set data[0]
	
	; Set error code
	mov w0, [w3+8]  ; set data[1]


; set the TX buffer as ready to send
; and wait it's done
; modify: nothing
send_can_packet:
	bset C1TR01CON, #3
_sec_loop:
	btsc C1TR01CON, #3 ; wait until the frame is sent
	bra _sec_loop
	return


; ask can runlevel
; in: w0: runlevel shifted by 8 bits
; out: nothing
; modify: w0, w1, w2
setup_can_runlevel:
	mov C1CTRL1, w1
	mov #0xF8FF,w2
	and w1,w2,w1
	ior w1,w0,w1
	mov w1, C1CTRL1
	mov #0xE0, w2
	lsr w0,#3,w0
_setup_can_runlevel_loop:
	mov C1CTRL1, w1
	and w1,w2,w1
	sub w1,w0,w1
	bra NZ, _setup_can_runlevel_loop
	return;
