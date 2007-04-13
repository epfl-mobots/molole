.include cpuinc

; PIC have SRAM, datasheet say that it keep data if Vdd > 2.8V
; Moreover a PDF by microchip say the ram is untouched from a software 
; reset or a MCLR reset 

config __FOSCSEL, FNOSC_FRCPLL ; FRC Oscillator with PLL

; Clock Switching and Fail Safe Clock Monitor is disabled 
; Primary Oscillator Mode: Disabled
; OSC2 Pin Function: OSC2 is Clock Output
config __FOSC, FCKSM_CSDCMD & OSCIOFNC_OFF  & POSCMD_NONE

; page size ( in nb of instructions  )
.equ PAGE_SIZE, 512
; row size ( in nb of instructions )
.equ row_size, 64

; MUST be a multiple of page_size
.equ bootloadersize, PAGE_SIZE
; any size, as long as it's a multiple of 2
.equ confsize, row_size

.equ can_config_mode, 0x400
.equ can_normal_mode, 0x0

.equ can_answer_OK, 0x0
.equ can_answer_INVALID_SIZE, 0x1
.equ can_answer_INVALID_VALUE, 0x2
.equ can_answer_PROGRAM_FAILED, 0x3
.equ can_answer_NOT_PROGRAMMING, 0x4

.equ can_CMD_RESET, 0x8000		; ask the bootloader to reset
.equ can_CMD_READ_PAGE, 0x8001	; ask the bootloader to read a page and send it back
.equ can_CMD_WRITE_PAGE, 0x8002	; ask the bootloader to write a page
.equ can_CMD_WRITE_CONF, 0x8003	; ask the bootloader to write configs bits
.equ can_CMD_PAGE_DATA, 0x8004	; CAN message containing data from host
.equ can_PUSH_PAGE_DATA, 0x8005	; CAN message containing data to host
.equ can_PUSH_ACK, 0x8006		; CAN message containing return value

.equ can_STATE_CMD, 0x0
.equ can_STATE_PAGE, 0x1

.equ can_MSG_SHORT, 0x300
.equ can_MSG_PHY_TYPE_MASK, 0x700

.equ BOOTLOADER_VERSION, 0x1

.bss

; so we can see the other variables when debugging with ICD2
_workaround_mplab_ICD2_bug:	.space 1536

can_sid:		.space 2

; Memory for a page when flashing
page_buf: 		.space 1536
page_lowptr:	.space 2

.section canbuf,bss,dma
can_buf:		.space 64

.text
.global __reset

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Configuration page                   ;
; DO NOT CHANGE THE ORDER AND OFFSETS  ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.section confpage,code,address(FLASH_END - 0x200 - bootloadersize - confsize)
pll_pllfbd_conf:
.word 38 
pll_clkdiv_conf:
.word 0
pll_osctun_conf:
.word 23
can_sid_conf:
.word 0xa  ; configured for 700'000 Kbit/s
can_c1cfg1_conf:
.word 0xc0
can_c1cfg2_conf:
.word 0x7bf
low_wait_conf:
.word 65535
high_wait_conf:
.word (FCY/(65536*6))
can_enable_latch_addr:
.word 0x2D6 ; LATD ( TRIS == LAT - 4 )
can_enable_mask:
.word 0xFFEF ; bit 4

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

.section bootloader,code,address(FLASH_END - 0x200 - bootloadersize)
__reset:
; setup stack pointer
	mov #__SP_init, W15
	mov #__SPLIM_init, W0
	mov W0, SPLIM
	nop
	nop
	
; init pll
	; get the pll conf from program memory
	mov #tblpage(pll_pllfbd_conf), w0
	mov w0, _TBLPAG
	mov #tbloffset(pll_pllfbd_conf), w0
	tblrdl [w0++],w1 				; pllfbd
	mov w1, PLLFBD
	tblrdl [w0++],w1				; clkdiv
	mov w1, CLKDIV
	tblrdl [w0] ,w1 				; osctun
	mov w1, OSCTUN
; wait on pll stabilisation
pll_stab_init:
	btss OSCCON,  #5
	bra pll_stab_init
	
; init can
	mov #can_config_mode, w0
	rcall ask_can_runlevel
	
	bset C1CTRL1, #11 		; Fcan = FCY
	
	mov #tblpage(can_enable_latch_addr), w0
	mov w0, _TBLPAG
	mov #tbloffset(can_enable_latch_addr), w0
	tblrdl [w0++],w1 		; can latch addr
	tblrdl [w0], w2			; can bit mask
	mov [w1], w8			; get the latch value
	and w8,w2,w8
	mov w8,[w1]				; clear the correct LATCH bit according to mask
	sub #4,w1				; get the tris addr
	mov [w1],w8
	and w8,w2,w8
	mov w8,[w1]				; clear the correct TRIS bit according to mask
	
; load can config
	mov #tblpage(can_sid_conf), w0
	mov w0, _TBLPAG
	mov #tbloffset(can_sid_conf), w0
	tblrdl [w0++], w1
	mov w1, can_sid			; can SID
	
	tblrdl [w0++] ,w1 		; C1CFG1
	mov w1, C1CFG1
	tblrdl [w0], w1 		; C1CFG2
	mov w1, C1CFG2
	
; configure Filter 
	bset C1CTRL1, #0 		; C1CTRL1.WIN = 1
	
; enable only first filter
	clr C1FEN1
	bset C1FEN1, #0
	mov can_sid, w0
	sl w0, #5, w0
	mov #0xFFE0, w1
;	and w0,w1,w0
	mov w0, C1RXF0SID
	mov w1, C1RXM0SID
	
	bset C1RXM0SID, #3  	; Match only SID messages
	mov #1, w0				; filter 1 point RX buffer 1
	mov w0, C1BUFPNT1
	mov #0x0080, w0			; Buffer 0 TX, 1 RX
	mov w0, C1TR01CON	
	clr C1FCTRL				; 4 can buffer in DMA
	
	bclr C1CTRL1, #0		; C1CTRL1.WIN = 0
	
	clr C1RXFUL1
	clr C1RXOVF1
	

; configure DMA channels
	mov #0x0020, w0
	mov w0, DMA0CON
	mov #C1RXD,w0
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

; prepare can TX frame header
	mov #can_buf, w3
	mov can_sid, w0
	sl w0,#2,w0
	mov w0, [w3]		; sid
	clr w0
	mov w0, [w3+2]		; eid

	add w3,#16,w4		; setup can RX ptr
	clr w6				; status == 0

	clr C1INTF
; enable CAN "interrupt" ( but not enabled in interrupt controller )
	bset C1INTE, #1
	bset C1INTE, #0

; Initialization phase complete ! 
	mov #can_normal_mode, w0
	rcall ask_can_runlevel


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

; wait on incomming CAN packet ( RX buffer 1 )
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

; The main state machine
got_can_packet:
	; Is it for us ?
	mov [w4+8], w0
	cp w0, #NODE_ID
	bra NZ, wait_can_packet
	
	; If yes, parse aseba message type
	mov [w4+6], w0
	
	mov #can_CMD_PAGE_DATA, w1
	cp w0, w1
	bra Z, cmd_PAGE
	
	mov #can_CMD_RESET, w1
	cp w0, w1
	bra Z, cmd_reset
	
	mov #can_CMD_READ_PAGE, w1
	cp w0, w1
	bra Z, cmd_readpage
	
	mov #can_CMD_WRITE_PAGE, w1
	cp w0, w1
	bra Z, cmd_writepage
	
	mov #can_CMD_WRITE_CONF, w1
	cp w0, w1
	bra Z, cmd_writeconf

	; Others aseba messages
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
	
	; Set flash pointers
	mov [w4 + 10], w0	; page number from data[2]
	mov w0,TBLPAG
	
	; Set DLC
	mov #6, w0
	mov w0, [w3 + 4]
	
	; Set aseba message type
	mov #can_PUSH_PAGE_DATA, w0
	mov w0, [w3 + 6]	; set data[0]
	
	; Loop (PAGE_SIZE / 2) times
	mov #(PAGE_SIZE / 2), w1
	mov 0, w2
readpage_loop:
	mov w3, w8
	add w8, #8, w8		; w8 points to data[1]
	
	; First instruction
	tblrdh.b [w2],w0
	tblrdl [w2++], [w8++]
	mov.b w0, [w8++]
	
	; Second instruction
	tblrdh.b [w2],w0
	tblrdl [w2++], [w8++]
	mov.b w0, [w8]
	
	rcall send_can_packet
	dec w1, w1
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
	subr w0, #8, w0
	bra NZ, error_dlc
	
	; Set flash pointers
	mov [w4 + 10], w0	; high word from data[2]
	mov w0,TBLPAG
	mov [w4 + 12], w2	; low word from data[3]
	
	; Check if page size is divisible by 1024
	mov w2,w1
	lsr w1, #10, w1
	sl w1, #10, w1
	cpseq w1, w2
	bra error_inval
	
	; Go into page mode
	mov #384, w7		; number of times cmd_PAGE should be called per page (384 * 4 = 1536 = 512 * 3)
	mov #can_STATE_PAGE, w6	; state is now "page mode"
	mov w2, page_lowptr
	mov #page_buf, w5

	bra wait_can_packet

error_inval:
	mov #can_answer_INVALID_VALUE, w0
	rcall send_error_code
	bra wait_can_packet


;;;;;;;;;;;;
; Page loading command
;;;;;;;;;;;;
cmd_PAGE:
	; Read the next 4 bytes
	mov [w4+10], w0		; from data[2]
	mov w0, [w5++]
	mov [w4+12], w0		; from data[3]
	mov w0, [w5++]
	
	; If we have not finished page
	dec w7, w7
	bra NZ, wait_can_packet
	
	; If we have read the whole page
	; zzz we are HERE TODO FIXME
	rcall do_flash_page
	mov #can_STATE_CMD,w6
	cp w0,#0
	bra Z,return_state_page
	mov w0,[w3 + 6]
	mov w1,[w3 + 8]
	mov TBLPAG, w0
	mov w0,[w3 + 10]
	mov #6, w0
	mov w0,[w3 + 4]
	rcall send_can_packet
	bra wait_can_packet



; programm a flash page ( 512 instr. )
; and verify it's correct
; in: nothing
; out: w0: the result
;      w1: the failing low address ( if failed )
; modify: w0,w1,w2,w5,w9,w10,w8
do_flash_page:
; earse the page
	mov #0x4042, w0
	mov w0,NVMCON		; ask for erase operation

	mov page_lowptr, w0
	tblwtl w0,[w0]		; set base address
	rcall do_key_seq	; do it !

; now write it ( in chunk of 64 instr ) !
	mov #page_buf, w5 	; reset the page buf ptr
	mov #0x4001, w0
	mov w0,NVMCON		; ask for row write
	mov page_lowptr, w11
	mov #8, w10		; 64*8 = 512 couter

dfp_prog_loop_page:
	mov #64,w9		; 64 instr. counter
dfp_prog_loop_row:
	mov.b [w5++],w1
	mov.b [w5++],w2
	mov.b [w5++],w8
	tblwth.b w8,[w11]
	tblwtl.b w1,[w11++]
	tblwtl.b w2,[w11++]
	dec w9,w9
	bra NZ, dfp_prog_loop_row
	rcall do_key_seq
	dec w10,w10
	bra NZ, dfp_prog_loop_page
	
; now verify the whole thing
	mov #page_buf, w5	; reset page buf ptr
	mov page_lowptr,w0

	mov #512,w10		; 512 instr. counter

dfp_verify_loop:
	mov.b [w5++], w2
	tblrdh [w0],w8
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; BUG in the microcontroller !
; If you read the flash with tblrdl.b two time (with post-increment)
; The second read is _corrupted_. You MUST use 
; tblrdl with word addressing then swap the word
; MANY thanks to Microchip for this undocummented bug 
; it took about ~10 hours to catch this one ... 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; There is still a bug somewhere. I _REALLY_ don't know why
; but sometime I get wrong data either from w5 or flash ... 
; The most strange thing is that if we verify after with a READPAGE
; it works .... 
; Maybe a errata will pop on Microhip in 6 month ? 
	tblrdl [w0++],w1
	cpseq.b w1,w2
	bra dfp_error_prog
	
	mov.b [w5++], w2
	swap w1
	cpseq.b w1,w2
	bra dfp_error_prog

	mov.b [w5++], w2
	cpseq.b w1,w8
	bra dfp_error_prog
	
	dec w10,w10
	bra NZ, dfp_verify_loop
	
	retlw #can_answer_OK,w0
	
dfp_error_prog:
	dec w0,w1
	retlw #can_answer_PROGRAM_FAILED,w0



; Do the key sequence and start the flash programming
; in: nothing
; out: nothin
; modify: w0,w1
do_key_seq:
	disi #5
	mov #0x55,w0
	mov w0,NVMKEY
	mov #0xaa,w1
	mov w1,NVMKEY
	bset NVMCON, #WR
	nop 
	nop
	return


;;;;;;;;;;;;
; Write configuration Command
;;;;;;;;;;;;
cmd_writeconf:
	mov [w4 + 4], w0
	subr w0,#8,w0	
	bra NZ, error_dlc
	mov [w4 + 8], w0
	mov w0,TBLPAG
	btss w0,#7	; check that this is really a configuration address
	bra error_inval
	mov [w4 + 10], w0
	mov [w4 + 12], w1
	mov #0x4000, w2
	mov w2, NVMCON
	tblwtl w0, [w1]
	rcall do_key_seq	; program
	mov w0,w2
	mov #can_answer_OK, w0
	tblrdl [w1],w1
	cpseq w1,w2		; check it's ok
	mov #can_answer_PROGRAM_FAILED, w0
	rcall send_error_code
	bra wait_can_packet


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
ask_can_runlevel:
	mov C1CTRL1, w1
	mov #0xF8FF,w2
	and w1,w2,w1
	ior w1,w0,w1
	mov w1, C1CTRL1
	mov #0xE0, w2
	lsr w0,#3,w0
_ask_can_runlevel_loop:
	mov C1CTRL1, w1
	and w1,w2,w1
	sub w1,w0,w1
	bra NZ, _ask_can_runlevel_loop
	return;
