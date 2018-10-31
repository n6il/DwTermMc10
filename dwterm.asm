CNSLIN	equ	$F865
KEYIN	equ	$F883
PUTCHR	equ	$F9C6
* BLINK	equ	$F83F		* RoM's routine doesnt work well for us

CR	equ	13
BREAK	equ	3
CTRL_1	equ	142
CTRL_2	equ	147
CTRL_7	equ	149
CTRL_8	equ	157

SLEEP_FAST	equ	5
SLEEP_SLOW	equ	200
BUFSIZ	equ	214

channel equ	1

* DW Commands
OP_DWINIT	equ	$5A
OP_SERSETSTAT	equ	$C4
OP_SERREAD	equ	$43
OP_SERREADM	equ	$63
OP_FASTWRITE	equ	$80
* DW SERSETSTAT Codes
SS_Open		equ	$29
SS_Close	equ	$2A
	
	org	20000

start:
	ldx	#MESSAGE	* Print Banner
	ldab	#MSGLEN
	jsr	putstr
	jsr	putpr		* Print initial prompt
	ldaa	#SLEEP_SLOW	* Initialize sleep counter
	staa	SLPCNTR

	jsr	dwinit		* Initialize DW
	ldaa	#channel	* Open virtual serial channel 1
	jsr	open_channel
	jsr	get_status	* Initial get status
	bra	inkey		* start the main look

* need this as a target for conditionals as done is too far away
Xdone:
	jmp	done

* Keyboard input loop
inkey:
	jsr	BLINK
	jsr	KEYIN		* get char from keyboard
	cmpa	#0		* check for no input
	beq	sleep		* sleep and loop if not

	ldx	CRSPTR		* got something so clear the cursor first
	ldab	#143
	stab	,X

	cmpa	#BREAK		* BREAK key
	beq	Xdone		* get outta here
*	cmpa	#CR
*	beq	mcr
	cmpa	#CTRL_7		* 7-bit toggle
	bne	inswitch2
	ldab	#$FF		* toggle the flag
	eorb	disp_7bit
	stab	disp_7bit
	jsr	b7togglemsg
	bra	sleep
inswitch2:
	cmpa	#CTRL_8		* ascii toggle
	bne	dodwwrite
	ldab	#$FF		* toggle the flag
	eorb	disp_ascii
	stab	disp_ascii
	jsr	atogglemsg
	bra	sleep
dodwwrite:
	ldab	#1		* dw channel number
	jsr	writeChannel	* write char to drivewire
	jsr	PUTCHR		* write char to screen
	ldab	#SLEEP_FAST	* fast sleep since a key was entered
	stab	SLPCNTR
sleep:
	dec	SLPCNTR		* decrement sleep counter
	bne	inkey		* loop until 0

* Try to read data from DW Server
input:
	ldaa	#1		* dw channel
	ldab	#BUFSIZ		* buffer size
	clr	rcwait		* no wait
	jsr	readChannel	* try to read from drivewire
	tab			* move result to b
	cmpb	#0		* nothing was read
	beq	slow		* slow sleep
fast:
	ldaa	#SLEEP_FAST	* poll dw faster if there was some data
	bra	update
slow:
	ldaa	#SLEEP_SLOW	* poll kb faster if there was no data
update:
	staa	SLPCNTR		* update sleep counter

* process data buffer from dw
process:
	ldx	#buffer
procloop:
	tstb			* check if anything was read
	beq	channel_check	* move on if not
	ldaa	,x		* Get the char
	clc
	jsr	do_iac		* Process Telnet IAC Protocol
	bcs	proccont	* if carry set: don't output char
	jsr	do_ascii
	bcs	proccont	* if carry set: don't output char
procprint:
	jsr	PUTCHR		* output the char
proccont:
	decb			* counter
	inx			* bufer pointer
	bra	procloop	* go for more

* Check the Dw channel and re-open it if necessary
channel_check:
	tst	channel_open	* check if the channel is still open
	bne	inkey		* return to keyboard loop if so
	ldaa	#1		* ensure channel is closed
	jsr	close_channel
	jsr	get_status	* get status to make sure server closed it
	ldaa	#1
	jsr	open_channel	* reopen the channel
	jsr	get_status	* initial get status
	ldaa	#SLEEP_SLOW	* start with sleep slow
	staa	SLPCNTR
mcr:
	ldaa	#CR		* print CR and prompt
	jsr	putcpr
	jmp	inkey		* return to the keyboard loop

done:
	tst	channel_open
	beq	done2
	ldaa	#1
	jsr	close_channel
done2:
	ldx	#BYEMSG
	ldab	#BYELEN
	jsr	putstr
	rts

dwinit:
	ldx	#initmsg
	ldd	#2
	jsr	SerWrite
	ldx	#buffer
	ldd	#1
	jsr	SerRead
	rts
initmsg:
	fcb	OP_DWINIT, $FF

openchan:
 rts

getstat:
 rts

* input - A char to write
dwwritechr:
	pshx
	psha
	ldx	#cmdbuf
	staa	,x
	ldd	#1
	jsr	SerWrite
	pula
	pulx
	rts

dwread:
 rts

putcpr:
	jsr	PUTCHR
putpr:
	ldx	#PROMPT
	ldab	#PRLEN
	jsr	putstr
	rts

putstr:
psloop:
	cmpb	#0
	beq psdone
	ldaa	,x
	jsr	PUTCHR
	inx
	decb
	bra	psloop
psdone	rts

mPUTCHR:
	pshx
	pshb
	ldx	CRSPTR
	ldab	#143
	stab	,X
	pulb
	pulx
	jsr	PUTCHR
	rts

* openCh - open Dw Channel
* input: A - channel
open_channel:
	ldx	#cmdbuf
	ldab	#OP_SERSETSTAT	* Param 1: DW Command
	stab	,x
	staa	1,x		* Param 2: Channel
	staa	channel_open
	ldab	#SS_Open	* Param 3: Serstat op
	stab	2,x
	ldd	#3		* Command Length
	jsr	SerWrite
	rts

* close_channel - open Dw Channel
* input: A - channel
close_channel:
	ldx	#cmdbuf
	ldab	#OP_SERSETSTAT	* Param 1: DW Command
	stab	,x
	staa	1,x		* Param 2: Channel
	ldab	#SS_Close	* Param 3: Serstat op
	stab	2,x
	ldd	#3		* Command Length
	jsr	SerWrite
	rts

* get_status - get Dw Channel status
* output: X - checksum
*	CC - 0 - success, 1 Framing error, -1 Timeout
get_status:
	ldx	#cmdbuf
	ldab	#OP_SERREAD	* Param 1: DW Command
	stab 	,x
	ldd	#1		* Command Length
	jsr	SerWrite

	ldx	#status
	ldd	#0		* clear status
	std	,x
	ldd	#2		* Get 2 bytes
	jsr	SerRead
	rts			* Status and CC set
	
* writeChannel
* input - B channel
* 	A data
writeChannel:
	pshx
	psha
	pshb
	ldx	#cmdbuf
	staa	1,x		* Param 2: data
	ldaa	#OP_FASTWRITE	* OP_FASTWRITE+Channel
	aba			
	staa	,x		* Param 1: FASTWRITE command
	ldd	#2		* Command Length
	jsr	SerWrite
	pulb
	pula
	pulx
	rts

* readChannel
* input - A channel
*	B size
*	wait - global
* variables
rcread	fcb	0
rcwait	fcb	0
rcchannel fcb	0
rcsize	fcb	0
rcbuf	fdb	0

readChannel:
	pshx
	staa	rcchannel	* Save channel
	stab	rcsize		* Save size
	clr	rcread		* initialize read counter
	ldx	#buffer
	stx	rcbuf	
	jsr	get_status
	beq	rc2cont		*  0 - got the status
rc2exit:
	pulx
	ldaa	rcread
	rts
rc2cont:
	ldaa	status
	beq	rc2exit		*  0 - nothing read
	cmpa	#16		* 16 - status
	beq	rc2status
	bgt	rc2multi	* >16 -  multiple bytes
rc2readone:
	ldaa	status+1	* get the byte
	staa	,x		* save it
	inc	rcread		* increment read counter
	bra	rc2exit		* all done
rc2multi:
	ldaa	#OP_SERREADM	* Param 1: Serial Read Command
	ldx	#cmdbuf
	staa	,x
	ldaa	rcchannel	* Param 2: Channel
	staa	1,x
	ldaa	status+1	* Param 3: bytes
	staa	2,x
	ldd	#3		* Command Length
	jsr	SerWrite	* Send the read command
	ldx	#buffer		* X - buffer
	clra			* D - num bytes to read
	ldab	status+1
	jsr	SerRead		* Read the data in
	bne	rc2exit		* Something went wrong
	ldaa	status+1	* Get the number of bytes requested
	staa	rcread		* Save is as successfully read
	bra	rc2exit		* All done
rc2status:
*	ldaa	status+1
*	suba	#17
*	cmpa	rcchannel
*	bne	rc2status2
	clr	channel_open	* channel is closed
	jsr	get_status	* need to get status to cause it to close
	bra	rc2exit		* so get outta here
rc2status2:
	cmpa	#255		* reboot request
	bra	rc2exit		* and anything else just exit


iac	fcb	0
* do_iac - process Telnet IAC Protocol
* input A - char
* outpt A - what to send
do_iac:
	pshb
	tst	iac
	beq	iswitch
	ldab	#255
	stab	cmdbuf
	ldab	iac
	stab	cmdbuf+1
	staa	cmdbuf+2
	ldx	#cmdbuf
	ldd	#3
	jsr	SerWrite
	clr	iac
	bra	inosend
iswitch:
	cmpa	#251
	bne iswitch2
	ldab	#253
	stab	iac
	bra	inosend
iswitch2:
	cmpa	#252
	bne	iswitch3
	ldab	#254
	stab	iac
	bra	inosend
iswitch3:
	cmpa	#253
	beq	iswitch3a
	cmpa	#254
	bne	iswitch4
iswitch3a:
	ldab	#252
	stab	iac
	bra	inosend
iswitch4:
	cmpa	#255
	beq	inosend
iswitch5:
	cmpa	#$10
	beq	inosend
iacsend:
	pulb
	clc
	rts
inosend:
	pulb
	sec
	rts

do_ascii:
	tst	disp_ascii	* test ascii flag
	beq	a7bit		* zero: next check
	cmpa	#$80		* first non ascii
	bhs	anosend
a7bit:
	tst	disp_7bit	* test 7-bit flag
	beq	asend		* zero: send without change
	anda	#$7F		* strip high bit
asend:
	clc
	rts
anosend:
	sec
	rts

atogglemsg:
	pshx
	pshb
	ldx	#disp_ascii	* get the flag address
	pshx			* save for later
	ldx	#ATOGGLESTR	* print ascii message
	ldab	#ATOGGLELEN
	bra	toggleprint

b7togglemsg:
	pshx
	pshb
	ldx	#disp_7bit	* get the flag address
	pshx			* save for later
	ldx	#B7TOGGLESTR	* print 7bit message
	ldab	#B7TOGGLELEN

toggleprint:
	jsr	putstr
	pulx			* get saved flag address
	ldab	,x		* load value
	beq	toff		* its off
	ldx	#ONMSG		* print the on messge
	ldab	#ONLEN
	bra	tvalueprint
toff:
	ldx	#OFFMSG		* print off message
	ldab	#OFFLEN
tvalueprint:
	jsr	putstr
	pulb
	pulx
	rts	



* Stole this code from Micro Basic ROM
* BLNKTM	equ	$422B			* cursor blink timer (1)
* CRSPTR	equ	$4280			* cursor position
* CRSCLR	equ	$4282			* cursor color
* MCX Locations
BLNKTM	equ	$012B			* cursor blink timer (1)
CRSPTR	equ	$0180			* cursor position
CRSCLR	equ	$0182			* cursor color
BLNKTM2	fdb	$2c0
* Blink the cursor if it is time
BLINK:
*LF83F	dec       BLNKTM              * decrement blink timer
*	bne       LF85E               * branch if not yet time to blink
	ldx	  BLNKTM2
	dex
	stx	  BLNKTM2
	bne       LF85E               * branch if not yet time to blink
	ldaa      CRSCLR              * get current cursor state
	eora      #$0F                * toggle the four pixel bits
	staa      CRSCLR              * save new cursor state
	oraa      #$80                * set the graphics bit
	ldx       CRSPTR              * point X to cursor location in Video RAM
	staa      ,X                  * change the cursor color
*	ldab      #22                 * timer value when cursor is off
	ldx       #$2c0               * timer value when cursor is off
	anda      #$0F                * test the cursor state
	beq       LF85B               * branch if cursor is off
*	ldab      #88                 * timer value when cursor is on
	ldx       #$2c0               * timer value when cursor is on
*LF85B	stab      BLNKTM              * restart the timer
LF85B	stx       BLNKTM2             * restart the timer
LF85E	rts

ATOGGLESTR fcc	'[ASCII DISPLAY '
ATOGGLELEN equ	*-ATOGGLESTR
B7TOGGLESTR fcc	'[7-BIT DISPLAY '
B7TOGGLELEN equ	*-B7TOGGLESTR
ONMSG	fcc	'ON]'
	fcb	CR
ONLEN	equ	*-ONMSG
OFFMSG	fcc	'OFF]'
	fcb	CR
OFFLEN	equ	*-OFFMSG

MESSAGE	fcc	'DWTERM 0.1 FOR MC-10'
	fcb	CR,CR
	fcc	'TYPE [BREAK] TO QUIT,'
	fcb	CR
	fcc	'DW OR AT COMMANDS AT THE PROMPT'
	fcb	CR
MSGLEN	equ	*-MESSAGE

PROMPT	fcc	'DW> '
PRLEN	equ	*-PROMPT

BYEMSG	fcb	CR
	fcc	'THANKS FOR USING DWTERM, BYE!'
	fcb	CR
	fcc	'RETURNING TO BASIC'
	fcb	CR
BYELEN	equ	*-BYEMSG

SLPCNTR	fcb	SLEEP_SLOW

* global variables
* flag whether the channel is open
channel_open	fcb	0	
* current channel status
status	zmb	2
* ascii flag - 0 - display all 
disp_ascii	fcb	0
* 7-bit flag - 0 - display all 
disp_7bit	fcb	0

* bufffers
* command buffer
cmdbuf	zmb	BUFSIZ
* data buffer
buffer	zmb	BUFSIZ


*******************************************************
*******************************************************
*******************************************************
* Serial Code Starts Here
*******************************************************
*******************************************************
*******************************************************

P2DATA      equ          $0003             ; Port 2 Data Register :
*                                          ;    bit 0 = serial output
*                                          ;    bit 2 = serial input


***
***  Variables used by the high speed serial routines.
***  These reuse other temporary locations in Basic's Direct Page.
***  The code makes timing assumptions based on Direct Page addressing.
***
FP1EXP      equ          $00D6             ; floating point accumulator #1
SERSP       equ          FP1EXP            ; saved stack pointer
SERSUM      equ          FP1EXP+2          ; checksum accumulator
SERPTR      equ          FP1EXP+4          ; IO byte pointer
SERCTR      equ          FP1EXP+6          ; IO byte counter



*******************************************************
*
* SerRead
*    Receive a block of data through the serial port.
*    Times out if 2 (or 4) seconds ellapse without any data.
*    Serial format:  8-N-1 at 38400 bps
*
* Entry:
*     X  = storage address for incoming data
*     D  = number of bytes wanted
*  CC.C  = doubles initial timeout to 4 seconds if set
*
* Exit:
*    X  = 16 bit checksum of data received.
*    A  = clobberd.
*    B  = staus code:
*          0 = Success
*         -1 = Timed Out
*          1 = Framing Error
*   CC  = IRQ mask is set, Z and N reflect status code in B.
*
SerRead     sei                                 ; mask IRQs
            std         SERCTR                  ; save request count
            ldd         #0                      ; initialize..
            std         SERSUM                  ; ..checksum
            ldd         #$0401                  ; ACCA = input mask, ACCB = initial timeout MSB
            adcb        #0                      ; double timeout MSB if carry set
            dex                                 ; adjust storage ptr for pre-increment
            stx         SERPTR                  ; save storage ptr
            sts         SERSP                   ; save stack pointer

* Wait for start bit or timeout.
rx010       lds         SERCTR      4  | 13     ; get request counter
            beq         rxDone      3  |        ; exit if all bytes received
            bita        P2DATA      3 /         ; check for start bit
            beq         rxByte      3 \         ; 
            ldx         #0          3  | 9      ; initialize inner timeout loop counter
            bita        P2DATA      3 /         ; check for start bit
            beq         rxByte      3 \  6      ; 
rx020       bita        P2DATA      3 /         ; check for start bit
            beq         rxByte      3 \         ; 
            dex                     3  | 9      ; decrement inner loop counter
            bita        P2DATA      3 /         ; check for start bit
            beq         rxByte      3 \         ; 
            cpx         #1          4  | 10     ; set Carry if inner loop counter = 0
            bita        P2DATA      3 /         ; check for start bit
            beq         rxByte      3 \         ; 
            bcc         rx020       3  | 9      ; loop if carry clear
            bita        P2DATA      3 /         ; check for start bit
            beq         rxByte      3 \         ; 
            decb                    2  | 11     ; decrement timeout MSB
            bne         rx020       3 /         ; loop if timeout hasn't expired
            decb                                ; ACCB = TIMEOUT status (-1)
            bra         rxExit                  ; exit

* Read one byte
rxByte      des                     3 \         ; decrement the..
            sts         >SERCTR     5  |        ; ..request counter
            ins                     3  |        ; 3 cycle delay
            lds         SERPTR      4  | 24     ; point S to the location..
            ins                     3  |        ; ..where the byte will be stored
            sts         SERPTR      4  |        ; save storage ptr
            ldab        #$40        2 /         ; setup ACCB for 7 shifts
rx030       ldaa        P2DATA      3           ; read data bit from serial port
            dex                     3 \         ; delay for..
            inx                     3  |        ; ..six cycles
            ldx         SERSUM      4  |        ; load checksum value into X
            lsra                    2  | 20     ; shift the data bit down..
            lsra                    2  |        ; ..to the lowest bit of ACCA..
            lsrd                    3  |        ; ..then into highest bit of ACCB
            bcc         rx030       3 /         ; loop until 7 data bits have been read
            ldaa        >P2DATA     4           ; read 8th data bit
            lsra                    2 \         ; shift the data bit down..
            lsra                    2  |        ; ..to the lowest bit of ACCA..
            lsrd                    3  |        ; ..then into highest bit of ACCB
            pshb                    3  | 24     ; store received byte
            abx                     3  |        ; add byte value to checksum
            stx         SERSUM      4  |        ; save checksum
            ldd         #$0401      3  |        ; ACCA = mask, ACCB = Timeout MSB / FRAMING ERROR
            bita        >P2DATA     4 /         ; sample the stop bit
            bne         rx010       3 \         ; loop if good stop bit
            fcb         $21                     ; skip next single-byte instruction (BRN opcode)

* Epilog
rxDone      clrb                                ; ACCB = SUCCESS
rxExit      lds         SERSP                   ; pop storage ptr off the stack
            tstb                                ; setup CC.Z and CC.N to reflect status
            rts                                 ; return


*******************************************************
*
* SerWrite
*    Send a block of data through the serial port.
*    Serial format:  8-N-1 at 38400 bps
*
* Entry:
*    X  = address of first byte to send
*    D  = number of bytes to send
*
* Exit:
*   CC  = IRQ mask bit is set, Carry is cleared.
*    X  = 16 bit checksum of sent data.
*    A and B are clobbered.
*
*
SerWrite    std         SERCTR                  ; store byte count
            clra                                ; initialize..
            clrb                                ; ..checksum..
            std         SERSUM                  ; ..to zero
            ldaa        #$11                    ; set bit 0 and the IRQ mask bit in ACCA
            tap                                 ; mask IRQs

txByte      staa        P2DATA      3           ; transmit stop bit
            ldab        ,x          4 \         ; get next data byte
            inx                     3  |        ; increment data ptr
            pshx                    4  |        ; save data ptr on stack
            ldx         SERSUM      4  | 22     ; X = checksum value
            abx                     3  |        ; add byte value to checksum
            ldaa        #8          2  |        ; ACCA = loop counter (start bit + 7 data bits)
            lslb                    2 /         ; bit 7 into Carry; '0' into bit 0

tx010       stab        P2DATA      3           ; transmit bit 0 of ACCB
            rorb                    2 \         ; move next bit into position
            stx         SERSUM      4  |        ; store checksum value
            pshx                    4  | 20     ; \ 9 cycle delay
            pulx                    5  |        ; /
            suba        #1          2  |        ; decrement loop counter (use instead of DECA to clear carry)
            bne         tx010       3 /         ; loop until 7 data bits have been sent

            stab        >P2DATA     4           ; transmit 8th data bit (Ext Addressing for 24th cycle)
            inca                    2 \         ; ACCA = stop bit (MARK)
            ldx         SERCTR      4  |        ; X = remaining byte count (including this one)
            dex                     3  | 21     ; decrement remaining byte count
            stx         SERCTR      4  |        ; store updated count
            pulx                    5  |        ; restore data ptr from stack
            bne         txByte      3 /         ; loop if more bytes to send

            staa        P2DATA      3           ; transmit final stop bit
            ldx         SERSUM      4           ; checksum is returned in X
            rts                     5           ; return

