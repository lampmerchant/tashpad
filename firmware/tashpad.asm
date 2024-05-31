;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  TashPad: Emulator for Gravis GamePad using Super Famicom Controller
;;;
;


;;; Connections ;;;

;;;                                                   ;;;
;                    .--------.                         ;
;            Supply -|01 \/ 08|- Ground                 ;
;      ADB <--> RA5 -|02    07|- RA0 <--- Data In       ;
;      LED <--- RA4 -|03    06|- RA1 ---> Data Clock    ;
;    !MCLR ---> RA3 -|04    05|- RA2 ---> Data Latch    ;
;                    '--------'                         ;
;                                                       ;
;    LED is active low.                                 ;
;                                                       ;
;;;                                                   ;;;


;;; Assembler Directives ;;;

	list		P=PIC12F1501, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P12F1501.inc
	errorlevel	-302	;Suppress "register not in bank 0" messages
	errorlevel	-224	;Suppress TRIS instruction not recommended msgs
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_ON & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_ON	RA3/!MCLR is !MCLR
			;_CP_OFF	Code protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
	__config	_CONFIG2, _WRT_OFF & _STVREN_ON & _BORV_LO & _LPBOR_OFF &_LVP_OFF
			;_WRT_OFF	Write protection off
			;_STVREN_ON	Stack over/underflow causes reset
			;_BORV_LO	Brownout reset voltage low trip point
			;_LPBOR_OFF	Low power brownout reset disabled
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

KBDHDLR	equ	0x02	;ADB handler ID when in keyboard mode (default)
PADHDLR	equ	0x34	;ADB handler ID when in gamepad mode

;Pin Assignments:
DAT_PIN	equ	RA0	;Data input pin
CLK_PIN	equ	RA1	;Data clock pin
LAT_PIN	equ	RA2	;Data latch pin
LED_PIN	equ	RA4	;LED pin
ADB_PIN	equ	RA5	;ADB pin

;FLAGS:
PADMODE	equ	7	;Set when device is in gamepad mode
SWRIGHT	equ	6	;Set when gamepad switch is set to the right
SRQENBL	equ	5	;Set when SRQs are enabled
COLLIDE	equ	4	;Set when the last transmission collided
TXEVENT	equ	3	;Set when the state machine wants Tx events

;AP_FLAG:
AP_RST	equ	7	;Set when a reset condition is detected, user clears
AP_COL	equ	6	;Set when the transmission collided, user clears
AP_RXCI	equ	5	;Set when command byte in AP_BUF, user clears
AP_RXDI	equ	4	;Set when data byte in AP_BUF, user clears
AP_DONE	equ	3	;Set when transmission or reception done, user clears
AP_TXI	equ	2	;User sets after filling AP_BUF, interrupt clears
AP_SRQ	equ	1	;User sets to request service, user clears
AP_RISE	equ	0	;Set when FSA should be entered on a rising edge too


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	FLAGS	;You've got to have flags
	AP_FLAG	;ADB flags
	AP_FSAP	;Pointer to where to resume ADB FSA
	AP_SR	;ADB shift register
	AP_BUF	;ADB buffer
	AP_DTMR	;ADB down-cycle timer value
	AC_FSAP	;Pointer to where to resume ADB user program state
	AC_ADDR	;ADB address of gamepad
	AC_TEMP	;Temporary variable between FSA states
	SF_FSAP	;Pointer to where to resume Super Famicom controller FSA
	SF_PADS	;Super Famicom converted to gamepad button state
	SF_LAST	;Last gamepad button state
	X3
	X2
	X1
	X0
	
	endc

	;FSR0 - Button state queue pop pointer
	;FSR1 - Button state queue push pointer

	;Linear memory:
	;0x2000-0x201F - Button state queue
	;0x2020-0x202F - unused


;;; Vectors ;;;

	org	0x0		;Reset vector
	goto	Init

	org	0x4		;Interrupt vector
	;fall through


;;; Interrupt Handler ;;;

Interrupt
	bcf	STATUS,C	;Copy the Timer0 flag into the carry bit so it
	btfsc	INTCON,TMR0IF	; doesn't change on us mid-stream
	bsf	STATUS,C	; "
	btfsc	STATUS,C	;If the Timer0 flag is set and the interrupt is
	btfss	INTCON,TMR0IE	; enabled, handle it as an event for the ADB
	bra	$+2		; state machine
	call	IntAdbTimer	; "
	movlb	7		;If the ADB pin has had a negative or positive
	btfsc	IOCAF,ADB_PIN	; edge, handle it as an event for the ADB state
	call	IntAdbEdge	; machine
	retfie

IntAdbTimer
	movlb	1		;Disable the Timer0 interrupt
	bcf	INTCON,TMR0IE	; "
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag and its mirror
	bcf	STATUS,C	; in the carry bit
	return
	
IntAdbEdge
	movlw	1 << ADB_PIN	;Toggle the edge that the IOC interrupt catches
	xorwf	IOCAN,F		; "
	xorwf	IOCAP,F		; "
	bcf	IOCAF,ADB_PIN	;Clear the interrupt flag
	btfsc	IOCAN,ADB_PIN	;If the edge we just caught is a rising edge,
	bra	IntAdbRising	; jump ahead, otherwise fall through
	;fall through

IntAdbFalling
	movlb	0		;If Timer0 overflowed, this falling edge is
	btfsc	STATUS,C	; the first after a too-long period, so handle
	bra	IntAdbTimeout	; it as a timeout
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbRising
	movlb	0		;If Timer0 overflowed, this rising edge is at
	btfsc	STATUS,C	; the end of a reset pulse
	bra	IntAdbReset	; "
	movf	TMR0,W		;Save the current value of Timer0 so it can be
	movwf	AP_DTMR		; considered after its corresponding falling
	clrf	TMR0		; edge, then clear it and its flag
	bcf	INTCON,TMR0IF	; "
	btfss	AP_FLAG,AP_RISE	;If the flag isn't set that the state machine
	return			; wants to be resumed on a rising edge, done
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbReset
	bsf	AP_FLAG,AP_RST	;Set the reset flag
	clrf	AP_DTMR		;Clear the down timer
	;fall through

IntAdbTimeout
	clrf	AP_FSAP		;Reset the ADB state machine
	clrf	TMR0		;Reset Timer0 and its flag and disable its
	bcf	INTCON,TMR0IF	; interrupt
	bcf	INTCON,TMR0IE	; "
	return


;;; Hardware Initialization ;;;

Init
	banksel	OSCCON		;16 MHz high-freq internal oscillator
	movlw	B'01111000'
	movwf	OSCCON

	banksel	IOCAN		;ADB sets IOCAF on negative edge
	movlw	(1 << ADB_PIN)
	movwf	IOCAN

	banksel	OPTION_REG	;Timer0 uses instruction clock, 1:16 prescaler,
	movlw	B'11010011'	; thus ticking every 4 us
	movwf	OPTION_REG

	banksel	T1CON		;Timer1 ticks once per instruction cycle
	movlw	B'00000001'
	movwf	T1CON

	banksel	T2CON		;Timer2 ticks 1:1 with instruction clock and
	movlw	24		; measures 6 us intervals
	movwf	PR2
	movlw	B'00000100'
	movwf	T2CON

	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA

	banksel	LATA		;Default state of output pins is low, ADB is
	clrf	LATA		; ready to be pulled low

	banksel	TRISA		;Latch, clock, and LED output, data input, ADB
	movlw	B'00101001'	; open collector ready to be pulled low
	movwf	TRISA

	movlb	0		;Initialize key globals
	movlw	0x20
	movwf	FSR0H
	clrf	FSR0L
	movwf	FSR1H
	clrf	FSR1L
	clrf	FLAGS
	clrf	AP_FLAG
	clrf	AP_FSAP
	movlw	low SfcFsaStart
	movwf	SF_FSAP
	clrf	SF_LAST
	call	SvcAdbReset

	movlw	B'10001000'	;On-change interrupt and interrupt subsystem on
	movwf	INTCON

	;fall through


;;; Mainline ;;;

Main
	call	SvcAdb		;Service the ADB user program
	call	SvcSfc		;Service the Super Famicom pad
	bra	Main		;Loop

SvcSfc
	movlb	0		;If Timer2 hasn't tripped yet, do nothing
	btfss	PIR1,TMR2IF	; "
	return			; "
	clrf	TMR2		;Reset Timer2
	bcf	PIR1,TMR2IF	; "
	movlb	2		;Call into the Super Famicom pad FSA
	movlp	high SfcFsa	; "
	movf	SF_FSAP,W	; "
	callw			; "
	movwf	SF_FSAP		;On returning, save the address returned in W
	return

SvcAdb
	btfsc	AP_FLAG,AP_RST	;If the reset flag is set, handle it
	call	SvcAdbReset	; "
	btfsc	AP_FLAG,AP_RXCI	;If the received-command flag is set, handle it
	call	SvcAdbCommand	; "
	btfss	AP_FLAG,AP_RXDI	;If either the received-data or collision flag
	btfsc	AP_FLAG,AP_COL	; is set, handle it
	call	SvcAdbData	; "
	btfsc	AP_FLAG,AP_DONE	;If the transmission-done flag is set, handle
	call	SvcAdbData	; it
	btfss	FLAGS,TXEVENT	;If the transmit-buffer-ready flag is set and
	return			; the state machine wishes to be notified of
	btfss	AP_FLAG,AP_TXI	; this, handle it
	call	SvcAdbData	; "
	return			; "

SvcAdbReset
	bsf	FLAGS,SRQENBL	;SRQ on by default
	bcf	FLAGS,COLLIDE	;No collision by default
	bcf	FLAGS,PADMODE	;Keyboard handler by default
	movlw	0x2		;Address 0x2 by default
	movwf	AC_ADDR		; "
	bcf	AP_FLAG,AP_RST	;Clear the reset flag, if it was set
	return

SvcAdbCommand
	bcf	AP_FLAG,AP_RXCI	;Clear the command flag
	bcf	AP_FLAG,AP_SRQ	;No SRQ by default
	movf	AP_BUF,W	;If the low four bits of the command are zero,
	andlw	B'00001111'	; this is a SendReset command and should be
	btfsc	STATUS,Z	; treated the same as a reset pulse
	bra	SvcAdbReset	; "
	swapf	AP_BUF,W	;If the address of the command matches the
	xorwf	AC_ADDR,W	; gamepad's address, proceed, else skip ahead
	andlw	B'00001111'	; "
	btfss	STATUS,Z	; "
	bra	SvcAdC0		; "
	movlp	high ACFsa	;Call into the gamepad state machine's initial
	movlw	0		; state
	callw			; "
	movwf	AC_FSAP		;On returning, save the address returned in W
	return			;Done
SvcAdC0	movf	FSR0L,W		;If this command is for another device and we
	xorwf	FSR1L,W		; have more button state to report and SRQs are
	btfss	STATUS,Z	; enabled and we're in pad mode, raise SRQ
	bsf	AP_FLAG,AP_SRQ	; "
	btfsc	FLAGS,PADMODE	; "
	btfss	FLAGS,SRQENBL	; "
	bcf	AP_FLAG,AP_SRQ	; "
	return

SvcAdbData
	movlp	high ACFsa	;Call into the gamepad state machine
	movf	AC_FSAP,W	; "
	callw			; "
	movwf	AC_FSAP		;On returning, save the address returned in W
	bcf	AP_FLAG,AP_RXDI	;Clear the flags that could have brought us
	bcf	AP_FLAG,AP_COL	; here
	bcf	AP_FLAG,AP_DONE	; "
	return


;;; State Machines ;;;

ACFsa	org	0x200

ACFsaCommand
	btfsc	AP_BUF,2	;If this is a talk command, handle it
	bra	ACFsaTalk	; "
	btfss	AP_BUF,3	;If this is not a listen command, ignore it
	retlw	low ACFsaIgnore	; "
	;fall through

ACFsaListen
	btfsc	AP_BUF,1	;We only respond to Listen 3, so if it's 0-2,
	btfss	AP_BUF,0	; ignore it
	retlw	low ACFsaIgnore	; "
	retlw	low ACFsaLstn3H	;If it's Listen 3, transition to receive bytes

ACFsaLstn3H
	movf	AP_BUF,W	;We can't act on the high byte until we know
	movwf	AC_TEMP		; what the low byte (handler ID) is, so store
	retlw	low ACFsaLstn3L	; it in temporary space

ACFsaLstn3L
	movf	AP_BUF,W	;If handler ID is 0x00, it means to change the
	btfsc	STATUS,Z	; device's address and SRQ enable bit
	bra	ACFL3L1		; unconditionally
	addlw	2		;If handler ID is 0xFE, it means to change the
	btfsc	STATUS,Z	; device's address if a collision hasn't been
	bra	ACFL3L0		; detected
	addlw	-KBDHDLR-2	;If handler ID is KBDHDLR, disable gamepad mode
	btfsc	STATUS,Z	; "
	bcf	FLAGS,PADMODE	; "
	addlw	-PADHDLR+KBDHDLR;If handler ID is PADHDLR, enable gamepad mode
	btfsc	STATUS,Z	; "
	bsf	FLAGS,PADMODE	; "
	retlw	low ACFsaIgnore	;If anything else, ignore
ACFL3L0	btfss	FLAGS,COLLIDE	;If a collision has not been detected, skip
	bra	ACFL3L2		; ahead to change the address; if one has been
	bcf	FLAGS,COLLIDE	; detected, clear it and ignore this command
	retlw	low ACFsaIgnore	; "
ACFL3L1	bcf	FLAGS,SRQENBL	;Copy the state of the SRQ enable bit to the SRQ
	btfsc	AC_TEMP,5	; enable flag
	bsf	FLAGS,SRQENBL	; "
ACFL3L2	movf	AC_TEMP,W	;Accept the low four bits of the first received
	andlw	B'00001111'	; byte as our new address and we're done
	movwf	AC_ADDR		; "
	retlw	low ACFsaIgnore	; "

ACFsaTalk
	movf	AP_BUF,W	;Switch off by which register was ordered to
	andlw	B'00000011'	; talk
	btfsc	STATUS,Z	; "
	bra	ACFsaTalk0H	; "
	addlw	-1		; "
	btfsc	STATUS,Z	; "
	bra	ACFsaTalk1H	; "
	addlw	-1		; "
	btfsc	STATUS,Z	; "
	bra	ACFsaTalk2H	; "
	;fall through

ACFsaTalk3H
	movlb	0		;Get a pseudorandom four-bit number and put it
	movf	TMR1H,W		; into the low nibble of the buffer; this way
	xorwf	TMR1L,W		; we replace address (which the host already
	andlw	B'00001111'	; knows) with a random number, which helps with
	movwf	AP_BUF		; collision detection
	bsf	AP_BUF,6	;Exceptional event bit always set
	btfsc	FLAGS,SRQENBL	;SRQ enabled bit set if SRQs are enabled
	bsf	AP_BUF,5	; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,TXEVENT	; and we are interested in transmission events
	retlw	low ACFsaTalk3L	;Deal with what happened in the next state

ACFsaTalk3L
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	ACFT3L0		; bit, clear the transmit events flag, and we're
	bsf	FLAGS,COLLIDE	; done until we get another command
	bcf	FLAGS,TXEVENT	; "
	retlw	low ACFsaIgnore	; "
ACFT3L0	movlw	KBDHDLR		;Load KBDHDLR for transmit if we're in keyboard
	btfsc	FLAGS,PADMODE	; mode, or PADHDLR if we're in gamepad mode
	movlw	PADHDLR		; "
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	FLAGS,TXEVENT	; and that we no longer want transmit events
	retlw	low ACFsaTalkE	;Deal with what happened in the next state

ACFsaTalkE
	btfsc	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bsf	FLAGS,COLLIDE	; bit
	retlw	low ACFsaIgnore	;Either way, we're done

ACFsaTalk2H
	movlw	0xFF		;Talk 2 always returns 0xFF 0xFF, so load 0xFF
	movwf	AP_BUF		; to be transmitted
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,TXEVENT	; and we are interested in transmission events
	retlw	low ACFsaTalk2L	;Deal with what happened in the next state

ACFsaTalk2L
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	ACFT2L0		; bit, clear the transmit events flag, and we're
	bsf	FLAGS,COLLIDE	; done until we get another command
	bcf	FLAGS,TXEVENT	; "
	retlw	low ACFsaIgnore	; "
ACFT2L0	movlw	0xFF		;Talk 2 always returns 0xFF 0xFF, so load 0xFF
	movwf	AP_BUF		; to be transmitted
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	FLAGS,TXEVENT	; and that we no longer want transmit events
	retlw	low ACFsaTalkE	;Deal with what happened in the next state

ACFsaTalk1H
	movlw	0x03		;Talk 1 always returns 0x03 0x00, so load 0x03
	movwf	AP_BUF		; to be transmitted
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,TXEVENT	; and we are interested in transmission events
	retlw	low ACFsaTalk1L	;Deal with what happened in the next state

ACFsaTalk1L
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	ACFT1L0		; bit, clear the transmit events flag, and we're
	bsf	FLAGS,COLLIDE	; done until we get another command
	bcf	FLAGS,TXEVENT	; "
	retlw	low ACFsaIgnore	; "
ACFT1L0	clrf	AP_BUF		;Load 0x00 to be transmitted
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	FLAGS,TXEVENT	; and that we no longer want transmit events
	retlw	low ACFsaTalkE	;Deal with what happened in the next state

ACFsaTalk0H
	btfss	FLAGS,PADMODE	;If we're not in gamepad mode, don't respond to
	retlw	low ACFsaIgnore	; Talk 0
	movf	FSR1L,W		;If the queue is empty, we have no data to
	xorwf	FSR0L,W		; return, so ignore the command
	btfsc	STATUS,Z	; "
	retlw	low ACFsaIgnore	; "
	movf	INDF0,W		;Grab the next button state off the queue
	movwf	AP_BUF		;Load it to be transmitted
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bsf	FLAGS,TXEVENT	; and we are interested in transmission events
	retlw	low ACFsaTalk0L	;Deal with what happened in the next state

ACFsaTalk0L
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	ACFT0L0		; bit, clear the transmit events flag, and we're
	bsf	FLAGS,COLLIDE	; done until we get another command
	bcf	FLAGS,TXEVENT	; "
	retlw	low ACFsaIgnore	; "
ACFT0L0	movlw	0xFE		;Return all ones for bits 7:1 and the switch
	btfsc	FLAGS,SWRIGHT	; position in bit 0
	movlw	0xFF		; "
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Set the flags to say we have a byte to send
	bcf	FLAGS,TXEVENT	; and that we no longer want transmit events
	retlw	low ACFsaTalk0E	;Deal with what happened in the next state

ACFsaTalk0E
	btfss	AP_FLAG,AP_COL	;If there was a collision, set the collision
	bra	ACFT0E0		; bit and we're done until we get another
	bsf	FLAGS,COLLIDE	; command
	retlw	low ACFsaIgnore	; "
ACFT0E0	incf	FSR0L,F		;Advance the queue pop pointer since we sent the
	bcf	FSR0L,5		; state successfully
	retlw	low ACFsaIgnore	;And we're done

ACFsaIgnore
	retlw	low ACFsaIgnore	;Utility state to ignore until next command


SfcFsa

SfcFsaStart
	bsf	LATA,CLK_PIN	;Raise the clock and latch pins
	bsf	LATA,LAT_PIN	; "
	retlw	low SfcFsaLatch	;Transition to next state

SfcFsaLatch
	bcf	LATA,LAT_PIN	;Lower latch so controller clocks out first bit
	clrf	SF_PADS		;Zero out converted pin data
	retlw	low SfcFsaLatB	;Transition to next state

SfcFsaLatB
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of B button from controller and
	btfss	PORTA,DAT_PIN	; translate this into the yellow gamepad button
	bsf	SF_PADS,7	; "
	retlw	low SfcFsaClkY	;Transition to next state

SfcFsaClkY
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatY	;Transition to next state

SfcFsaLatY
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of Y button from controller and
	btfss	PORTA,DAT_PIN	; translate this into the red gamepad button
	bsf	SF_PADS,5	; "
	retlw	low SfcFsaClkSt	;Transition to next state

SfcFsaClkSt
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatSt	;Transition to next state

SfcFsaLatSt
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of start button from controller and
	btfss	PORTA,DAT_PIN	; if pressed, move the gamepad switch to the
	bsf	FLAGS,SWRIGHT	; right
	retlw	low SfcFsaClkSl	;Transition to next state

SfcFsaClkSl
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatSl	;Transition to next state

SfcFsaLatSl
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of select button from controller and
	btfss	PORTA,DAT_PIN	; if pressed, move the gamepad switch to the
	bcf	FLAGS,SWRIGHT	; left
	retlw	low SfcFsaClkUp	;Transition to next state

SfcFsaClkUp
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatUp	;Transition to next state

SfcFsaLatUp
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of D-pad up from controller and
	btfss	PORTA,DAT_PIN	; translate this to the gamepad's D-pad
	bsf	SF_PADS,3	; "
	retlw	low SfcFsaClkDn	;Transition to next state

SfcFsaClkDn
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatDn	;Transition to next state

SfcFsaLatDn
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of D-pad down from controller and
	btfss	PORTA,DAT_PIN	; translate this to the gamepad's D-pad
	bsf	SF_PADS,1	; "
	retlw	low SfcFsaClkLt	;Transition to next state

SfcFsaClkLt
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatLt	;Transition to next state

SfcFsaLatLt
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of D-pad left from controller and
	btfss	PORTA,DAT_PIN	; translate this to the gamepad's D-pad
	bsf	SF_PADS,0	; "
	retlw	low SfcFsaClkRt	;Transition to next state

SfcFsaClkRt
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatRt	;Transition to next state

SfcFsaLatRt
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of D-pad right from controller and
	btfss	PORTA,DAT_PIN	; translate this to the gamepad's D-pad
	bsf	SF_PADS,2	; "
	retlw	low SfcFsaClkA	;Transition to next state

SfcFsaClkA
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatA	;Transition to next state

SfcFsaLatA
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of A button from controller and
	btfss	PORTA,DAT_PIN	; translate this into the green gamepad button
	bsf	SF_PADS,6	; "
	retlw	low SfcFsaClkX	;Transition to next state

SfcFsaClkX
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatX	;Transition to next state

SfcFsaLatX
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of X button from controller and
	btfss	PORTA,DAT_PIN	; translate this into the blue gamepad button
	bsf	SF_PADS,4	; "
	retlw	low SfcFsaClkL	;Transition to next state

SfcFsaClkL
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatL	;Transition to next state

SfcFsaLatL
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of L button from controller and
	btfss	PORTA,DAT_PIN	; translate this into the red gamepad button
	bsf	SF_PADS,5	; "
	retlw	low SfcFsaClkR	;Transition to next state

SfcFsaClkR
	bsf	LATA,CLK_PIN	;Raise clock so controller clocks out next bit
	retlw	low SfcFsaLatR	;Transition to next state

SfcFsaLatR
	bcf	LATA,CLK_PIN	;Lower clock
	movlb	0		;Get state of R button from controller and
	btfss	PORTA,DAT_PIN	; translate this into the blue gamepad button
	bsf	SF_PADS,4	; "
	movf	SF_PADS,W	;If any button is down, turn the LED on, else
	movlb	2		; turn it off
	btfsc	STATUS,Z	; "
	bsf	LATA,LED_PIN	; "
	btfss	STATUS,Z	; "
	bcf	LATA,LED_PIN	; "
	comf	SF_PADS,W	;If the current button state is the same as the
	xorwf	SF_LAST,W	; last, loop around to poll the buttons again
	btfsc	STATUS,Z	; without doing anything with the button state
	retlw	low SfcFsaStart	; "
	comf	SF_PADS,W	;Update the last button state
	movwf	SF_LAST		; "
	movwf	INDF1		;Push the button state onto the queue
	incf	FSR1L,W		;If the queue push pointer cannot be advanced
	andlw	B'00011111'	; (because the queue is full), don't advance it
	xorwf	FSR0L,W		; and instead zero out the last button state,
	btfsc	STATUS,Z	; which will all but ensure that we keep trying
	clrf	SF_LAST		; to push the latest button state onto the
	btfsc	STATUS,Z	; queue, and loop around to poll the buttons
	retlw	low SfcFsaStart	; again
	incf	FSR1L,F		;Advance and wrap the queue push pointer
	bcf	FSR1L,5		; "
	retlw	low SfcFsaStart	;Loop around to poll the buttons again


AdbFsa	org	0x300

AdbFsaIdle
	clrf	TMR0		;Reset timer
	movf	AP_DTMR,W	;If the down time was 194-206 ticks (800 us +/-
	addlw	-207		; 3%), this is an attention pulse, so prepare
	btfsc	STATUS,C	; the shift register to receive a command byte
	retlw	low AdbFsaIdle	; and transition to receive the first bit
	addlw	13		; "
	btfss	STATUS,C	; "
	retlw	low AdbFsaIdle	; "
	movlw	0x01		; "
	movwf	AP_SR		; "
	retlw	low AdbFsaCmdBit; "

AdbFsaCmdBit
	btfsc	AP_DTMR,7	;If either the down time or the up time is over
	retlw	low AdbFsaIdle	; 127 (508 us, ridiculous), throw up our hands
	btfsc	TMR0,7		; and wait for an attention pulse
	retlw	low AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaCmdBit; there are more command bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXCI	; that a command has been received
	movlw	-12		;Set a timer to expire and interrupt after 48us
	movwf	TMR0		; so that the user program has time to decide
	bsf	INTCON,TMR0IE	; whether or not to request service or transmit
	bsf	AP_FLAG,AP_RISE	;Set to catch rising edge that starts Tlt
	retlw	low AdbFsaSrq	;Set to enter the state handling service reqs

AdbFsaSrq
	btfsc	BSR,0		;If for some reason we're here because of an
	bra	AFSrq0		; edge, cancel our timer interrupt, stop
	bcf	INTCON,TMR0IE	; catching rising edges, reset timer, and bail
	bcf	AP_FLAG,AP_RISE	; out to waiting for an attention pulse
	clrf	TMR0		; "
	retlw	low AdbFsaIdle	; "
AFSrq0	btfss	AP_FLAG,AP_SRQ	;If the user didn't call for a service request,
	retlw	low AdbFsaTlt	; just wait for Tlt to begin
	bcf	TRISA,ADB_PIN	;If the user did call for a service request,
	movlw	-63		; pull the pin low and set a timer for 252 us
	movlb	0		; above the 48 us we already waited to release
	movwf	TMR0		; it
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaSrqEnd; "

AdbFsaSrqEnd
	btfss	BSR,0		;On the off chance we're here because edge, go
	retlw	low AdbFsaSrqEnd; around again until the timer expires
	bsf	TRISA,ADB_PIN	;Release the pin that we pulled low to request
	retlw	low AdbFsaTlt	; service and wait for Tlt (could be right now)

AdbFsaTlt
	bcf	AP_FLAG,AP_RISE	;No longer need to catch rising edges
	movlw	-65		;Shorten the timeout period to 260 us, which is
	movwf	TMR0		; slightly longer than Tlt is expected to be
	bsf	INTCON,TMR0IE	;Timer interrupts whether we transmit or not
	btfss	AP_FLAG,AP_TXI	;If the user doesn't wish to transmit, just
	retlw	low AdbFsaTltEnd; wait for data to start
	movf	TMR1H,W		;Get a pseudorandom between 0 and 15, adjust it
	xorwf	TMR1L,W		; to between 199 and 214; that will make Timer0
	andlw	B'00001111'	; overflow in between 168us and 228us, which is
	addlw	-57		; close enough to the specced range of 160us to
	movwf	TMR0		; 240us to wait before transmitting
	movlw	B'11000000'	;Set the shift register so it outputs a 1 start
	movwf	AP_SR		; bit and then loads data from the buffer
	retlw	low AdbFsaTxBitD;Bring us to the transmission start state

AdbFsaTxBitD
	btfsc	BSR,0		;If we're here because of a timer interrupt,
	bra	AFTxBD0		; as we hope, skip ahead
	bsf	AP_FLAG,AP_COL	;If not, set the collision flag, clear the
	bcf	AP_FLAG,AP_TXI	; transmit flag, cancel the timer, and go back
	clrf	TMR0		; to waiting for an attention pulse
	bcf	INTCON,TMR0IE	; "
	retlw	low AdbFsaIdle	; "
AFTxBD0	bcf	TRISA,ADB_PIN	;Pull the pin low
	lslf	AP_SR,F		;Shift the next bit to send into carry bit
	btfss	STATUS,Z	;If we shifted the placeholder out of the shift
	bra	AFTxBD1		; register, continue, else skip ahead
	btfss	AP_FLAG,AP_TXI	;If there's no new byte ready to load, clear
	bcf	STATUS,C	; carry so we send a zero as our last bit
	btfss	AP_FLAG,AP_TXI	;If there's a new byte ready to load, load it,
	bra	AFTxBD1		; shift its MSB out and a 1 placeholder into
	rlf	AP_BUF,W	; its LSB and clear the transmit flag; else
	movwf	AP_SR		; leave the shift register all zeroes as a
	bcf	AP_FLAG,AP_TXI	; signal to the up phase state that we're done
AFTxBD1	movlw	-8		;Set a timer to interrupt in 8 cycles (32us) if
	movlb	0		; sending a 1 bit, double that to 16 cycles
	btfss	STATUS,C	; (64us) if we're sending a 0 bit, and also
	lslf	WREG,W		; save this value for use by the up phase state
	movwf	TMR0		; "
	movwf	AP_DTMR		; "
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaTxBitU; "

AdbFsaTxBitU
	btfss	BSR,0		;If we're here because of the falling edge we
	retlw	low AdbFsaTxBitU; just triggered, ignore and return posthaste
	bsf	TRISA,ADB_PIN	;Release the pin
	DELAY	2		;Wait for it to actually go high
	movlb	0		;If the pin is still low, we've collided; set
	btfsc	PORTA,ADB_PIN	; the collision flag, clear the transmit flag,
	bra	AFTxBU0		; and go back to waiting for an attention pulse
	bsf	AP_FLAG,AP_COL	; "
	bcf	AP_FLAG,AP_TXI	; "
	retlw	low AdbFsaIdle	; "
AFTxBU0	movf	AP_SR,W		;If the down phase let the shift register stay
	btfsc	STATUS,Z	; at zero, the bit we just transmitted is the
	bsf	AP_FLAG,AP_DONE	; stop bit and transmission is over, so set the
	btfsc	STATUS,Z	; done flag and return to waiting for an
	retlw	low AdbFsaIdle	; attention pulse
	movlw	B'00001000'	;Whatever delay (8 or 16) we did during the
	xorwf	AP_DTMR,W	; down phase, set a timer to do the other one
	movwf	TMR0		; "
	bsf	INTCON,TMR0IE	; "
	movlb	7		;Reverse the IOC interrupt and clear the flag
	bcf	IOCAP,ADB_PIN	; set by releasing the pin so the timer we just
	bsf	IOCAN,ADB_PIN	; set doesn't immediately get reset
	bcf	IOCAF,ADB_PIN	; "
	retlw	low AdbFsaTxBitD;Timer will take us to the down phase again

AdbFsaTltEnd
	btfsc	BSR,0		;If we're here because of a timer interrupt, the
	bra	AFTltE0		; transmission never started, so skip ahead
	clrf	TMR0		;This state is the end of Tlt and the start of
	retlw	low AdbFsaRxStrt; host or other device transmitting data
AFTltE0	bsf	AP_FLAG,AP_DONE	;Set the done flag because the data payload is
	retlw	low AdbFsaIdle	; effectively done and return to idle

AdbFsaRxStrt
	movlw	0x01		;Start bit should be 1, but who cares, just set
	movwf	AP_SR		; up the shift register to receive the first
	clrf	TMR0		; data bit
	bsf	AP_FLAG,AP_RISE	;Catch rising edges to set receive timeout timer
	retlw	low AdbFsaRxBitD; "

AdbFsaRxBitD
	movlw	-31		;Set the timeout timer for 124 us, up time on a
	movwf	TMR0		; received bit should never be this long, and
	bsf	INTCON,TMR0IE	; wait for a falling edge or a timeout
	retlw	low AdbFsaRxBitU; "

AdbFsaRxBitU
	btfss	BSR,0		;If we got here because of a timer overflow, the
	bra	AFRxBU0		; data payload must be done, so disable catching
	bcf	AP_FLAG,AP_RISE	; rising edges, set the done flag, and return to
	bsf	AP_FLAG,AP_DONE	; idle
	retlw	low AdbFsaIdle	; "
AFRxBU0	movlw	31		;Compensate for us setting Timer0 to time out
	addwf	TMR0,F		; early
	btfsc	AP_DTMR,7	;If the down time is over 127 (508 us,
	bcf	AP_FLAG,AP_RISE	; ridiculous), throw up our hands and wait for
	btfsc	AP_DTMR,7	; an attention pulse
	retlw	low AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaRxBitD; there are more data bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXDI	; that a data byte has been received
	movlw	0x01		;Set up the shift register to receive the next
	movwf	AP_SR		; bit and wait for it
	retlw	low AdbFsaRxBitD; "


;;; End of Program ;;;
	end
