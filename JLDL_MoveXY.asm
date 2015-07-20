; JLDL_MoveXY.asm
; Created by Kevin Johnson
; Modified by John Braatz, Laura Palmore, Don Gi Min, and Lovissa Winyoto
; (no copyright applied; edit freely, no attribution necessary)
; This program does basic initialization of the DE2Bot
; and includes subroutines for position logging.

; Section labels are for clarity only.

;***************************************************************
;* Jump Table
;***************************************************************
; When an interrupt occurs, execution is redirected to one of
; these addresses (depending on the interrupt source), which
; need to either contain RETI (return from interrupt) if not
; used, or a JUMP instruction to the desired interrupt service
; routine (ISR).  The first location is the reset vector, and
; should be a JUMP instruction to the beginning of your normal
; code.
ORG        &H000       ; Jump table is located in mem 0-4
	JUMP   Init        ; Reset vector
	RETI               ; Sonar interrupt (unused)
	JUMP   CTimer_ISR  ; Timer interrupt
	RETI               ; UART interrupt (unused)
	RETI               ; Motor stall interrupt (unused)
	
;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display batt voltage on LCD

WaitForSafety:
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety
	
WaitForUser:
	; Wait for user to press PB3
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue

;***************************************************************
;* Main code
;***************************************************************
Main: ; "Real" program starts here.
	OUT    RESETPOS    ; reset odometer in case wheels moved after programming
	CALL   InputCoord  ; Input coordinate to the robot manually
	CALL   UARTClear   ; empty the UART receive FIFO of any old data
	CALL   StartLog    ; enable the interrupt-based position logging
	
	in switches ; @ set number of ticks to cover using switch
	store DestR   

	call MoveXY
	
	

Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD   Zero        ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD        ; An indication that we are dead
	OUT    SSEG2
	CALL   StopLog     ; Disable position logging
Forever:
	JUMP   Forever     ; Do this forever.
DEAD:      DW &HDEAD   ; Example of a "local variable"

;***************************************************************
;* Input Coordinate, store to "Input" variable
;***************************************************************
InputCoord:
	LOAD	Seven		;
	OUT		SSEG1		; SSEG1 = 7
	OUT		SSEG2		; SSEG2 = 7

WaitForSafety2:
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser2 ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety2
	
WaitForUser2:
	; Wait for user to press PB3
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  3           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask1       ; KEY2 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser2 ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue

	LOAD   Six
	OUT	   SSEG1		; SSEG1 = 6
	OUT	   SSEG2		; SSEG2 = 6
	
	IN	   SWITCHES
	STORE  Input
	OUT	   SSEG1

	RETURN

;***************************************************************
;* Move XY Subroutines
;***************************************************************

; *****************************************************************************
; ** MoveXY **
; Precondition:  DestR (input param) contains distance to travel in a straight line
; Postcondition: DE2Bot travels with specified PD control
; *****************************************************************************
MoveXY:
	out RESETPOS ; reset odometry
	
ControlLoop:
	out TIMER ; reset timer
	
UpdateErrors:
	; left
	in LPOS            ; |
	store MotorTemp    ; |
	load DestR         ; | 
	sub MotorTemp      ; V Target - Position
	store ErrorL       ; update left encoder error
	
	sub ErrorLPrev
	store ErrorLDeriv  ; update derivative error
	
	load ErrorL
	store ErrorLPrev   ; update previous error
	; right
	in RPOS            ; |
	store MotorTemp    ; |
	load DestR         ; | 
	sub MotorTemp      ; V Target - Position
	store ErrorR       ; update right encoder error
	
	sub ErrorRPrev
	store ErrorRDeriv  ; update derivative error
	
	load ErrorR
	store ErrorRPrev   ; update previous error

KpL:
	load ErrorL
	store MSError
	loadi Kp_m
	call MultiplySanitize
	store CorrectionL

KdL:
	load ErrorLDeriv
	store MSError
	loadi Kd_m
	call MultiplySanitize
	store CorrDerivL

KpR:
	load ErrorR
	store MSError
	loadi Kp_m
	call MultiplySanitize
	store CorrectionR
	
KdR:
	load ErrorRDeriv
	store MSError
	loadi Kd_m
	call MultiplySanitize
	store CorrDerivR
	
	call AccumError
	
LoopWait:
	in TIMER
	addi -1 ; reset after 2 timer ticks
	jneg LoopWait
	jump ControlLoop
	
; *****************************************************************************
; ** MultiplySanitize **
; Precondition:  ACC has value to multiply (Kp or Kd)
;                MSError (input param) contains error to be multiplied with
; Postcondition: ACC has multiplied value, capped at 1 word
; *****************************************************************************

; input params
MSError:
	dw 0;

MultiplySanitize:
	store m16sA        ; |
	load MSError       ; |
	store m16sB        ; |
	call Mult16s       ; V call multiplication 
	
	load mres16sH      ; Load high word
	jzero Trivial     ; High word is 0x0000, means positive
	sub FFFF
	jzero Trivial     ; High word is 0xFFFF, means negative
	jump NTrivial     ; High word is not trivial

Trivial: ; write out trivial values that only occupy first word
	load mres16sL
	return
 
NTrivial: ; High word is nontrivial
	load mres16sH      
	jpos SetToMax      ; |
	jneg SetToMin      ; V set nontrivial values as the extreme
	
SetToMax: ; set to maximum value
	load MaxPosVal
	return

SetToMin: ; set minimum value
	load MaxNegVal
	return
	

; constants
MaxPosVal: 
	dw &H7FFF
MaxNegVal:
	dw &H8000


; *****************************************************************************
; ** AccumError **
; Precondition:  CorrectionL/R and CorrDerivL/R are populated
; Postcondition: Correct motor value is written to motor
; Requires:      MotorLimit
; *****************************************************************************

AccumError:
	load CorrectionL
	shift -15 ; get sign bit for left prop
	store AccumErrorSign
	load CorrDerivL
	shift -15 ; get sign bit for left deriv
	xor AccumErrorSign 
	store AccumErrorSignFlag ; 0 if both sign bits are the same, red flag for overflow
	
AddL:
	load CorrectionL
	add CorrDerivL
	store AccumErrorSum ; compute sum
	load AccumErrorSignFlag
	jzero ChkOverflowL ; jump if overflow is impossible
	jump WriteOutL

ChkOverflowL:
	; overflow possible
	load CorrectionL
	jzero WriteOutL ; fringe case
	load AccumErrorSum ; get sum
	shift -15 ; get sign bit of sum
	xor AccumErrorSign ; 0 if signs are the same, 1 = overflow
	jzero WriteOutL ; no overflow
	; overflow
	load AccumErrorSign
	jzero AEPosL
	
	load ZERO
	sub MaxSpeed
	store AccumErrorSum
	
AEPosL: ; overflow in positive direction
	load MaxSpeed
	store AccumErrorSum
	jump WriteOutL
	
WriteOutL: ; write to motor
	load AccumErrorSum
	call MotorLimit
	out LVELCMD

;right motor
	load CorrectionR
	shift -15 ; get sign bit for left prop
	store AccumErrorSign
	load CorrDerivR
	shift -15 ; get sign bit for left deriv
	xor AccumErrorSign 
	store AccumErrorSignFlag ; 0 if both sign bits are the same, red flag for overflow
	
AddR:
	load CorrectionR
	add CorrDerivR
	store AccumErrorSum ; compute sum
	load AccumErrorSignFlag
	jzero ChkOverflowR ; jump if overflow is impossible
	jump WriteOutR
	
ChkOverflowR:
	; overflow possible
	load CorrectionR
	jzero WriteOutR ; fringe case
	load AccumErrorSum ; get sum
	shift -15 ; get sign bit of sum
	xor AccumErrorSign ; 0 if signs are the same, 1 = overflow
	jzero WriteOutR ; no overflow
	; overflow
	load AccumErrorSign
	jzero AEPosR
	load ZERO
	sub MaxSpeed
	store AccumErrorSum

AEPosR: ; overflow in negative direction
	load MaxSpeed
	store AccumErrorSum
	jump WriteOutR
	
WriteOutR: ; write to motor
	load AccumErrorSum
	call MotorLimit
	out RVELCMD
	return
	
; local variables
AccumErrorSum:
	dw 0
AccumErrorSign:
	dw 0
AccumErrorSignFlag: ; flag if can be overflowed, 1 = no overflow possible
	dw 0
	
; *****************************************************************************
; ** MotorLimit **
; Precondition:  ACC has value from 0x0 to 0xffff
; Postcondition: ACC has value limited from -0d511 to 0d511
; *****************************************************************************

MotorLimit:
	store MotorTemp ; save ACC value
	call Abs
	sub MaxSpeed
	jpos OverLimit
	load MotorTemp
	return
OverLimit:
	load MotorTemp ; recall ACC
	jpos LimitHigh ; check if original intended motor speed was positive
	load ZERO
	sub MaxSpeed ; motor speed negative, get negative max speed
	return
LimitHigh: ; motor speed positive, get positive max speed
	load ZERO
	add MaxSpeed
	return

;***************************************************************
;* Move XY Variables / Constants
;***************************************************************
; input parameters
DestX:
DW 0
DestY:
DW 0

; destination parameters
DestR:
DW 0

; current error parameters
ErrorL:
DW 0
ErrorR:
DW 0

; previous error parameters
ErrorLPrev:
DW 0
ErrorRPrev:
DW 0

ErrorLDeriv:
dw 0
ErrorRDeriv:
dw 0

; current control parameters
CorrectionL: ; correction resulting from prop term
DW 0
CorrectionR:
DW 0
; 
CorrDerivL: ; correction resulting from deriv term
dw 0
CorrDerivR:
dw 0

; ** Constants

; ** Kp : Proportional Control Constant
Kp_m: EQU 4 ; multiply by this number

; ** Kd : Derivative Control Constant
Kd_m: EQU 16 ; 

MotorTemp:
	dw 0
MaxSpeed:
	dw 511
FFFF:
	dw &HFFFF


	
;***************************************************************
;* Subroutines
;***************************************************************

; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second in 10Hz.
	JNEG   Wloop
	RETURN

; Subroutine to wait the number of timer counts currently in AC
WaitAC:
	STORE  WaitTime
	OUT    Timer
WACLoop:
	IN     Timer
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	SUB    WaitTime
	JNEG   WACLoop
	RETURN
	WaitTime: DW 0     ; "local" variable.
	
; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

; Subroutine to send AC value through the UART,
; formatted for default base station code:
; [ AC(15..8) | AC(7..0)]
; Note that special characters such as \lf are
; escaped with the value 0x1B, thus the literal
; value 0x1B must be sent as 0x1B1B, should it occur.
UARTSend:
	STORE  UARTTemp
	SHIFT  -8
	ADDI   -27   ; escape character
	JZERO  UEsc1
	ADDI   27
	OUT    UART_DAT
	JUMP   USend2
UEsc1:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
USend2:
	LOAD   UARTTemp
	AND    LowByte
	ADDI   -27   ; escape character
	JZERO  UEsc2
	ADDI   27
	OUT    UART_DAT
	RETURN
UEsc2:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
	RETURN
	UARTTemp: DW 0

; Subroutine to send a newline to the computer log
UARTNL:
	LOAD   NL
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NL: DW &H0A1B

; Subroutine to send a space to the computer log
UARTNBSP:
	LOAD   NBSP
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NBSP: DW &H201B

; Subroutine to clear the internal UART receive FIFO.
UARTClear:
	IN     UART_DAT
	JNEG   UARTClear
	RETURN
	
; Timer interrupt, used to send position data to the server
CTimer_ISR:
	CALL   UARTNL ; newline
	IN     XPOS
	CALL   UARTSend
	CALL   UARTNBSP ; space
	IN     YPOS
	CALL   UARTSend
	RETI   ; return from interrupt

; Configure the interrupt timer and enable interrupts
StartLog:
	CALL   UARTNL      ; send a newline to separate data
	LOADI  100
	OUT    CTIMER      ; configure timer for 0.01*100=1s interrupts
	CLI    &B0010      ; clear any pending interrupt from timer
	SEI    &B0010      ; enable interrupt from timer (source 1)
	RETURN

; Disable the interrupt timer and interrupts
StopLog:
	LOADI  0
	OUT    CTIMER      ; reset configurable timer
	CLI    &B0010      ; disable interrupt source 1 (timer)
	RETURN
	
;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
	LOADI  0
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOADI  16           ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s      
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry, 
	ADD    m16sA        ;  add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ;  subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOADI  0
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOADI  17
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOADI  1
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOADI  0
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	XOR    NegOne
	ADDI   1
	STORE  dres16sQ
	RETURN	
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
	JPOS   Abs_r
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

;***************************************************************
;* Variables
;***************************************************************
Temp:     DW 0 ; "Temp" is not a great name, but can be useful
Input:	  DW 0 ; "Input" is the coordinate input from user

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

; some useful movement values
OneMeter: DW 961       ; ~1m in 1.05mm units
HalfMeter: DW 481      ; ~0.5m in 1.05mm units
TwoFeet:  DW 586       ; ~2ft in 1.05mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 130       ; 13.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
