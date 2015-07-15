ORG        &H000       ;Begin program at x000
; square root estimation. Given distances X and Y it estimates sqrt(x^2+y^2) and stores it in ANS
SQUAREROOT:
	LOAD X
	JPOS XNOTNEG
	JZERO XNOTNEG
	SUB X
	SUB X
	STORE X
XNOTNEG:
	LOAD Y
	JPOS YNOTNEG
	JZERO YNOTNEG
	SUB Y
	SUB Y
	STORE Y
YNOTNEG:
	LOAD Y
	SUB X ;Y-X
	JNEG XLARGER
	;otherwise y is larger or they are equal
	LOAD X
	STORE MIN
	LOAD Y
	STORE MAX
CALC:
	; first add all terms we need to subtract
	LOAD MAX
	SHIFT 4			;ALU = max<<4
	STORE REGISTER 	;register = max<<4
	
	LOAD MAX
	SHIFT 1 		;ALU = max<<1
	ADD REGISTER 	;ALU = (max<<4) + (max<<1)
	STORE REGISTER 	;register = (max<<4) + (max<<1)
	
	LOAD MIN
	SHIFT 5			;ALU = min<<5
	ADD REGISTER 	;ALU = (max<<4) + (max<<1) + (min<<5)
	STORE REGISTER 	;register = (max<<4) + (max<<1) + (min<<5)
	
	LOAD MIN
	SHIFT 1 		;ALU = min<<1
	ADD REGISTER 	;ALU = (min<<1)+ (max<<4) + (max<<1) + (min<<5)
	STORE REGISTER 	;register = (min<<1)+ (max<<4) + (max<<1) + (min<<5)
	
	LOAD MAX
	SHIFT 8 		;ALU = max<<8
	SUB REGISTER 	;ALU =(max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	STORE REGISTER 	;register = (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	
	LOAD MAX
	SHIFT 3			;ALU = max<<3
	ADD REGISTER	;ALU = (max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	STORE REGISTER	;register = (max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	
	LOAD MIN
	SHIFT 7 		;ALU = min<<7
	ADD REGISTER 	;ALU = (min<<7)+(max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	STORE REGISTER	;register = (min<<7)+(max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	
	LOAD MIN
	SHIFT 3			;ALU = min<<3
	ADD REGISTER	;ALU = (min<<3) + (min<<7)+ (max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	
	SHIFT -8		; ALU = [(min<<3) + (min<<7)+ (max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))] >>8
	STORE ANS
	OUT LEDS
	
XLARGER:
	LOAD Y
	STORE MIN
	LOAD X
	STORE MAX
	JUMP CALC

X:		DW 12
Y:		DW 9
ANS:	DW 0
MIN:    DW 0
MAX:    DW 0
REGISTER: DW 0
LEDS:			EQU &H01;
