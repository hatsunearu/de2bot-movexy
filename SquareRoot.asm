ORG        &H005       ;Begin program at x000
; square root estimation. Given distances X and Y it estimates sqrt(x^2+y^2) and stores it in ANS
SQUAREROOT:
	;load x and check if x is negative
	;LOAD X
	;JPOS XNOTNEG
	;JZERO XNOTNEG
	;ADD X
	;ADD X
	;STORE X
;XNOTNEG:
	;LOAD Y
	;JPOS YNOTNEG
	;JZERO YNOTNEG
	;ADD Y
	;ADD Y
	;STORE Y
	
YNOTNEG:
	LOAD Y
	SUB X ;Y-X
	JNEG XLARGER
	JUMP YLARGER
	
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
	SHIFT 1 		;AC = min<<1
	ADD REGISTER 	;AC = (min<<1)+ (max<<4) + (max<<1) + (min<<5)
	STORE REGISTER 	;register = (min<<1)+ (max<<4) + (max<<1) + (min<<5)
	
	LOAD MAX
	SHIFT 8 		;AC = max<<8
	SUB REGISTER 	;AC =(max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	STORE REGISTER 	;register = (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	
	LOAD MAX
	SHIFT 3			;AC = max<<3
	ADD REGISTER	;AC = (max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	STORE REGISTER	;register = (max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	
	LOAD MIN
	SHIFT 7 		;AC = min<<7
	ADD REGISTER 	;AC = (min<<7)+(max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	STORE REGISTER	;register = (min<<7)+(max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	
	LOAD MIN
	SHIFT 3			;AC = min<<3
	ADD REGISTER	;AC = (min<<3) + (min<<7)+ (max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))
	
	SHIFT -8		; AC = [(min<<3) + (min<<7)+ (max<<3) + (max<<8) - ((min<<1)+ (max<<4) + (max<<1) + (min<<5))] >>8
	STORE ANS
	OUT SEVENSEG
HALT:
	JUMP HALT
	
XLARGER:
	LOAD Y
	STORE MIN
	LOAD X
	STORE MAX
	LOAD MIN
	JUMP CALC

YLARGER:
	LOAD X
	STORE MIN
	LOAD Y
	STORE MAX
	LOAD MIN
	JUMP CALC

X:		DW 500
Y:		DW 500
ANS:	DW 0
MIN:    DW 0
MAX:    DW 0
ONE:	DW 1
REGISTER: DW 0
LEDS:			EQU &H01;
SEVENSEG:		EQU &H04;
