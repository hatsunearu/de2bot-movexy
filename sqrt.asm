;When sqrt is called, it calculates the integer square root of AC and writes it back to AC
;The most this calculation will be off is 1, which will correspond to 1/16" in our final implementation.

    LOAD    N	
    CALL    sqrt

test:
	OUT		LCD
	JUMP	test

sqrt:
    CALL    Numbits  	 
    SHIFT   -1
    STORE   B
    LOAD    bits 
    AND     One
    ADD     B
    STORE   ceilnumbits/2
    
    ;calculate 2^ceil(numbits(N)/2)
    LOAD    One
    STORE   x
    LOAD    ceilnumbits/2
Loop1:
    ADDI    -1
    STORE   B
    LOAD    x
    SHIFT   1
    STORE   x
    LOAD    B
    JPOS    Loop1

Loop3:
	LOAD	N
	STORE	num
	LOAD	Zero
	STORE	floorN/x
	
;calculate floor(N/x)
divide:
	LOAD	floorN/x
	ADDI	1
	STORE 	floorN/x
	LOAD	num
	SUB		x
	STORE	num
	JPOS	divide
	JZERO	done
	LOAD	floorN/x
	ADDI	-1
done:
    LOAD	floorN/x

	ADD     x
	SHIFT   -1
	STORE   y	
    SUB     x
    ADDI    1
    JPOS    returnx
    LOAD    y
    STORE   x
    JUMP    Loop3
returnx:
    LOAD    x
    RETURN


;Finds the number of bits needed to store the integer in AC

Numbits:
    JZERO   ItsZero
    SHIFT   -1
    STORE   A
    LOAD    bits
    ADDI    1
    Store   bits
    LOAD    A
    JUMP    Numbits
ItsZero:
    LOAD    bits
    RETURN
    
Zero:		DW	0
One:    	DW	1
N:		DW	25625 ;number to be square rooted
A:		DW	0
B:		DW	0
x:		DW	0
y:		DW	0
ceilnumbits/2:	DW	0
floorn/x:	DW	0
bits:		DW	0
LCD:      EQU &H06  ; primitive 4-digit LCD display
