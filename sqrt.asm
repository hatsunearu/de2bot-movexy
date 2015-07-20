
;The variable and loop names are garbage, and I don't think this code handles the input of zero properly, but this is the gist
;of the integer square root algorithm. I don't even think SCASM would accept some of the names I have but I'll deal with that later


    LOAD    N
    CALL    sqrt

sqrt:
    CALL    Numbits
    SHIFT   &B1000000001
    STORE   B
    LOAD    numbits
    AND     1
    ADD     B
    STORE   ceilnumbits/2 ;ceilnumbits/2 is a varaible name that literally holds the value ceil(numbits(N)/2)

    ;calculate 2^ceil(numbits(N)/2)
    LOAD    One
    STORE   x
    LOAD    ceilnumbits/2
Loop1
    ADDI    -1
    STORE   B
    LOAD    x
    SHIFT   1
    STORE   x
    LOAD    B
    JPOS    Loop


Loop3:
;calculate floor(N/x)
    LOAD    One
    STORE   floorN/x
    LOAD    N
Loop2:
    ADDI    -1
    STORE   B
    LOAD    floorN/x
    SHIFT   &B1000000001
    STORE   floorN/x        ;floorN/x is also a variable name that holds floor(N/x)
    LOAD    B
    JPOS    Loop2

ADD     x
SHIFT   &B1000000001
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
    SHIFT   &B1000000001
    STORE   A
    LOAD    Counter
    ADDI    1
    Store   Counter
    LOAD    A
    JUMP    Numbits
ItsZero:
    LOAD    Counter
    RETURN

Counter:    DW  0
Zero:       DW  0
One:        DW  1
N:			DW	16
A:			DW	0
B:			DW	0
x:			DW	0
y:			DW	0
ceilnumbits/2:	DW	0
floorn/x:	DW	0

