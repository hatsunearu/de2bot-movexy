; This worked when I copy and pasted it into the main part of the code in SimpleRobotProgram.asm between Main (after call
UARTClear) and Die


;  convert output of ATAN2 (Heading) to [-180,180]
LOAD   Heading
ADDI   -180        ; test whether facing 0-179 or 180-359
JPOS   NegAngle    ; robot facing 180-360; handle that separately
PosAngle:
ADDI   180         ; undo previous subtraction
JUMP   CheckAngle  ; THETA positive, so carry on
NegAngle:
ADDI   -180        ; finish conversion to negative angle:
OUT    Heading


GetAngle:
; The 0/359 jump in THETA can be difficult to deal with.
; This code shows one way to handle it: by moving the
; discontinuity away from the current heading.
IN     THETA       ; get the current angular position
ADDI   -180        ; test whether facing 0-179 or 180-359
JPOS   NegAngle    ; robot facing 180-360; handle that separately
PosAngle:
ADDI   180         ; undo previous subtraction
JUMP   CheckAngle  ; THETA positive, so carry on
NegAngle:
ADDI   -180        ; finish conversion to negative angle:
;  angles 180 to 359 become -180 to -1

SUB    Heading


CheckAngle:
; AC now contains the +/- angular difference from Heading
OUT    LCD         ; Good data to display for debugging
JPOS   TurnRight   ; handle +/- separately
TurnLeft:
; If the angle is small, we don't want to do anything.
ADD    DeadZone
JPOS   NoTurn
; otherwise, turn CCW
LOAD   RSlow
JUMP   SendToMotors
TurnRight:
SUB    DeadZone    ; if near 0, don't turn
JNEG   NoTurn
; otherwise, turn CW
LOAD   FSlow
JUMP   SendToMotors
NoTurn:
LOAD   Zero
JUMP   SendToMotors

SendToMotors:
; Since we want to spin in place, we need to send inverted
;  velocities to the wheels.  The current AC value is used for
;  the left wheel, and its negative is used for the right wheel
STORE  Temp        ; store desired velocity for later
; send the direct value to the left wheel
OUT    LVELCMD
; send the negated number to the right wheel
LOAD   Zero
SUB    Temp        ; AC = 0 - velocity
OUT    RVELCMD
JUMP   GetAngle    ; repeat whole process forever

DeadZone:  DW 1
Heading:    DW  30
