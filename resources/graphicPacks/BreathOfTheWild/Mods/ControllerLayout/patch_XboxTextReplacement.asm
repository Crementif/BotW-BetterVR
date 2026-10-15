[BotW_XboxTextReplacement_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

textReplacementFormat:
.string "    -> Button %s: Changed text to %.10s = hex: 0x%2X%c"
textReplacementCharacter:
.int 10
.align 4

wcharToCharBuffer:
.string "                        "

.align 4
str_WideA:
.byte 0x00
.byte 0x41
.byte 0x00
.byte 0x00
str_WideB:
.byte 0x00
.byte 0x42
.byte 0x00
.byte 0x00
str_WideX:
.byte 0x00
.byte 0x58
.byte 0x00
.byte 0x00
str_WideY:
.byte 0x00
.byte 0x59
.byte 0x00
.byte 0x00
.align 4

str2_WideA:
.byte 0xE0
.byte 0x40
.byte 0x00
.byte 0x00
str2_WideB:
.byte 0xE0
.byte 0x41
.byte 0x00
.byte 0x00
str2_WideX:
.byte 0xE0
.byte 0x42
.byte 0x00
.byte 0x00
str2_WideY:
.byte 0xE0
.byte 0x43
.byte 0x00
.byte 0x00
str2_WideDefault:
.byte 0xE0
.byte 0x43
.byte 0x00
.byte 0x00
.align 4

; free registers r9, r3, r31, r5, r0
; r4 holds text
; r29 + 0x80 is start of the name of the text pane
patchText:

mr r31, r6
mr r12, r4
mflr r9

li r3, $enableDebugLogging
cmpwi r3, 1
bne skipTextLogging

; Make temporary conversion of wide char to char
lis r3, wcharToCharBuffer@ha
addi r3, r3, wcharToCharBuffer@l

li r5, 1
li r6, 0
wcharToCharLoop:
lbzx r0, r4, r5
stbx r0, r3, r6
addi r5, r5, 2
addi r6, r6, 1
cmpwi r6, 10 ; only copy 10 characters
bne wcharToCharLoop

lis r5, wcharToCharBuffer@ha
addi r5, r5, wcharToCharBuffer@l

; Log the text that's being loaded
mr r0, r7
crxor 4*cr1+eq, 4*cr1+eq, 4*cr1+eq
lis r3, textReplacementFormat@ha
addi r3, r3, textReplacementFormat@l
addi r4, r29, 0x80
lwz r6, 0(r12)
lis r7, textReplacementCharacter@ha
lwz r7, textReplacementCharacter@l(r7)
bl import.coreinit.OSReport
mr r7, r0


skipTextLogging:
; Check for DoCommand button (button prompts start with T_[X]_00)
addi r3, r29, 0x80

lbz r5, 0(r3)
cmpwi r5, 0x54 ; T
bne nextButton
lbz r5, 1(r3)
cmpwi r5, 0x5F ; _
bne nextButton

; Check _00 part
lbz r5, 3(r3)
cmpwi r5, 0x5F ; _
bne nextButton
lbz r5, 4(r3)
cmpwi r5, 0x30 ; 0
bne nextButton
lbz r5, 5(r3)
cmpwi r5, 0x30 ; 0
bne nextButton

; Check which button it is
lbz r5, 2(r3)
cmpwi r5, 0x41 ; A
beq replaceButtonB
lbz r5, 2(r3)
cmpwi r5, 0x42 ; B
beq replaceButtonA
lbz r5, 2(r3)
cmpwi r5, 0x58 ; X
beq replaceButtonY
lbz r5, 2(r3)
cmpwi r5, 0x59 ; Y
beq replaceButtonX

nextButton:
; Check guide buttons in most of the other screens
addi r3, r29, 0x80

lbz r5, 0(r3)
cmpwi r5, 0x54 ; T
bne exitTextPatch
lbz r5, 1(r3)
cmpwi r5, 0x5F ; _
bne exitTextPatch
lbz r5, 2(r3)
cmpwi r5, 0x49 ; I
bne exitTextPatch
lbz r5, 3(r3)
cmpwi r5, 0x63 ; c
bne exitTextPatch
lbz r5, 4(r3)
cmpwi r5, 0x6F ; o
bne exitTextPatch
lbz r5, 5(r3)
cmpwi r5, 0x6E ; n
bne exitTextPatch
lbz r5, 6(r3)
cmpwi r5, 0x5F ; _
bne exitTextPatch
# lbz r5, 7(r3)
# cmpwi r5, 0x30 ; 0
# bne exitTextPatch

b replaceAllButtons

# lbz r5, 8(r3)
# cmpwi r5, 0x30 ; 0
# beq replaceAllButtons
# lbz r5, 8(r3)
# cmpwi r5, 0x31 ; 1
# beq replaceAllButtons
# lbz r5, 8(r3)
# cmpwi r5, 0x32 ; 2
# beq replaceAllButtons
# lbz r5, 8(r3)
# cmpwi r5, 0x33 ; 3
# beq replaceAllButtons

b exitTextPatch

; Load the current text and see what button it is
replaceAllButtons:
mr r4, r12

lbz r5, 1(r4)
cmpwi r5, 0x40 ; A
beq replaceWideButtonB

lbz r5, 1(r4)
cmpwi r5, 0x41 ; B
beq replaceWideButtonA

lbz r5, 1(r4)
cmpwi r5, 0x42 ; X
beq replaceWideButtonY

lbz r5, 1(r4)
cmpwi r5, 0x43 ; Y
beq replaceWideButtonX

; If none matched, just go to exit
b exitTextPatch

replaceButtonA:
lis r12, str_WideA@ha
addi r12, r12, str_WideA@l
b exitTextPatch

replaceButtonB:
lis r12, str_WideB@ha
addi r12, r12, str_WideB@l
b exitTextPatch

replaceButtonX:
lis r12, str_WideX@ha
addi r12, r12, str_WideX@l
b exitTextPatch

replaceButtonY:
lis r12, str_WideY@ha
addi r12, r12, str_WideY@l
b exitTextPatch

replaceWideButtonA:
lis r12, str2_WideA@ha
addi r12, r12, str2_WideA@l
b exitTextPatch

replaceWideButtonB:
lis r12, str2_WideB@ha
addi r12, r12, str2_WideB@l
b exitTextPatch

replaceWideButtonX:
lis r12, str2_WideX@ha
addi r12, r12, str2_WideX@l
b exitTextPatch

replaceWideButtonY:
lis r12, str2_WideY@ha
addi r12, r12, str2_WideY@l
b exitTextPatch

exitTextPatch:
mr r4, r12
mr r6, r31
mtlr r9

subf r31, r30, r3
blr

0x03C4BA40 = bla patchText