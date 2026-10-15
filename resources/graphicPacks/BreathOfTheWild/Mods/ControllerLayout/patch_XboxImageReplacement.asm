[BotW_XboxImageReplacement_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

str_Nt_KeyTexA_00_d:
.string "Nt_KeyTexA_00^d.bflim"
str_Nt_KeyTexB_00_d:
.string "Nt_KeyTexB_00^d.bflim"
str_Nt_KeyTexX_00_d:
.string "Nt_KeyTexX_00^d.bflim"
str_Nt_KeyTexY_00_d:
.string "Nt_KeyTexY_00^d.bflim"

loadLineFormat:
.string "    -> Loaded .bflym: %s - Replaced with: %s %c"
loadLineReplacement:
.string "None"
loadLineCharacter:
.int 10
.align 4


; compares the string from r4 and r5
; r4 is untouched, so use that to do multiple comparisons
; other registers used: r0, r11, r3, r31
tempR4:
.int 0
tempR5:
.int 0
tempR6:
.int 0
tempR7:
.int 0
tempR0:
.int 0
tempR11:
.int 0
tempR3:
.int 0
tempR13:
.int 0
tempR31:
.int 0

; Unused registers: r12, r0, r31, r10, r6
_replaceButtonPrompts:
; Check if image filename isn't 0
cmpwi r5, 0
beqlr

mflr r13

; Backup registers
lis r12, tempR3@ha
stw r3, tempR3@l(r12)
lis r12, tempR4@ha
stw r4, tempR4@l(r12)
lis r12, tempR5@ha
stw r5, tempR5@l(r12)
lis r12, tempR6@ha
stw r6, tempR6@l(r12)
lis r12, tempR7@ha
stw r7, tempR7@l(r12)
lis r12, tempR0@ha
stw r0, tempR0@l(r12)
lis r12, tempR11@ha
stw r11, tempR11@l(r12)
lis r12, tempR13@ha
stw r13, tempR13@l(r12)
lis r12, tempR31@ha
stw r31, tempR31@l(r12)

; Check for buttons from e.g. DoCommand layout (used when holding something, climbing, jumping off a cliff etc)
lis r4, str_Nt_KeyTexA_00_d@ha
addi r4, r4, str_Nt_KeyTexA_00_d@l
bla _compareString
beq replaceWithBButton
lis r4, str_Nt_KeyTexB_00_d@ha
addi r4, r4, str_Nt_KeyTexB_00_d@l
bla _compareString
beq replaceWithAButton
lis r4, str_Nt_KeyTexX_00_d@ha
addi r4, r4, str_Nt_KeyTexX_00_d@l
bla _compareString
beq replaceWithYButton
lis r4, str_Nt_KeyTexY_00_d@ha
addi r4, r4, str_Nt_KeyTexY_00_d@l
bla _compareString
beq replaceWithXButton

li r3, $enableDebugLogging
cmpwi r3, 1
bne skipImageLogging

; Log the image that's trying to get loaded, r5 already contains the image name
crxor 4*cr1+eq, 4*cr1+eq, 4*cr1+eq
lis r3, loadLineFormat@ha
addi r3, r3, loadLineFormat@l
mr r4, r5
lis r5, loadLineReplacement@ha
addi r5, r5, loadLineReplacement@l
lis r6, loadLineCharacter@ha
lwz r6, loadLineCharacter@l(r6)
bl import.coreinit.OSReport

skipImageLogging:

; If no matches, exit the hook
lis r12, tempR5@ha
lwz r5, tempR5@l(r12)
b exitHook

replaceWithAButton:
lis r6, str_Nt_KeyTexA_00_d@ha
addi r6, r6, str_Nt_KeyTexA_00_d@l
b logReplacement

replaceWithBButton:
lis r6, str_Nt_KeyTexB_00_d@ha
addi r6, r6, str_Nt_KeyTexB_00_d@l
b logReplacement

replaceWithXButton:
lis r6, str_Nt_KeyTexX_00_d@ha
addi r6, r6, str_Nt_KeyTexX_00_d@l
b logReplacement

replaceWithYButton:
lis r6, str_Nt_KeyTexY_00_d@ha
addi r6, r6, str_Nt_KeyTexY_00_d@l
b logReplacement


logReplacement:
; Log the image that's trying to get loaded, r5 already contains the image name
crxor 4*cr1+eq, 4*cr1+eq, 4*cr1+eq
lis r3, loadLineFormat@ha
addi r3, r3, loadLineFormat@l
mr r4, r5
mr r5, r6
lis r6, loadLineCharacter@ha
lwz r6, loadLineCharacter@l(r6)
bl import.coreinit.OSReport
b exitHook

exitHook:
lis r12, tempR3@ha
lwz r3, tempR3@l(r12)
lis r12, tempR4@ha
lwz r4, tempR4@l(r12)
lis r12, tempR6@ha
lwz r6, tempR6@l(r12)
lis r12, tempR7@ha
lwz r7, tempR7@l(r12)
lis r12, tempR0@ha
lwz r0, tempR0@l(r12)
lis r12, tempR11@ha
lwz r11, tempR11@l(r12)
lis r12, tempR13@ha
lwz r13, tempR13@l(r12)
lis r12, tempR31@ha
lwz r31, tempR31@l(r12)

mtlr r13
lwz r12, 0(r3)
blr

0x03C48B68 = bla _replaceButtonPrompts

; compares the string from r4 and r5
; r4 is untouched, so use that to do multiple comparisons
; other registers used: r0, r11, r3, r31
_compareString:
mr r31, r5

startLoop:
lbz r0, 0(r31)
lbz r3, 0(r4)

cmpwi r0, 0
bne checkForMatch
cmpwi r3, 0
bne checkForMatch
li r4, 1
cmpwi r4, 1
blr
 
checkForMatch:
cmpw r0, r3
bne noMatch
addi r31, r31, 1
addi r4, r4, 1
b startLoop
 
noMatch:
li r4, 0
cmpwi r4, 1
blr