[BotW_FenceMethod_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

0x031FAB00 = fullFenceAddr:
0x031FAB04 = skipFenceAddr:

conditionalFence:

# Check which fence it's running
li r11, $fenceMethod
cmpwi r11, 1
beq performanceFence
li r11, $fenceMethod
cmpwi r11, 2
beq accurateFence
li r11, $fenceMethod
cmpwi r11, 3
beq doFenceSkip

performanceFence:
li r0, 1 ; Use 1 as the fence value
subf r0, r10, r0 ; replicate previous instruction
add r6, r12, r0 ; original instruction
b doFence

accurateFence:
add r6, r12, r0 ; original instruction
cmpwi r6, 500 ; check if tick is the first 500 frames/ticks
blt doFence ; if so, do a full fence with the original value
b doFenceSkip ; after 500 frames, skip the fence

allFenceSkip:
add r6, r12, r0 ; original instruction
b doFenceSkip

doFence:
lis r11, fullFenceAddr@ha
addi r11, r11, fullFenceAddr@l 
mtctr r11
bctrl
doFenceSkip:
lis r11, skipFenceAddr@ha
addi r11, r11, skipFenceAddr@l 
mtctr r11
bctrl

0x031FAAFC = b conditionalFence