[BotW_ControllerScrollSpeed_V208]
moduleMatches = 0x6267BFD0
.origin = codecave

0x03053bd4 = blr ; disable scroll speed adjustment (would require a bigger rewrite for dynamic FPS support)
0x030BD694 = ba _ZN4sead14ControllerBase23updateDerivativeParams_Ejj ; redirect to our re-implementation

scrollSpeed:
	.float 1.0

0x10327D60 = menuNavigationSpeedPackEnabled:
0x10327D64 = menuNavigationSpeedPackValue:

_readExternalCursorSpeed:
lis r11, menuNavigationSpeedPackEnabled@ha
lwz r11, menuNavigationSpeedPackEnabled@l(r11)
cmpwi r11, 0x1337
beq useCustomSpeed
useDefaultSpeed:
lis r11, const_1@ha
lfs f12, const_1@l(r11)
lis r11, scrollSpeed@ha
stfs f12, scrollSpeed@l(r11)
b _checkExternalSpeed
useCustomSpeed:
lis r11, menuNavigationSpeedPackValue@ha
lfs f12, menuNavigationSpeedPackValue@l(r11)
lis r11, scrollSpeed@ha
stfs f12, scrollSpeed@l(r11)
b _checkExternalSpeed

_ZN4sead14ControllerBase21createStickCrossMask_Ev:
	stwu r1,-16(r1)
	stw r31,12(r1)
	mr r31,r1
	lwz r10,252(r3)
	li r9,0
	cmpwi r10,0
	BLT _87c9cb495aa50099_L2
	li r9,15
	slw r9,r9,r10
_87c9cb495aa50099_L2:
	lwz r8,256(r3)
	cmpwi r8,0
	BLT _87c9cb495aa50099_L3
	li r10,15
	slw r10,r10,r8
	or r9,r9,r10
_87c9cb495aa50099_L3:
	mr r3,r9
	addi r11,r31,16
	lwz r31,-4(r11)
	mr r1,r11
	blr
_ZN4sead14ControllerBase23updateDerivativeParams_Ejj:
	stwu r1,-128(r1)
	mflr r0
	stw r0,132(r1)
	stfd f30,112(r1)
	stfd f31,120(r1)
	stw r19,60(r1)
	stw r20,64(r1)
	stw r21,68(r1)
	stw r22,72(r1)
	stw r23,76(r1)
	stw r24,80(r1)
	stw r25,84(r1)
	stw r26,88(r1)
	stw r27,92(r1)
	stw r28,96(r1)
	stw r29,100(r1)
	stw r30,104(r1)
	stw r31,108(r1)
	mr r31,r1
	mr r30,r3
	mr r29,r4
	mr r24,r5
	lwz r6,252(r3)
	cmpwi r6,0
	BLT _87c9cb495aa50099_L7
	addi r5,r30,280
	lfs f1,232(r30)
	lfs f2,240(r30)
	lis r9,0x30b
	ori r9,r9,53900
	mtctr r9
	bctrl
	mr r28,r3
	lwz r6,256(r30)
	cmpwi r6,0
	BLT _87c9cb495aa50099_L8
_87c9cb495aa50099_L25:
	mr r3,r30
	mr r4,r29
	addi r5,r30,288
	lfs f1,236(r30)
	lfs f2,244(r30)
	lis r9,0x30b
	ori r9,r9,53900
	mtctr r9
	bctrl
	or r28,r28,r3
	lwz r8,252(r30)
	li r9,0
	cmpwi r8,0
	BLT _87c9cb495aa50099_L9
_87c9cb495aa50099_L23:
	li r10,15
	slw r9,r10,r8
_87c9cb495aa50099_L9:
	lwz r8,256(r30)
	cmpwi r8,0
	BLT _87c9cb495aa50099_L10
	li r10,15
	slw r10,r10,r8
	or r9,r9,r10
	.align 4
_87c9cb495aa50099_L10:
	lwz r10,268(r30)
	andc r10,r10,r9
	or r9,r10,r28
	stw r9,268(r30)
	andc r10,r29,r9
	stw r10,4(r30)
	andc r9,r9,r29
	stw r9,0(r30)
	li r9,0
	stw r9,8(r30)
	lis r9,averageFPS30@ha
	lfs f31,averageFPS30@l(r9)
	lis r9,scrollSpeed@ha
	lfs f30,scrollSpeed@l(r9)
	lwz r9,248(r30)
	cmpwi r9,0
	BEQ _87c9cb495aa50099_L11
	addi r27,r30,199
	addi r28,r30,104
	li r29,0
	lis r23,_87c9cb495aa50099_LC0@ha
	lis r26,_87c9cb495aa50099_LC1@ha
	li r21,1
	li r25,0
_87c9cb495aa50099_L18:
	lbzu r8,1(r27)
	lbz r10,-32(r27)
	lwz r9,268(r30)
	srw r9,r9,r29
	andi. r7,r9,1
	BNE cr0, _87c9cb495aa50099_L12
	sth r25,-64(r28)
	sth r25,0(r28)
	b _87c9cb495aa50099_L13
_87c9cb495aa50099_L12:
	mr r20,r28
	lfs f0,_87c9cb495aa50099_LC0@l(r23)
	fdivs f0,f0,f31
	fmuls f0,f0,f30
	lfs f12,_87c9cb495aa50099_LC1@l(r26)
	fcmpu f0,f12
	cror 30,29,30
	BEQ _87c9cb495aa50099_L14
	fctiwz f0,f0
	stfd f0,8(r31)
	lwz r7,12(r31)
	b _87c9cb495aa50099_L15
_87c9cb495aa50099_L14:
	lfs f12,_87c9cb495aa50099_LC1@l(r26)
	fsubs f0,f0,f12
	fctiwz f0,f0
	stfd f0,16(r31)
	lwz r7,20(r31)
	addis r7,r7,0x8000
_87c9cb495aa50099_L15:
	lhz r9,0(r20)
	add r9,r7,r9
	rlwinm r9, r9, 0, 16, 31
	cmplwi r9,1091
	BGT _87c9cb495aa50099_L16
	sth r9,0(r20)
	b _87c9cb495aa50099_L13
_87c9cb495aa50099_L16:
	addi r9,r9,-1092
	sth r9,0(r20)
	lhz r19,-64(r20)
	cmpwi r8,0
	BEQ _87c9cb495aa50099_L17
	cmplw r10,r19
	BGT _87c9cb495aa50099_L17
	subf r10,r10,r19
	divwu r9,r10,r8
	mullw r8,r9,r8
	cmpw r10,r8
	BNE _87c9cb495aa50099_L17
	slw r9,r21,r29
	lwz r10,8(r30)
	or r9,r10,r9
	stw r9,8(r30)
_87c9cb495aa50099_L17:
	addi r19,r19,1
	sth r19,-64(r20)
_87c9cb495aa50099_L13:
	addi r29,r29,1
	addi r28,r28,2
	lwz r9,248(r30)
	cmpw r29,r9
	BLT _87c9cb495aa50099_L18
_87c9cb495aa50099_L11:
	cmpwi r24,0
	BNE _87c9cb495aa50099_L19
	lwz r9,12(r30)
	andi. r8,r9,1
	BEQ cr0, _87c9cb495aa50099_L20
	rlwinm r9,r9,0,30,28
	ori r9,r9,2
	stw r9,12(r30)
	b _87c9cb495aa50099_L21
_87c9cb495aa50099_L24:
	ori r10,r10,4
	stw r10,12(r30)
	lfs f0,272(r30)
	fctiwz f0,f0
	stfd f0,24(r31)
	lwz r9,28(r31)
	stw r9,16(r30)
	lfs f0,276(r30)
	fctiwz f0,f0
	stfd f0,32(r31)
	lwz r9,36(r31)
	stw r9,20(r30)
	b _87c9cb495aa50099_L6
_87c9cb495aa50099_L20:
	li r9,0
	stw r9,12(r30)
_87c9cb495aa50099_L21:
	lfs f0,272(r30)
	fctiwz f0,f0
	stfd f0,40(r31)
	lwz r9,44(r31)
	stw r9,16(r30)
	lfs f0,276(r30)
	fctiwz f0,f0
	stfd f0,48(r31)
	lwz r9,52(r31)
	stw r9,20(r30)
	b _87c9cb495aa50099_L6
_87c9cb495aa50099_L8:
	lwz r8,252(r30)
	li r9,0
	cmpwi r8,0
	BLT _87c9cb495aa50099_L10
	b _87c9cb495aa50099_L23
_87c9cb495aa50099_L19:
	lwz r9,12(r30)
	rlwinm r10,r9,0,31,29
	andi. r7,r9,1
	BNE cr0, _87c9cb495aa50099_L20
	b _87c9cb495aa50099_L24
_87c9cb495aa50099_L7:
	lwz r6,256(r3)
	li r28,0
	li r9,0
	cmpwi r6,0
	BLT _87c9cb495aa50099_L10
	b _87c9cb495aa50099_L25
_87c9cb495aa50099_L6:
	addi r11,r31,128
	lwz r0,4(r11)
	mtlr r0
	lwz r19,-68(r11)
	lwz r20,-64(r11)
	lwz r21,-60(r11)
	lwz r22,-56(r11)
	lwz r23,-52(r11)
	lwz r24,-48(r11)
	lwz r25,-44(r11)
	lwz r26,-40(r11)
	lwz r27,-36(r11)
	lwz r28,-32(r11)
	lwz r29,-28(r11)
	lwz r30,-24(r11)
	lwz r31,-20(r11)
	lfd f30,-16(r11)
	lfd f31,-8(r11)
	mr r1,r11
	blr
_87c9cb495aa50099_LC0:
	.int	1191182336
_87c9cb495aa50099_LC1:
	.int	1325400064