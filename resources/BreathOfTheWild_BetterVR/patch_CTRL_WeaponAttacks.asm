[BetterVR_WeaponAttacks_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; change weapon sensor to be sort of inactive when not swung in first-person mode
; this prevents hitting enemies and NPCs when you don't want to, + grass

0x0339488C = ksys_act_hasTag:

hook_enableWeaponAttack:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)
stw r7, 0x0C(r1)

stw r3, 0x04(r1) ; store Actor* in stack
stw r31, 0x08(r1) ; store Weapon* in stack

; check if this is an EquipStand (the weapon stands in Link's house)
lis r3, ksys_act_hasTag@ha
addi r3, r3, ksys_act_hasTag@l
mtctr r3
lwz r3, 0x08(r1) ; r3 = Weapon*
addi r3, r3, 0x610
lis r4, -0x438E
addi r4, r4, 0x3F22 # Tag_IsEquipStand (0xBC723F22)
bctrl
cmpwi r3, 1 ; skip if weapon is held by EquipStand
beq exit_hook_enableWeaponAttack

; call Weapon::isHolding
lwz r3, 0x08(r1) ; r3 = Weapon*
lwz r4, 0xE8(r3) ; load vtable
lwz r4, 0x4CC(r4) ; get Weapon::isHolding function pointer
mtctr r4
lwz r3, 0x08(r1) ; r3 = Weapon*
lwz r4, 0x04(r1) ; r4 = Actor*
bctrl
mr r6, r3 ; r6 = Weapon::isHolding()

; check if parent is player
lwz r3, 0x08(r1) ; r3 = Weapon*
lwz r3, 0xE8(r3) ; load vtable
lwz r3, 0x4EC(r3) ; get Weapon::isParentPlayer function pointer
mtctr r3
lwz r3, 0x08(r1) ; r3 = Weapon*
bctrl
cmpwi r3, 0
beq exit_hook_enableWeaponAttack

lwz r3, 0x08(r1) ; r3 = Weapon*
lwz r5, 0x5F4(r3) ; r5 = Weapon::heldIndex

; check if weapon should have its attack sensor enabled
checkAndEnableWeaponAttackSensor:
lis r7, currentFrameCounter@ha
lwz r7, currentFrameCounter@l(r7)
bla import.coreinit.hook_EnableWeaponAttackSensor
b exit_hook_enableWeaponAttack

exit_hook_enableWeaponAttack:
lwz r7, 0x0C(r1)
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20

lbz r0, 0x8A0(r31)
blr

0x024AA7C4 = bla hook_enableWeaponAttack

activateAttackSensor_formatString:
.string "ActiveAttackSensor( type = %u, attr = %08X, power = %f, impulse = %f, powerReduce = %f, weaponParams = %08X, shieldBreakPower = %u, powerForPlayer = %u )"

custom_activateAttackSensor:
; === original function ===
stw r4, 0x10(r3) ; type
stw r5, 0x14(r3) ; attrFlags
stw r6, 0x1C(r3) ; powerOrDamage
stw r7, 0x20(r3) ; impactOrImpulseValue
stfs f1, 0x24(r3) ; powerReduce
stw r8, 0x28(r3) ; weaponParams
stw r9, 0x2C(r3) ; unk1_seemsZero
stw r10, 0x30(r3) ; shieldBreakPower
lwz r0, 0xC(r1)
stw r0, 0x34(r3) ; powerForPlayer
lwz r11, 0x10(r1)
stw r11, 0x38(r3) ; unk2_minus1ForAttack
lbz r12, 0xB(r1)
stb r12, 0x40(r3) ; alwaysZeroSeemingly

; === logging function ===
mflr r0
stwu r1, -0x40(r1)
stw r0, 0x44(r1)
stw r3, 0x3C(r1)
stw r4, 0x38(r1)
stw r5, 0x34(r1)
stw r6, 0x30(r1)
stfs f1, 0x2C(r1)
stfs f2, 0x28(r1)
stfs f3, 0x24(r1)
stfs f4, 0x20(r1)
stfs f5, 0x1C(r1)
stfs f6, 0x18(r1)

lwz r3, 0x3C(r1)
; format string
lis r5, activateAttackSensor_formatString@ha
addi r5, r5, activateAttackSensor_formatString@l
; r6 = type
lwz r6, 0x10(r3)
; r7 = attr
lwz r7, 0x14(r3)
; f1 = powerOrDamage
lfs f1, 0x1C(r3)
; f2 = impactOrImpulseValue
lfs f2, 0x20(r3)
; f3 = powerReduce
lfs f3, 0x24(r3)
; r8 = weaponParams
lwz r8, 0x28(r3)
; r9 = shieldBreakPower
lwz r9, 0x30(r3)
; r10 = powerForPlayer
lwz r10, 0x34(r3)

bl printToCemuConsoleWithFormat

lfs f1, 0x2C(r1)
lfs f2, 0x28(r1)
lfs f3, 0x24(r1)
lfs f4, 0x20(r1)
lfs f5, 0x1C(r1)
lfs f6, 0x18(r1)
lwz r3, 0x3C(r1)
lwz r4, 0x38(r1)
lwz r5, 0x34(r1)
lwz r6, 0x30(r1)
lfs f1, 0x2C(r1)
lwz r0, 0x44(r1)
addi r1, r1, 0x40
mtlr r0
blr

0x02C14F90 = ba custom_activateAttackSensor