[BetterVR_WeaponHands_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

0x020081AC = PlayerOrEnemy__dropWeapon:
0x033BCD9C = ActorWeapons__resetBaseProc:
0x033BD09C = ActorWeapons__dropWeapon:

dropPosition:
.float 0.0
.float 0.0
.float 0.0

changeWeaponMtx:
lwz r3, 0x58(r1) ; the actor we hackily put on the stack

mflr r0
stwu r1, -0x50(r1)
stw r0, 0x54(r1)

stw r3, 0x1C(r1)
mr r3, r31

stw r3, 0x08(r1)
stw r4, 0x0C(r1)
stw r5, 0x10(r1)
stw r6, 0x14(r1)
stw r7, 0x18(r1)
stw r8, 0x20(r1)
stw r9, 0x24(r1)
stw r10, 0x28(r1)
stw r11, 0x2C(r1)
stw r12, 0x30(r1)
stfs f1, 0x34(r1)
stw r13, 0x38(r1)

; observe the resolved hand binding and check for a drop request
lwz r3, 0x1C(r1) ; the source actor
lwz r4, 0x18(r31) ; the char array of the weapon name
lwz r8, 0x0C(r1) ; the target actor
li r13, 0
bl import.coreinit.hook_ChangeWeaponMtx
stw r13, 0x3C(r1) ; store the Weapon* that should be dropped

; if drop weapon is set to 1, we will drop the weapon
cmpwi r11, 0
beq noWeaponDrop


; 0x3C(r1) holds the Weapon* the C++ hook wants dropped. Resolve its equip slot first, so the drop
; and the matching BaseProcLink reset can't end up targeting different slots.
dropWeapon:
lwz r13, 0x38(r1) ; restore the small data area base before re-entering game code
lwz r0, 0x3C(r1)
cmpwi r0, 0
beq noWeaponDrop

lwz r3, 0x1C(r1) ; r3 = Actor*
lwz r12, 0xE8(r3)
lwz r12, 0x314(r12)
mtctr r12
bctrl ; call PlayerOrEnemy::m_97_getWeapons
cmpwi r3, 0
beq noWeaponDrop
stw r3, 0x40(r1) ; the ActorWeapons*

; ActorWeapons::getEquippedWeapon reads back empty from this thread, see hook_FindWeaponEquipSlot.
; The hook also fills in dropPosition while it has the weapon.
lwz r4, 0x3C(r1) ; r4 = the Weapon* to drop
bl import.coreinit.hook_FindWeaponEquipSlot
cmpwi r3, 0
blt noWeaponDrop ; the held actor isn't in any equip slot, so leave it alone entirely
stw r3, 0x44(r1) ; the slot that holds the weapon

; ActorWeapons::dropWeapon opens with that same call, so do the two halves it would have done here
lwz r3, 0x3C(r1) ; r3 = Weapon*
lwz r12, 0xE8(r3)
lwz r12, 0x57C(r12)
mtctr r12
lis r4, dropPosition@ha
addi r4, r4, dropPosition@l
li r5, 0 ; spin-y drop caused by electricity
li r6, 0 ; also kinda spin-y
li r7, 0 ; this is some pointer?
li r8, 0 ; broke
bctrl ; call Weapon::m_174_dropWeapon

lis r12, ActorWeapons__resetBaseProc@ha
addi r12, r12, ActorWeapons__resetBaseProc@l
mtctr r12
lwz r3, 0x40(r1) ; r3 = ActorWeapons*
lwz r4, 0x44(r1) ; r4 = the slot that held the weapon
bctrl ; call ActorWeapons::resetBaseProc

noWeaponDrop:
lwz r3, 0x08(r1)
lwz r4, 0x0C(r1)
lwz r5, 0x10(r1)
lwz r6, 0x14(r1)
lwz r7, 0x18(r1)
lwz r8, 0x20(r1)
lwz r9, 0x24(r1)
lwz r10, 0x28(r1)
lwz r11, 0x2C(r1)
lwz r12, 0x30(r1)
lfs f1, 0x34(r1)
lwz r13, 0x38(r1)

lwz r0, 0x54(r1)
mtlr r0
addi r1, r1, 0x50

blr

0x0312587C = bla changeWeaponMtx

0x0312575C = stwu r1, -0x54(r1)
0x03125898 = addi r1, r1, 0x54
0x03125900 = addi r1, r1, 0x54
0x03125924 = addi r1, r1, 0x54

storeR7OnStack:
stw r3, 0x58(r1) ; store result of the function
mr. r7, r3
blr

0x0312577C = bla storeR7OnStack

; PlayerOrEnemy__dropWeapon
custom_PlayerOrEnemy__dropWeapon:
stwu r1, -0x20(r1)
stmw r26, 8(r1)
mflr r0
stw r0, 0x24(r1)

bl import.coreinit.hook_DropWeaponLogging

mr r31, r4
mr r26, r5
mr r27, r6
mr r28, r7
mr r30, r9
mr r29, r8
lwz r12, 0xE8(r3)
lwz r0, 0x314(r12)
mtctr r0
bctrl

mr r4, r31 ; idx
mr r5, r26 ; a3
mr r6, r27 ; a4
mr r7, r28 ; a5
mr r8, r29 ; a6
mr r9, r30 ; broke
bl ActorWeapons__dropWeapon
lmw r26, 8(r1)
lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20
blr

0x020081AC = ba custom_PlayerOrEnemy__dropWeapon
