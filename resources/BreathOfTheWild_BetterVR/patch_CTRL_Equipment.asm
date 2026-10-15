[BetterVR_SpawnEquipment_V208]
moduleMatches = 0x6267BFD0

.origin = codecave


spawnString:
.string "Weapon_Sword_022"
.align 1
.byte 0
.align 40*4


0x10463708 = CreatePlayerEquipActorMgr__sInstance:
0x10263910 = sead__SafeString__vt:

0x02A15998 = requestCreateWeapon:

vr_spawnEquipment:
stwu r1, -0x3C(r1)
stw r0, 0x04(r1)
stw r3, 0x08(r1)
stw r4, 0x0C(r1)
stw r5, 0x10(r1)
stw r6, 0x14(r1)
stw r7, 0x18(r1)
stw r8, 0x1C(r1)
stw r9, 0x20(r1)
stw r10, 0x24(r1)
stw r11, 0x28(r1)
stw r12, 0x2C(r1)
stw r13, 0x30(r1)
; 0x34 = safestring c_str ptr
; 0x38 = safestring vtable ptr
; 0x3C = safestring data
mflr r3
stw r3, 0x40(r1)

; r3 is spawnee?
lis r3, CreatePlayerEquipActorMgr__sInstance@ha
lwz r3, CreatePlayerEquipActorMgr__sInstance@l(r3)

; r4 is the weapon/armor type
; - PouchItemType_Sword = 0x0,
; - PouchItemType_Bow = 0x1,
; - PouchItemType_Arrow = 0x2,
; - PouchItemType_Shield = 0x3,
; - PouchItemType_ArmorHead = 0x4,
; - PouchItemType_ArmorUpper = 0x5,
; - PouchItemType_ArmorLower = 0x6,
; - PouchItemType_Material = 0x7,
; - PouchItemType_Food = 0x8,
; - PouchItemType_KeyItem = 0x9,
li r4, 0x00

; r5 is the actor name
lis r5, spawnString@ha
addi r5, r5, spawnString@l
stw r5, 0x34(r1)

lis r5, sead__SafeString__vt@ha
addi r5, r5, sead__SafeString__vt@l
stw r5, 0x38(r1)

addi r5, r1, 0x34
; r6 is the weapon's life
li r6, 160
; r7 is a pointer to the weapons's modifier. Can use 0 for none.
li r7, 0


lwz r3, 0x40(r1)
mtlr r3
lwz r0, 0x04(r1)
lwz r3, 0x08(r1)
lwz r4, 0x0C(r1)
lwz r5, 0x10(r1)
lwz r6, 0x14(r1)
lwz r7, 0x18(r1)
lwz r8, 0x1C(r1)
lwz r9, 0x20(r1)
lwz r10, 0x24(r1)
lwz r11, 0x28(r1)
lwz r12, 0x2C(r1)
lwz r13, 0x30(r1)

addi r1, r1, 0x3C
blr