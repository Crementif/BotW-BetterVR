[BetterVR_FirstPersonMode_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; replaces the blr in Actor/Weapon/OptionalWeapon::CalculateModelOpacity
0x024A6F20 = ba import.coreinit.hook_CalculateModelOpacity
0x033AFA9C = ba import.coreinit.hook_CalculateModelOpacity
0x037B13A8 = ba import.coreinit.hook_CalculateModelOpacity

; hooks Actor::setOpacity() and replace it with a custom C++ function
0x037B13AC = ba import.coreinit.hook_SetActorOpacity

; sets distance from the camera
0x10216594 = const_05_10216594:
0x10216598 = const_3_10216598:

hook_UseCustomCameraDistance:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x08(r1)
stw r4, 0x04(r1)

; load camera distance like normal
lis r9, const_05_10216594@ha
lfs f13, const_05_10216594@l(r9)
lis r12, const_3_10216598@ha
lfs f12, const_3_10216598@l(r12)

bl import.coreinit.hook_UseCameraDistance

lwz r4, 0x04(r1)
lwz r3, 0x08(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

0x02E5FEB8 = bla hook_UseCustomCameraDistance
0x02E5FEBC = nop
0x02E5FEC0 = nop
0x02E5FEC4 = nop
