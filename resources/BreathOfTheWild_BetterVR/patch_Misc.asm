[BetterVR_Misc_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; disable gyro controls
0x02E1905C = li r3, 0 ; always return 0 to signal disabled
0x02E19060 = blr
0x02E19064 = li r5, 0 ; always set 0

; disable inversed button controls
0x02E199AC = li r3, 0 ; return 0 to signal disabled
0x02E199B0 = blr
0x02E199B4 = li r5, 0 ; always set 0

; forces StartShiekSensorGaugeDemo to instantly finish and not get softlocked (which happens when you unlock a tower)
; this disables the gauge animation where it would get stuck in
; not ideal but better than softlocking
0x023F90E0 = nop
0x023F90E4 = nop
0x023F90EC = nop
0x023F910C = nop

; track and suppress first-person PlayerNormal damage reaction state changes
0x02D0A558 = bla import.coreinit.hook_PlayerNormalChangeState

; neutralize PlayerLaunch movement for Link by overriding the launch float params to 0.0f
; NoRagdollTime (0x02CAE098) is deliberately left alone: it's the window during which ragdolling is suppressed, so zeroing it causes instant ragdolls
0x02CAE018 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAE038 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAE058 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAE078 = bla import.coreinit.hook_OverwriteFloatParam

; neutralize PlayerLargeDamage movement for Link by overriding its speed, jump and impulse float params to 0.0f (NoRagdollTime at 0x02CAB5AC also left alone)
0x02CAB32C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB350 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB374 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB398 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB3BC = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB3E0 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB404 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB428 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB44C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB470 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB494 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB4B8 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB4DC = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB500 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB524 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB548 = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB56C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB58C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB5CC = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB5EC = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB60C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB62C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB64C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB66C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB68C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB6AC = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB6CC = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB6EC = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB70C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB72C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB74C = bla import.coreinit.hook_OverwriteFloatParam
0x02CAB76C = bla import.coreinit.hook_OverwriteFloatParam
0x02CABAE8 = bla import.coreinit.hook_OverwriteFloatParam
0x02CABB08 = bla import.coreinit.hook_OverwriteFloatParam
0x02CABB28 = bla import.coreinit.hook_OverwriteFloatParam
0x02CABB48 = bla import.coreinit.hook_OverwriteFloatParam
0x02CABB68 = bla import.coreinit.hook_OverwriteFloatParam
0x02CABB88 = bla import.coreinit.hook_OverwriteFloatParam
