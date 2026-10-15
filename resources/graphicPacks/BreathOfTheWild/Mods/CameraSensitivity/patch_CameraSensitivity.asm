[BotW_CameraSensitivity_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; Variables
vertMovementSens:
.float ($movementSensitivity * $vertMovementSens)

horMovementSens:
.float $movementSensitivity

; Patches
; Sets sensitivity everywhere
0x102164C0 = .float ($sensitivity*-1)

; Sets vertical sensitivity for movement
0x02B9C168 = lis r9, vertMovementSens@ha    ; load address into r9
0x02B9C170 = lfs f0, vertMovementSens@l(r9) ; load value into f0

; Sets horizontal sensitivity for movement
0x02B9C944 = lis r12, horMovementSens@ha    ; load address into r12
0x02B9C94c = lfs f6, horMovementSens@l(r12) ; load value into f6
