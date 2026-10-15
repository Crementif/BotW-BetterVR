[BotW_Ragdolls_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

const_ragdollTimestepA:
.float 0.03333334 ; the expected timestep at 30 FPS (1/30)

; -----------------------------
; Patch RagdollApplyForce function to always use the expected timestep of 1/30
; and disable the timestep scaling as its incorrect to scale one-frame impulses by time

; f12 is the timestep in seconds (so for 30 FPS thats 1/30 -> 0.03333)
RagdollHit_TimestepPatch1:
lis r6, const_ragdollTimestepA@ha
lfs f12, const_ragdollTimestepA@lo(r6)
blr

0x0333d350 = bla RagdollHit_TimestepPatch1 ; replaced instruction: lfs f12,0x48(r25)

; disable more timestep scaling
; 0333d80c  fnmsubs f9,f9,f9,f31
0x0333d80c = fmr f9, f31

; 0333dc78  fnmsubs f9,f9,f9,f30
0x0333dc78 = fmr f9, f30

; 0333dd58  fnmsubs f9,f9,f9,f30
0x0333dd58 = fmr f9, f30


;_changeImpulsePower:
;lis r10, averageFPS1Inv@ha
;lfs f13, averageFPS1Inv@l(r10)
;lis r10, debugMultiplier@ha
;lfs f11, debugMultiplier@l(r10)
;fmuls f13, f13, f11
;fmuls f1, f1, f13
;lwz r10, 0x2C(r28)
;blr
;
;0x0333E3D0 = bla _changeImpulsePower