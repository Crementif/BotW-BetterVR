[BotW_KorokChasing_V208]
moduleMatches = 0x6267BFD0

.origin = codecave
; --------------------------
; This set of patches fixes the speed with which hidden Koroks (shown as sparkles on the ground) move along their path
; In unpatched state, the speed would scale directly with FPS, so at 60 FPS the Koroks would move twice as fast

_constFloatTimestepKorok:
.float 30.0

0x0348ba3c = bla _HiddenKorokMoveCalcTimestep
_HiddenKorokMoveCalcTimestep:
; replaced instruction:
; 0348ba3c fdivs f25,f27,f13
; calculates:
; f25 = 1.0 / timeStepInSeconds
; available registers: r4
lis r4, _constFloatTimestepKorok@ha
lfs f25, _constFloatTimestepKorok@lo(r4)
blr

; 0344acd8 ec 0d 58 24     fdivs      f0,f13,f11
; general korok move function? maybe should be patched too?

; Hidden korok rest time is measured in frames. Normally it also counts +1.0 each frame but we use a dynamic timestep instead
0x022ac944 = bla _HiddenKorokRestActionCalcTimestep
_HiddenKorokRestActionCalcTimestep:
; replaced instruction:
; 022ac944 fadds f0,f12,f0
; available registers: r3
lis r3, averageFPS1Inv@ha
lfs f0, averageFPS1Inv@lo(r3)
fadds f0, f12, f0
blr

