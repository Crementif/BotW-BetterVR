[BetterVR_SmallerCutParticles_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; shrink the grass cut and flower petal particle bursts, which the game always spawns at scale 1.0
GrassCutParticleScale:
.float 0.4

FlowerPetalParticleScale:
.float 0.65

grassCutScale:
lis r12, GrassCutParticleScale@ha
lfs f10, GrassCutParticleScale@l(r12)
stfs f10, 0x08(r1)
stfs f10, 0x0C(r1)
stfs f10, 0x10(r1)
blr

flowerPetalScale:
lis r11, FlowerPetalParticleScale@ha
lfs f10, FlowerPetalParticleScale@l(r11)
stfs f10, 0x1C(r1)
stfs f10, 0x20(r1)
stfs f10, 0x24(r1)
blr

0x0336CE24 = bla grassCutScale
0x0336CE28 = nop
0x0336CE30 = nop
0x0336CFD4 = bla flowerPetalScale
0x0336CFD8 = nop
0x0336CFE0 = nop
