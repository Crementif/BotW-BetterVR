[BotW_DrawDistance_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; Actor draw distance
actorMultiplier:
.float $actor

_setActorDrawDistanceMultiplier:
lis r9, actorMultiplier@ha
lfs f9, actorMultiplier@l(r9)
stfs f9, 0x13BC(r28)
blr

0x03857F58 = nop ; Force the draw distance used for load balancing normally to be enabled
0x03857F5C = bla _setActorDrawDistanceMultiplier


; Object draw distance
objectMultiplier:
.float $object

objectDivider:
.float 1.0 - ($object - 1.0)

_setObjectDrawDistance:
lis r12, objectMultiplier@ha
lfs f0, objectMultiplier@l(r12)
; Doesn't catch objects that don't have their traverseRadiusXZ set, so those default to 0
blr

0x03137ECC = nop
0x03137ED0 = bla _setObjectDrawDistance

_setObjectDrawDistanceDivider:
fmuls f13, f1, f1
lis r8, objectDivider@ha
lfs f1, objectDivider@l(r8)
fmuls f13, f13, f1
blr

0x0313A60C = bla _setObjectDrawDistanceDivider
0x0313A678 = bla _setObjectDrawDistanceDivider


; Tree billboard draw distance
const_treeDrawDistance:
.float $tree

0x033CB548 = lis r7, const_treeDrawDistance@ha
0x033CB550 = lfs f12, const_treeDrawDistance@l(r7)


; Grass blades draw distance
0x1030A774 = .float $grass