[BotW_Durability_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

_modifierValue:
.float $multiplier

0x024ACBF0 = lis r3, _modifierValue@ha
0x024ACBF4 = lfs f31, _modifierValue@l(r3)
