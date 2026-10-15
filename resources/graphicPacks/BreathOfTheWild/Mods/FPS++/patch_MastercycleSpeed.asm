[BotW_MastercycleSpeed_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

0x100136B4 = originalMotorcycleBoostSpeed:


_changeMotorcycleBoostSpeed:
lfs f0, originalMotorcycleBoostSpeed@l(r9)
lis r9, averageFPS1@ha
lfs f8, averageFPS1@l(r9)
fdivs f0, f0, f8
blr


0x0209FFC0 = bla _changeMotorcycleBoostSpeed