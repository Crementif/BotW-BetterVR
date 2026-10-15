[BotW_MasterModeOptions_V208]
moduleMatches = 0x6267BFD0

# Constants

merge0x129 = $bit0 + $bit1 + $bit3 + $bit5 + $bit8
;Combines bits set by the user in rules.txt to a bitmask. Split between 2 patches because vanilla compiled it to 2 hex values instead of 1 for some
;reason. There's no overlap in vanilla between the 2 hex values, so I made sure each bit was set by the same instruction as in vanilla.
;(Variable names are the hex values that were in that location in vanilla)

merge0xd0 = $bit4 + $bit6 + $bit7
;Bit 2 is excluded from the logic because it does absolutely nothing. Bits 0 and 1 are connected to other parts of the game code, but don't actually
;do anything in-game.

# Patches

0x031696B8 = ori       r9, r0, merge0x129
0x031696C4 = ori       r0, r9, merge0xd0
0x02AF94FC = .int ((($mmdamage == 0) * 0x4182000C) + (($mmdamage == 1) * 0x48000005))
;This last patch is a clever hack from Crementif. Essentially, it sends the instruction in hex rather than in regular text form.
;
;When $mmdamage is 0, it sends 0x02AF94FC = beq 0x02AF9508 (the vanilla instruction).
;When $mmdamage is 1, it sends 0x02AF94FC = bl 0x02AF9500.
