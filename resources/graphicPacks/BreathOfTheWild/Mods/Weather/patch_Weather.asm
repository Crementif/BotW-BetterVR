[BotW_Weather_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

weatherBytesMap:
.byte $blueSky
.byte $cloudy
.byte $rain
.byte $heavyRain
.byte $snow
.byte $heavySnow
.byte $thunderstorm
.byte $thunderRain
.byte $blueSkyRain

_changeWeather:
lis r6, weatherBytesMap@ha			; Load first part of the weather map's address
addi r6, r6, weatherBytesMap@l		; Load second part of the address
mulli r26, r26, 0x04				; .byte values are 4 byte aligned, so multiply the offset by 0x04
lbzx r26, r6, r26					; Load the byte that's stored in the weather map using it's address plus an offset which is the original weather value to "map" each one
stb r26, 0x18(r30)					; Original instruction that would store the weather
blr									; Return back to the link register

0x03668FCC = nop					; Replace instruction that checks whether the weather has changed. Since the value is always modified/changed, just disable this branch.
0x03668FEC = bla _changeWeather		; Replace instruction that normally stores the weather with a jump to the weather changing function
