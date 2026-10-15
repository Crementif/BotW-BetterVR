[BotW_AspectRatio_Shared]
moduleMatches = 0xD91A406D,0x0F748D9C,0x9A61FF4C,0x8E3324A9,0xD71D859D,0x6FD41A61,0x9A2CA0C7,0x29DBB52A,0xFD091F9F,0xD472D8A5,0x6267BFD0

.origin = codecave

aspectRatio:
.float ($width/$height)

grassCulling: ; The grass is calculated in a weird way, but this fix seems to work. So for safety, only enable it for ultrawide resolutions.
.float (($ultrawideHUDMode != 0)*(($gameWidth/$gameHeight) / ($width/$height))) + (($ultrawideHUDMode == 0)*1.0)


[BotW_AspectRatio_V208]
moduleMatches = 0x6267BFD0

; rodata constants
0x101BF8E8 = .float ($width/$height)
0x1030A57C = .float ($width/$height)
0x1036DD4C = .float ($width/$height)

; 3D Rendering In Inventory (calculated every load)
0x02E2C564 = lis r9, aspectRatio@ha
0x02E2C578 = lfs f4, aspectRatio@l(r9)

; 3D Rendering (calculated every frame)
0x0386D01C = lis r28, aspectRatio@ha
0x0386D020 = lfs f12, aspectRatio@l(r28)

; Grass Culling (calculated every frame)
0x035b0a30 = lis r5, grassCulling@ha
0x035b0a38 = lfs f11, grassCulling@l(r5)

; 2nd Type Of Grass Culling (calculated every frame)
; Still kinda unsure how this should be calculated but it works
0x1047BFB8 = .float (($ultrawideHUDMode != 0)*(120*0.5) + (($ultrawideHUDMode == 0)*(120)))


[BotW_AspectRatio_V176V192]
moduleMatches = 0xFD091F9F,0xD472D8A5

; rodata constants
0x101BF878 = .float $width/$height
0x1030A3F4 = .float $width/$height
0x1036DBDC = .float $width/$height

; 3D Rendering In Inventory (calculated every load)
0x02E2BFC8 = lis r9, aspectRatio@ha
0x02E2BFDC = lfs f4, aspectRatio@l(r9)

; 3D Rendering (calculated every frame)
0x0386C314 = lis r28, aspectRatio@ha
0x0386C318 = lfs f12, aspectRatio@l(r28)

[BotW_AspectRatio_V160]
moduleMatches = 0x9A2CA0C7

; rodata constants
0x101A8A70 = .float $width/$height
0x102ECF88 = .float $width/$height
0x1034F684 = .float $width/$height

; 3D Rendering In Inventory (calculated every load)
0x02D53CF4 = lis r9, aspectRatio@ha
0x02D53D08 = lfs f4, aspectRatio@l(r9)

; 3D Rendering (calculated every frame)
0x0375AFF4 = lis r28, aspectRatio@ha
0x0375AFF8 = lfs f12, aspectRatio@l(r28)

[BotW_AspectRatio_V144]
moduleMatches = 0x9A2CA0C7

; rodata constants
0x101A8A70 = .float $width/$height
0x102ECFD0 = .float $width/$height
0x1034F6CC = .float $width/$height

; 3D Rendering In Inventory (calculated every load)
0x02D53CF4 = lis r9, aspectRatio@ha
0x02D53D08 = lfs f4, aspectRatio@l(r9)

; 3D Rendering (calculated every frame)
0x0375B128 = lis r28, aspectRatio@ha
0x0375B12C = lfs f12, aspectRatio@l(r28)

[BotW_AspectRatio_V112]
moduleMatches = 0x6FD41A61

; rodata constants
0x1019F9A8 = .float $width/$height
0x102DFB38 = .float $width/$height
0x103414D4 = .float $width/$height

; 3D Rendering In Inventory (calculated every load)
0x02D2F404 = lis r9, aspectRatio@ha
0x02D2F418 = lfs f4, aspectRatio@l(r9)

; 3D Rendering (calculated every frame)
0x03726E18 = lis r28, aspectRatio@ha
0x03726E1C = lfs f12, aspectRatio@l(r28)

[BotW_AspectRatio_V97]
moduleMatches = 0xD71D859D

; rodata constants
0x1019F980 = .float $width/$height
0x102DF8A0 = .float $width/$height
0x1034122C = .float $width/$height

; 3D Rendering In Inventory (calculated every load)
0x02D2E65C = lis r9, aspectRatio@ha
0x02D2E670 = lfs f4, aspectRatio@l(r9)

; 3D Rendering (calculated every frame)
0x03725CE8 = lis r28, aspectRatio@ha
0x03725CEC = lfs f12, aspectRatio@l(r28)

[BotW_AspectRatio_V80]
moduleMatches = 0x8E3324A9

; rodata constants
0x1019A5A8 = .float $width/$height
0x102D4E30 = .float $width/$height
0x103391B4 = .float $width/$height

; 3D Rendering In Inventory (calculated every load)
0x02D056C8 = lis r9, aspectRatio@ha
0x02D056DC = lfs f4, aspectRatio@l(r9)

; 3D Rendering (calculated every frame)
0x036D0324 = lis r28, aspectRatio@ha
0x036D0328 = lfs f12, aspectRatio@l(r28)

[BotW_AspectRatio_V64]
moduleMatches = 0x9A61FF4C

; rodata constants
0x10198250 = .float $width/$height
0x102CD450 = .float $width/$height
0x1033182C = .float $width/$height

; 3D Rendering In Inventory (calculated every load)
0x02CFD900 = lis r9, aspectRatio@ha
0x02CFD914 = lfs f4, aspectRatio@l(r9)

; 3D Rendering (calculated every frame)
0x036AEF04 = lis r28, aspectRatio@ha
0x036AEF08 = lfs f12, aspectRatio@l(r28)

[BotW_AspectRatio_V48]
moduleMatches = 0x0F748D9C

; rodata constants
0x10197FC0 = .float $width/$height
0x102CCFC8 = .float $width/$height
0x103313A4 = .float $width/$height

; 3D Rendering In Inventory (calculated every load)
0x02CFC2CC = lis r9, aspectRatio@ha
0x02CFC2E0 = lfs f4, aspectRatio@l(r9)

; 3D Rendering (calculated every frame)
0x036AD81C = lis r28, aspectRatio@ha
0x036AD820 = lfs f12, aspectRatio@l(r28)

[BotW_AspectRatio_V33]
moduleMatches = 0xD91A406D

; rodata constants
0x10197FC0 = .float $width/$height
0x102CCFC0 = .float $width/$height
0x10331374 = .float $width/$height

; 3D Rendering In Inventory (calculated every load)
0x02CFC260 = lis r9, aspectRatio@ha
0x02CFC274 = lfs f4, aspectRatio@l(r9)

; 3D Rendering (calculated every frame)
0x036AD410 = lis r28, aspectRatio@ha
0x036AD414 = lfs f12, aspectRatio@l(r28)