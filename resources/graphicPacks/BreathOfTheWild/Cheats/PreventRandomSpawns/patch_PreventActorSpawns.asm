[BotW_PreventActorSpawns_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

newLineFormat:
.string "Game is trying to place an actor with type of %s...%c"
newLineCharacter:
.int 10
.align 4

preventAutoPlacementSpawn:
addi r4, r1, 0x40
lwz r4, 0(r4)
# cmpwi r4, 5000
# bgtlr

; Print auto-placement
mr r24, r3
crxor 4*cr1+eq, 4*cr1+eq, 4*cr1+eq
lis r3, newLineFormat@ha
addi r3, r3, newLineFormat@l
lis r5, newLineCharacter@ha
lwz r5, newLineCharacter@l(r5)
mflr r6
bl import.coreinit.OSReport
mtlr r6
mr r3, r24

addi r4, r1, 0x40
lwz r4, 0(r4)

; Now do all of the checks
checkYigaSpawns:
li r24, $preventYigaClan
cmpwi r24, 1
bne checkStalkoblinSpawns

lbz r24, 0(r4)
cmpwi r24, 0x45 ; E
bne checkStalkoblinSpawns
lbz r24, 1(r4)
cmpwi r24, 0x6E ; n
bne checkStalkoblinSpawns
lbz r24, 2(r4)
cmpwi r24, 0x65 ; e
bne checkStalkoblinSpawns
lbz r24, 3(r4)
cmpwi r24, 0x6D ; m
bne checkStalkoblinSpawns
lbz r24, 4(r4)
cmpwi r24, 0x79 ; y
bne checkStalkoblinSpawns
lbz r24, 5(r4)
cmpwi r24, 0x5F ; _
bne checkStalkoblinSpawns
lbz r24, 6(r4)
cmpwi r24, 0x41 ; A
bne checkStalkoblinSpawns
lbz r24, 7(r4)
cmpwi r24, 0x73 ; s
bne checkStalkoblinSpawns
lbz r24, 8(r4)
cmpwi r24, 0x73 ; s
bne checkStalkoblinSpawns
lbz r24, 9(r4)
cmpwi r24, 0x61 ; a
bne checkStalkoblinSpawns
lbz r24, 10(r4)
cmpwi r24, 0x73 ; s
bne checkStalkoblinSpawns
lbz r24, 11(r4)
cmpwi r24, 0x73 ; s
bne checkStalkoblinSpawns
lbz r24, 12(r4)
cmpwi r24, 0x69 ; i
bne checkStalkoblinSpawns
lbz r24, 13(r4)
cmpwi r24, 0x6E ; n
bne checkStalkoblinSpawns
lbz r24, 14(r4)
cmpwi r24, 0x5F ; _
bne checkStalkoblinSpawns

b preventSpawnExit


checkStalkoblinSpawns:
li r24, $preventStalkoblin
cmpwi r24, 1
bne checkStalmoblinSpawns

lbz r24, 0(r4)
cmpwi r24, 0x45 ; E
bne checkStalmoblinSpawns
lbz r24, 1(r4)
cmpwi r24, 0x6E ; n
bne checkStalmoblinSpawns
lbz r24, 2(r4)
cmpwi r24, 0x65 ; e
bne checkStalmoblinSpawns
lbz r24, 3(r4)
cmpwi r24, 0x6D ; m
bne checkStalmoblinSpawns
lbz r24, 4(r4)
cmpwi r24, 0x79 ; y
bne checkStalmoblinSpawns
lbz r24, 5(r4)
cmpwi r24, 0x5F ; _
bne checkStalmoblinSpawns
lbz r24, 6(r4)
cmpwi r24, 0x42 ; B
bne checkStalmoblinSpawns
lbz r24, 7(r4)
cmpwi r24, 0x6F ; o
bne checkStalmoblinSpawns
lbz r24, 8(r4)
cmpwi r24, 0x6B ; k
bne checkStalmoblinSpawns
lbz r24, 9(r4)
cmpwi r24, 0x6F ; o
bne checkStalmoblinSpawns
lbz r24, 10(r4)
cmpwi r24, 0x62 ; b
bne checkStalmoblinSpawns
lbz r24, 11(r4)
cmpwi r24, 0x6C ; l
bne checkStalmoblinSpawns
lbz r24, 12(r4)
cmpwi r24, 0x69 ; i
bne checkStalmoblinSpawns
lbz r24, 13(r4)
cmpwi r24, 0x6E ; n
bne checkStalmoblinSpawns
lbz r24, 14(r4)
cmpwi r24, 0x5F ; _
bne checkStalmoblinSpawns
lbz r24, 15(r4)
cmpwi r24, 0x42 ; B
bne checkStalmoblinSpawns
lbz r24, 16(r4)
cmpwi r24, 0x6F ; o
bne checkStalmoblinSpawns
lbz r24, 17(r4)
cmpwi r24, 0x6E ; n
bne checkStalmoblinSpawns
lbz r24, 18(r4)
cmpwi r24, 0x65 ; e
bne checkStalmoblinSpawns

b preventSpawnExit


checkStalmoblinSpawns:
li r24, $preventStalmoblin
cmpwi r24, 1
bne checkStalizalfosSpawns

lbz r24, 0(r4)
cmpwi r24, 0x45 ; E
bne checkStalizalfosSpawns
lbz r24, 1(r4)
cmpwi r24, 0x6E ; n
bne checkStalizalfosSpawns
lbz r24, 2(r4)
cmpwi r24, 0x65 ; e
bne checkStalizalfosSpawns
lbz r24, 3(r4)
cmpwi r24, 0x6D ; m
bne checkStalizalfosSpawns
lbz r24, 4(r4)
cmpwi r24, 0x79 ; y
bne checkStalizalfosSpawns
lbz r24, 5(r4)
cmpwi r24, 0x5F ; _
bne checkStalizalfosSpawns
lbz r24, 6(r4)
cmpwi r24, 0x4D ; M
bne checkStalizalfosSpawns
lbz r24, 7(r4)
cmpwi r24, 0x6F ; o
bne checkStalizalfosSpawns
lbz r24, 8(r4)
cmpwi r24, 0x72 ; r
bne checkStalizalfosSpawns
lbz r24, 9(r4)
cmpwi r24, 0x69 ; i
bne checkStalizalfosSpawns
lbz r24, 10(r4)
cmpwi r24, 0x62 ; b
bne checkStalizalfosSpawns
lbz r24, 11(r4)
cmpwi r24, 0x6C ; l
bne checkStalizalfosSpawns
lbz r24, 12(r4)
cmpwi r24, 0x69 ; i
bne checkStalizalfosSpawns
lbz r24, 13(r4)
cmpwi r24, 0x6E ; n
bne checkStalizalfosSpawns
lbz r24, 14(r4)
cmpwi r24, 0x5F ; _
bne checkStalizalfosSpawns
lbz r24, 15(r4)
cmpwi r24, 0x42 ; B
bne checkStalizalfosSpawns
lbz r24, 16(r4)
cmpwi r24, 0x6F ; o
bne checkStalizalfosSpawns
lbz r24, 17(r4)
cmpwi r24, 0x6E ; n
bne checkStalizalfosSpawns
lbz r24, 18(r4)
cmpwi r24, 0x65 ; e
bne checkStalizalfosSpawns

b preventSpawnExit


checkStalizalfosSpawns:
li r24, $preventStalizalfos
cmpwi r24, 1
bne checkChuchuSpawns

lbz r24, 0(r4)
cmpwi r24, 0x45 ; E
bne checkChuchuSpawns
lbz r24, 1(r4)
cmpwi r24, 0x6E ; n
bne checkChuchuSpawns
lbz r24, 2(r4)
cmpwi r24, 0x65 ; e
bne checkChuchuSpawns
lbz r24, 3(r4)
cmpwi r24, 0x6D ; m
bne checkChuchuSpawns
lbz r24, 4(r4)
cmpwi r24, 0x79 ; y
bne checkChuchuSpawns
lbz r24, 5(r4)
cmpwi r24, 0x5F ; _
bne checkChuchuSpawns
lbz r24, 6(r4)
cmpwi r24, 0x4C ; L
bne checkChuchuSpawns
lbz r24, 7(r4)
cmpwi r24, 0x69 ; i
bne checkChuchuSpawns
lbz r24, 8(r4)
cmpwi r24, 0x7A ; z
bne checkChuchuSpawns
lbz r24, 9(r4)
cmpwi r24, 0x61 ; a
bne checkChuchuSpawns
lbz r24, 10(r4)
cmpwi r24, 0x6C ; l
bne checkChuchuSpawns
lbz r24, 11(r4)
cmpwi r24, 0x66 ; f
bne checkChuchuSpawns
lbz r24, 12(r4)
cmpwi r24, 0x6F ; o
bne checkChuchuSpawns
lbz r24, 13(r4)
cmpwi r24, 0x73 ; s
bne checkChuchuSpawns
lbz r24, 14(r4)
cmpwi r24, 0x5F ; _
bne checkChuchuSpawns
lbz r24, 15(r4)
cmpwi r24, 0x42 ; B
bne checkChuchuSpawns
lbz r24, 16(r4)
cmpwi r24, 0x6F ; o
bne checkChuchuSpawns
lbz r24, 17(r4)
cmpwi r24, 0x6E ; n
bne checkChuchuSpawns
lbz r24, 18(r4)
cmpwi r24, 0x65 ; e
bne checkChuchuSpawns

b preventSpawnExit


checkChuchuSpawns:
li r24, $preventChuchu
cmpwi r24, 1
bne checkKeeseSpawns

lbz r24, 0(r4)
cmpwi r24, 0x45 ; E
bne checkKeeseSpawns
lbz r24, 1(r4)
cmpwi r24, 0x6E ; n
bne checkKeeseSpawns
lbz r24, 2(r4)
cmpwi r24, 0x65 ; e
bne checkKeeseSpawns
lbz r24, 3(r4)
cmpwi r24, 0x6D ; m
bne checkKeeseSpawns
lbz r24, 4(r4)
cmpwi r24, 0x79 ; y
bne checkKeeseSpawns
lbz r24, 5(r4)
cmpwi r24, 0x5F ; _
bne checkKeeseSpawns
lbz r24, 6(r4)
cmpwi r24, 0x43 ; C
bne checkKeeseSpawns
lbz r24, 7(r4)
cmpwi r24, 0x68 ; h
bne checkKeeseSpawns
lbz r24, 8(r4)
cmpwi r24, 0x75 ; u
bne checkKeeseSpawns
lbz r24, 9(r4)
cmpwi r24, 0x63 ; c
bne checkKeeseSpawns
lbz r24, 10(r4)
cmpwi r24, 0x68 ; h
bne checkKeeseSpawns
lbz r24, 11(r4)
cmpwi r24, 0x75 ; u
bne checkKeeseSpawns

b preventSpawnExit


checkKeeseSpawns:
li r24, $preventKeese
cmpwi r24, 1
bne checkOctarockSpawns

lbz r24, 0(r4)
cmpwi r24, 0x45 ; E
bne checkOctarockSpawns
lbz r24, 1(r4)
cmpwi r24, 0x6E ; n
bne checkOctarockSpawns
lbz r24, 2(r4)
cmpwi r24, 0x65 ; e
bne checkOctarockSpawns
lbz r24, 3(r4)
cmpwi r24, 0x6D ; m
bne checkOctarockSpawns
lbz r24, 4(r4)
cmpwi r24, 0x79 ; y
bne checkOctarockSpawns
lbz r24, 5(r4)
cmpwi r24, 0x5F ; _
bne checkOctarockSpawns
lbz r24, 6(r4)
cmpwi r24, 0x4B ; K
bne checkOctarockSpawns
lbz r24, 7(r4)
cmpwi r24, 0x65 ; e
bne checkOctarockSpawns
lbz r24, 8(r4)
cmpwi r24, 0x65 ; e
bne checkOctarockSpawns
lbz r24, 9(r4)
cmpwi r24, 0x73 ; s
bne checkOctarockSpawns
lbz r24, 10(r4)
cmpwi r24, 0x65 ; e
bne checkOctarockSpawns

b preventSpawnExit


checkOctarockSpawns:
li r24, $preventOctarock
cmpwi r24, 1
bne checkLizalfosSpawns

lbz r24, 0(r4)
cmpwi r24, 0x45 ; E
bne checkLizalfosSpawns
lbz r24, 1(r4)
cmpwi r24, 0x6E ; n
bne checkLizalfosSpawns
lbz r24, 2(r4)
cmpwi r24, 0x65 ; e
bne checkLizalfosSpawns
lbz r24, 3(r4)
cmpwi r24, 0x6D ; m
bne checkLizalfosSpawns
lbz r24, 4(r4)
cmpwi r24, 0x79 ; y
bne checkLizalfosSpawns
lbz r24, 5(r4)
cmpwi r24, 0x5F ; _
bne checkLizalfosSpawns
lbz r24, 6(r4)
cmpwi r24, 0x4F ; O
bne checkLizalfosSpawns
lbz r24, 7(r4)
cmpwi r24, 0x63 ; c
bne checkLizalfosSpawns
lbz r24, 8(r4)
cmpwi r24, 0x74 ; t
bne checkLizalfosSpawns
lbz r24, 9(r4)
cmpwi r24, 0x61 ; a
bne checkLizalfosSpawns
lbz r24, 10(r4)
cmpwi r24, 0x72 ; r
bne checkLizalfosSpawns
lbz r24, 11(r4)
cmpwi r24, 0x6F ; o
bne checkLizalfosSpawns
lbz r24, 12(r4)
cmpwi r24, 0x63 ; c
bne checkLizalfosSpawns
lbz r24, 13(r4)
cmpwi r24, 0x6B ; k
bne checkLizalfosSpawns

b preventSpawnExit


checkLizalfosSpawns:
li r24, $preventLizalfos
cmpwi r24, 1
bne checkAnimalSpawns

lbz r24, 0(r4)
cmpwi r24, 0x45 ; E
bne checkAnimalSpawns
lbz r24, 1(r4)
cmpwi r24, 0x6E ; n
bne checkAnimalSpawns
lbz r24, 2(r4)
cmpwi r24, 0x65 ; e
bne checkAnimalSpawns
lbz r24, 3(r4)
cmpwi r24, 0x6D ; m
bne checkAnimalSpawns
lbz r24, 4(r4)
cmpwi r24, 0x79 ; y
bne checkAnimalSpawns
lbz r24, 5(r4)
cmpwi r24, 0x5F ; _
bne checkAnimalSpawns
lbz r24, 6(r4)
cmpwi r24, 0x4C ; L
bne checkAnimalSpawns
lbz r24, 7(r4)
cmpwi r24, 0x69 ; i
bne checkAnimalSpawns
lbz r24, 8(r4)
cmpwi r24, 0x7A ; z
bne checkAnimalSpawns
lbz r24, 9(r4)
cmpwi r24, 0x61 ; a
bne checkAnimalSpawns
lbz r24, 10(r4)
cmpwi r24, 0x6C ; l
bne checkAnimalSpawns
lbz r24, 11(r4)
cmpwi r24, 0x66 ; f
bne checkAnimalSpawns
lbz r24, 12(r4)
cmpwi r24, 0x6F ; o
bne checkAnimalSpawns
lbz r24, 13(r4)
cmpwi r24, 0x73 ; s
bne checkAnimalSpawns

b preventSpawnExit


checkAnimalSpawns:
li r24, $preventAnimals
cmpwi r24, 1
bne checkEnemySpawns

lbz r24, 0(r4)
cmpwi r24, 0x41 ; A
bne checkEnemySpawns
lbz r24, 1(r4)
cmpwi r24, 0x6E ; n
bne checkEnemySpawns
lbz r24, 2(r4)
cmpwi r24, 0x69 ; i
bne checkEnemySpawns
lbz r24, 3(r4)
cmpwi r24, 0x6D ; m
bne checkEnemySpawns
lbz r24, 4(r4)
cmpwi r24, 0x61 ; a
bne checkEnemySpawns
lbz r24, 5(r4)
cmpwi r24, 0x6C ; l
bne checkEnemySpawns
lbz r24, 6(r4)
cmpwi r24, 0x5F ; _
bne checkEnemySpawns

b preventSpawnExit


checkEnemySpawns:
li r24, $preventEnemies
cmpwi r24, 1
bne keepSpawnExit

lbz r24, 0(r4)
cmpwi r24, 0x45 ; E
bne keepSpawnExit
lbz r24, 1(r4)
cmpwi r24, 0x6E ; n
bne keepSpawnExit
lbz r24, 2(r4)
cmpwi r24, 0x65 ; e
bne keepSpawnExit
lbz r24, 3(r4)
cmpwi r24, 0x6D ; m
bne keepSpawnExit
lbz r24, 4(r4)
cmpwi r24, 0x79 ; y
bne keepSpawnExit
lbz r24, 5(r4)
cmpwi r24, 0x5F ; _
bne keepSpawnExit

b preventSpawnExit


; Exits
preventSpawnExit:
li r3, 0
cmpwi r3, 1 ; will always have already spawned
blr
keepSpawnExit:
cmpwi r3, 0 ; original instruction
blr

0x029FFB3C = bla preventAutoPlacementSpawn