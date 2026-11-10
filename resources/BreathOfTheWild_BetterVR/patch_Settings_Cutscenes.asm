[BetterVR_CutsceneSettings_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; Output is derived ONLY from flags (no categories). Each entry is: <DEMO_ID>,<FP>,<PAN>,<HND>,<CTRL>
; FP: FP_ON/FP_OFF (Use First Person)
; PAN: PAN_OFF/PAN_ON (Ignore Relative Camera Movement / Disable Panning)
; HND: HND_ON/HND_OFF (Disable player hands control / Use animated hands)
; CTRL: CTRL_ON/CTRL_OFF (Enable camera input; derived from Demo_EnableCameraInput node presence)
; PROMPT: PROMPT_ON (optional, offer the cutscene camera switch prompt)
; Entries without flags are commented out.
data_TableOfCutsceneEventsSettings:


.string "100enemy,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; Indeed mentions absolute coordinates, so best to make it third-person.
.string "AncientBall_Kakariko,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; Kinda Confusing. Link is getting shouted at for touching the ball.
.string "Animal_Forest,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; ?? does it show the little blurps in third person now
.string "Animal_SunazarashiSP_C,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Aoc2Resident,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AocResident,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "ArmorProcessingEventTag,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacementNear_Enemy_Assassin_Middle,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacementNear_Enemy_Assassin_Shooter_Junior,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacementNear_Enemy_Chuchu,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacementNear_Enemy_Dragon,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacementNear_Enemy_Lizalfos,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacementNear_Enemy_Octarock,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacementNear_Enemy_Stal,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacement_Animal,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacement_Bird,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacement_Enemy_Golem_Little,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacement_Enemy_Keese,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacement_Enemy_Lizalfos,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacement_Enemy_Octarock,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacement_Enemy_Wizzrobe,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacement_Fish,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacement_Insect,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "AutoPlacement_Material,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "BalladOfHeroCommon,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; Probably not used directly since its common.
.string "BalladOfHeroes,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "BalladOfHeroGerudo,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "BalladOfHeroGoron,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "BalladOfHeroRito,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "BalladOfHeroZora,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; Contains a ton of stuff. Hard to tell.
.string "bf2_collabo,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "BloodyMoonRelief,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Bottle_Mes,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Carnivorous_Boy,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "CarryingBlueFireEXMini,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "ClearRemains,FP_ON,PAN_ON,HND_OFF,CTRL_ON" ; GraceGet
.string "Cliff_Maze,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Common,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "CompleteDungeon,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "CursedStatue,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "DarkWoods,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Demo000_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: Treasure Chest Get Demo
.string "Demo000_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Treasure Chest Get Demo (inventory full)
.string "Demo001_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: Cooking Demo
.string "Demo002_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: Pick up Get Demo
.string "Demo003_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Shiekah Slate download demo
.string "Demo004_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Map Tower Demo
.string "Demo005_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Warp demo
.string "Demo005_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Warp out demo
.string "Demo006_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Game Over Demo
.string "Demo007_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: Sleeping in bed demo
.string "Demo007_1,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: Passing time at a campfire demo
.string "Demo007_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Sleeping in an inn (subflow) demo
.string "Demo008_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: CDungeon entrance demo 1: door opening
.string "Demo008_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: CDungeon entrance demo 2: elevator going down into shrine
.string "Demo008_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: CDungeon entrance demo 3: arriving in dungeon
.string "Demo008_3,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: CDungeon entrance demo 4: elevator going up back to main world
.string "Demo008_4,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: CDungeon entrance demo 5: going outside shrine
.string "Demo008_9,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: CDungeon entrance demo 1: door opening (for emergent lithograph)
.string "Demo010_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Using the scope for the first time
.string "Demo010_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Exiting the scope for the first time
.string "Demo011_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Blood Moon
.string "Demo012_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Horse equipment change demo
.string "Demo013_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Cucco angry demo
.string "Demo014_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Yiga assassin calling others demo
.string "Demo016_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Dungeon appearing demo on pedestal activation
.string "Demo017_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Hidden Korok found Demo
.string "Demo018_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Rescue Demo
.string "Demo019_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Retry Demo
.string "Demo020_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Taking off from flight range demo
.string "Demo021_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Arriving in sky demo
.string "Demo022_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Demonstration at Teva with Do | skydiving at the divine beast
.string "Demo023_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Assassin changing Demo
.string "Demo024_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Door opening demo
.string "Demo024_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Door opening demo (Do not stop chemicals)
.string "Demo024_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Door opening demo (do not stop actors and chemicals)
.string "Demo025_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: General scene start demo
.string "Demo025_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: E3 only scene exit demo
.string "Demo025_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Title Scene Start Demo
.string "Demo026_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Combat shrine: enemy appearing demo
.string "Demo027_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Old Man Appears Demo (Common)
.string "Demo027_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Old Man Camera Demo (under the tower)
.string "Demo028_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Great Fairy: Unlock Fountain Demo
.string "Demo029_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Great Fairy: Upgrade Armor Demo
.string "Demo030_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Malanya: Horse Resurrection Demo
.string "Demo032_0,FP_ON,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Sheikah Slate repair demo
.string "Demo033_0,FP_ON,PAN_OFF,HND_ON,CTRL_OFF,PROMPT_ON" ; name: CDungeon Monk Demo
.string "Demo033_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: CDungeon Monk Demo (cleared)
.string "Demo034_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Goddess Statue exchanging demo
.string "Demo035_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Horned Statue demo
.string "Demo037_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Lost Woods ZoneOut
.string "Demo038_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Lost Woods: For lost stray safety position
.string "Demo039_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Divine Beast: Map Download
.string "Demo040_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Enemy Base Chest Unlock
.string "Demo041_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Terminal access processing subflow
.string "Demo042_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Slate menu - First time open
.string "Demo042_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Slate menu - First time close
.string "Demo043_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Gyro terminal operation
.string "Demo044_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Relics terminal operation
.string "Demo045_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Great Fairy: Initial Encounter
.string "Demo046_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Dyeing Demo
.string "Demo047_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Marlon flowering demo
.string "Demo048_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Respawn demo
.string "Demo048_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Respawn demo (for Vah Ruta battle)
.string "Demo049_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Controls guide demo
.string "Demo050_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: After Divine Beast clear - Zelda's voice
.string "Demo051_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: After Divine Beast cry - Zelda's voice
.string "Demo200_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: Monk guidance ~ CASE 1
.string "Demo201_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: Monk guidance ~ CASE 2
.string "Demo202_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Placing the Master Sword to enter the Trials
.string "Demo202_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Trial of the Sword - Entering Room 1 (Beginning) | seems to just be a lot of panning shots that don't really highlight anything: https://www.youtube.com/watch?v=fNRVxCWbM8E
.string "Demo202_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Trial of the Sword - Entering Room 1 (Middle)
.string "Demo202_3,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Trial of the Sword - Entering Room 1 (Final)
.string "Demo203_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Appearance of sword trial movement point appearance (now room)
.string "Demo204_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Trial of the Sword - transfer between rooms
.string "Demo205_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Return from Trial of the Sword to Korok Forest
.string "Demo206_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Drunken cave - escape from the last room (except the first time in the extreme)
.string "Demo207_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Trial of the Sword - Break the barrier of the last room guardian | Link talks to the monks at the end of the master trials before they disappear https://youtu.be/fNRVxCWbM8E?t=3397
.string "Demo208_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Trial of the Sword - escape from the last room (first time in final trial)
.string "Demo209_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Korok Forest ~ Explanation by the Great Deku Tree | cinematic that explains how the master sword trial will work: https://youtu.be/-FK13qE4Dkc?t=25
.string "Demo210_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Korok Forest ~ Praise by the Great Deku Tree | cutscene you get after beating the master trials
.string "Demo211_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Trial of the Sword - Arriving in last room | link speaks to monks at the end of the master trial https://youtu.be/fNRVxCWbM8E?t=3397
.string "Demo299_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: "Hero's Path" Demo
.string "Demo300_0,FP_ON,PAN_OFF,HND_ON,CTRL_OFF,PROMPT_ON" ; name: Master Sword Pulling demo
.string "Demo305_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: [Rito kids] Five sisters fly away | rito kids discover the just unlocked shrine, and fly away: https://youtu.be/LeM4bfJ3Ecg?t=633
.string "Demo309_0,FP_ON,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Impa shows the 13th memory picture | https://youtu.be/Za2yRAMcGCc?t=1027
.string "Demo309_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Zelda's voice after seeing the last memory
.string "Demo309_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Memory demo | Memories seem to start with a little panning: https://youtu.be/ROcgrugcbOk?t=1141
.string "Demo318_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: Zelda's voice after leaving the Shrine of Resurrection
.string "Demo327_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: [Goron's Rock Carrying] Demonstration of cooking roast rock and son's eating and destroying the rock | There's like a whole cooking scene with multiple cuts. Probably not fit for VR: https://youtu.be/vSbd99LkE9o?t=508
.string "Demo330_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: [Snaarathi Race] Demo at the start of the race
.string "Demo330_1,FP_ON,PAN_OFF,HND_OFF,CTRL_ON,PROMPT_ON" ; name: [Snaarathi Race] Demo when I gave a new record
.string "Demo331_0,FP_OFF,PAN_OFF,HND_OFF,CTRL_OFF" ; name: [Do not step on flowers] Flower lady getting angry | https://www.youtube.com/watch?v=Tx0sx6M-nLI
.string "Demo333_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Stationary Guardian activation demo | Zoom-ins on stationary guardians on the great plateau https://www.youtube.com/watch?v=FeGfQ5zN5ZM  By default, this makes the camera go absolutely crazy, often spinning rapidly and flying up into the sky. Both in first and third person mode.
.string "Demo344_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON,PROMPT_ON" ; name: Vah Ruta before scream demo | TODO: Could check out whether calling the demo of the beast rising from the ground switches the demo id that the mod receives, or whether it's separate.  You talk to the king of the Zora domain, and a bunch of advisors. Link stays put in one place. It calls another Demo that shows the elephant rising from the water and roaring: https://www.youtube.com/watch?v=nQkO5UeoJm0
.string "Demo344_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Vah Ruta last attack demo
.string "Demo344_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: [Vah Ruta] Sidon joins
.string "Demo344_4,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: [Vah Ruta] Thanks
.string "Demo346_0,FP_ON,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Destroying rocks with bombs
.string "Demo346_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Yunobo heads to Death Mountain
.string "Demo346_3,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Bridge falls to enter Death Mountain
.string "Demo346_4,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Sentries come out from Vah Rudania
.string "Demo346_5,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Detected by sentry demo | detection sentries when you try to run up the mountain to the divine beast in the mountain area https://youtu.be/UH2mAfvC2HI?t=8955
.string "Demo346_6,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Cannon hitting Vah Rudania demo
.string "Demo346_9,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Vah Rudania before scream demo
.string "Demo347_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Assassin Boss · confrontation
.string "Demo347_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Assassin Boss · Kill
.string "Demo347_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: [Vah Naboris Challenge] Generating barrier with Thunder Helm demo
.string "Demo347_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: [Vah Naboris Challenge] Last attack on Vah Naboris demo | https://youtu.be/UH2mAfvC2HI?t=11649
.string "Demo347_4,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Introducing the gods of thunder The demonstrating animals before demo
.string "Demo347_5,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: First meeting with Riju
.string "Demo347_6,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: The Vah Naboris rouge suffers a thunderous helmet
.string "Demo348_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Demonstration of breaking the last battery of Vah Medohs
.string "Demo348_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Before Vah Medoh's cry
.string "Demo363_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: The guide of Zelda
.string "Demo370_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Wedding demo
.string "Demo378_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: [Sneak | Sneaking into the voe-forbidden area
.string "Demo378_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: [Sneak | Link gets the Voe-cloak clothing at the bazaar
.string "Demo380_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: First meeting with Impa
.string "Demo401_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: [Go to Zora Village] Encounter with Prince Sidon Demo
.string "Demo401_1,FP_ON,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: [Go to Zora Village] Prince Sidon's support
.string "Demo480_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Silver piece (freeing dragon from Malice demo)
.string "Demo480_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Dragon first encounter demo | meeting the mallice dragon in the ice region. Moves the player slightly forwards at the start.
.string "Demo480_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Casting dragon material into fountain demo
.string "Demo480_3,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Ice Dragon First Fall Demo
.string "Demo600_0,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; name: Zelda's voice (sky) CASE 1
.string "Demo600_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Zelda's voice (sky) CASE 2
.string "Demo601_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Challenge explanation ~ One-Hit Obliterator appears
.string "Demo601_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: One-Hit Obliterator Get
.string "Demo602_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: CDungeon Appearance | Used for when the master trial DLC dungeons appear
.string "Demo602_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: CDungeon Appearance (Yiga?)
.string "Demo603_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Shrine entry prohibition
.string "Demo604_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Clear One-Hit Obliterator shrine
.string "Demo605_0,FP_OFF,PAN_ON,HND_OFF,CTRL_ON" ; name: Interruption of One-Hit Obliterator challenge
.string "Demo606_0,FP_OFF,PAN_ON,HND_OFF,CTRL_ON" ; name: Restart of One-Hit Obliterator challenge
.string "Demo607_0,FP_OFF,PAN_ON,HND_OFF,CTRL_ON,PROMPT_ON" ; name: Kass's Songs - Swordsman's Song
.string "Demo609_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Clear One-Hit Obliterator challenge
.string "Demo610_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Kass's Songs - Champion's Song (Goron City)
.string "Demo610_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Kass's Songs - Champion's Song (Rito Village)
.string "Demo610_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Kass's Songs - Champion's Song (Zora Village)
.string "Demo610_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Kass's Songs - Champion's Song (city of Geld)
.string "Demo611_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Champion shrine clear
.string "Demo612_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Kass's Songs ~ Daruk's Song
.string "Demo612_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Kass's Songs ~ Revali's Song
.string "Demo612_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Kass's Songs ~ Mipha's Song
.string "Demo612_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Kass's Songs ~ Urbosa's Song
.string "Demo614_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Enter Rudania
.string "Demo614_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Enter Medoh
.string "Demo614_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Enter Ruta
.string "Demo614_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Enter Naboris
.string "Demo616_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Escape from the divine beast
.string "Demo617_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Return to the outside of the beast
.string "Demo620_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Fireblight Ganon R - Second half of the battle
.string "Demo620_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Windblight Ganon R - Second half of the battle
.string "Demo620_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Waterblight Ganon R - Second half of the battle
.string "Demo620_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Thunderblight Ganon R - Second half of the battle
.string "Demo622_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Returning outside of the beast - Daruk's voice
.string "Demo622_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Returning outside of the beast - Revali's voice
.string "Demo622_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Returning outside of the beast - Mipha's voice
.string "Demo622_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Returning outside of the beast - Urbosa's voice
.string "Demo623_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Monk guidance (back to the Shrine of Resurrection)
.string "Demo640_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Entering the Final Trial
.string "Demo641_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Entering the Final Trial from the Shrine of Resurrection
.string "Demo641_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Arriving in the Final Trial
.string "Demo642_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Escaping from the Final Trial
.string "Demo642_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Returning from the Final Trial to the Shrine of Resurrection
.string "Demo643_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Final Trial explanation
.string "Demo643_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Examine control terminal
.string "Demo643_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Examine the deepest door
.string "Demo644_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Final Trial boss: to the second form
.string "Demo644_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Final Trial boss: to the third form
.string "Demo645_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Final Trial Clear
.string "Demo646_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Kass's Songs - A song to glorify the champions
.string "Demo647_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Decorate photos
.string "Demo648_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Final Trial boss appears
.string "Demo649_0,FP_ON,PAN_ON,HND_OFF,CTRL_ON" ; name: Motorcycle fuel filling demo
.string "Demo650_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Gyro terminal operation for DLC dungeon
.string "Demo650_1,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; name: Door open demo (AoC 2)
.string "Demo650_2,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; name: Door open demo (AoC 2) (actors not stopped)
.string "Demo700_0,FP_ON,PAN_OFF,HND_ON,CTRL_OFF,PROMPT_ON" ; name: Sheikah Slate get demo
.string "Demo701_0,FP_ON,PAN_OFF,HND_ON,CTRL_OFF,PROMPT_ON" ; name: Open the second door that leads to the ground | should we prevent hands from all cutscenes or just these?  Camera flips back to look at Link, but maybe we don't want that.  Note: Nah we don't want that - mako  Alright, tested it without panning and it looks great! -Tim
.string "Demo702_0,FP_ON,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Teba flies to Flight Range with Tulin
.string "Demo704_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Vah Medoh - Explanation by the champion on first time enter
.string "Demo705_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Vah Rudania - Explanation by the champion on first time enter
.string "Demo706_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Vah Naboris - Explanation by the champion on first time enter
.string "Demo707_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Inventory increase by Hestu | Dance animation from hestu
.string "Demo708_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Vah Ruta - Explanation by the champion on first time enter
.string "Demo709_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; name: Vah Naboris - Attack point demo | I think this is cooler in 3rd person
.string "Demo710_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Leaving the plateau for the first time
.string "Demo712_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Vah Medoh - Transition to second half of Blight Ganon battle
.string "Demo712_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Vah Rudania - Transition to second half of Blight Ganon battle
.string "Demo712_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Vah Naboris - Transition to second half of Blight Ganon battle
.string "Demo712_3,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Vah Ruta - Transition to second half of Blight Ganon battle
.string "Demo717_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Ganon (incomplete form) - Transition to final stage | Link falls through the floor together with ganon. Maybe really cool, but extremely scare?
.string "Demo718_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Ganon (incomplete form) - Guardian launch
.string "Demo719_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Tutorial dungeon
.string "Demo720_0,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; name: Ganon (incomplete form) - Setup
.string "Demo721_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: Vah Naboris's Attack demo | Unsure what this is?
.string "Demo799_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; name: Field
.string "Demo985_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; name: 2016_E3 Thankyou screen demo
.string "Desert_Maze,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "DgnObj_DLC_Weapon_Sword_502,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "DgnObj_HintBoard,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "DisplayShop,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "DLC_TwnObj_HatenoLinkHouse_Check_Photo,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "DokuroEye,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "DontDamageFlower,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; https://www.youtube.com/watch?v=lT3MW6eCHL4
.string "Drag_Hero,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; NPC standing outside of Gerudo town giving you hints about where to find the cross-dresser
.string "Dummy,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Electric_Relic,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; Contains a lot of shared things and dialogue conversations of the Gerudo quest
.string "FairyFountain,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; small quest-ish dialogue from an artist telling you about a fairy
.string "FindDungeon,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; King Rhoam shows you the two peaks, and gives you the paraglider
.string "Find_4Relic,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Find_Impa,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; Guards and Paya dialogue before finding Impa
.string "Fire_Relic,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Kinda 50/50. Its pretty cool I think to fire shots in first-person, but I'm unsure whether its a good method to test things.
.string "FironeMini_GiantHorse,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Talking to some quest givers that point towards where you can find this giant horse
.string "FironeMini_HeartPond,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "FironeMini_HorseEnemy,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; You're getting asked to get rid of some bokoblins and save a horse, I think
.string "FironeMini_TerribleThunder,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "FirstOhenro,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; Quest where you help a brave korok
.string "FirstTower,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Cutscene file is just missing?
.string "FldObj_DLC_BlowSwordBase_A,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "FldObj_GerudoTownBook,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; reading cookbooks in gerudo town
.string "FldObj_GerudoTownBook2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; reading cookbooks in gerudo town 2
.string "FldObj_HatenoGateBook,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "FldObj_RockZoraRelief_A_M_01_ma,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "FourJewel,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; Four orbs quest at the thundra plateau: https://www.youtube.com/watch?v=E6XTGHQ35-s
.string "Gaman,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; https://www.youtube.com/watch?v=t2IhifpPE4w
.string "GanonQuest,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "GerudoMiniJewel,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; Dialogue with jewelry shop in gerudo town
.string "Gerudo_CarryIce,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Gerudo_CarryIce_mini,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Gerudo_Ch_FindingValetta,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; Very dialogue heavy quest, but its just talking to characters.
.string "Gerudo_Ch_Helmet,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; https://www.youtube.com/watch?v=WXlf12vosMc
.string "Gerudo_Ch_Poison,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Gerudo_Ch_SandWarm,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Gerudo_Ch_SecretClub,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Gerudo_Ch_SnowBoots,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Gerudo_Ch_SnowMT,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Gerudo_HorseBuyer,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "Gerudo_tsukamidake,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "GetDemo,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Get_MasterSword,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Giant_ZoraMini,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "GodTree,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "GoronCamp,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "GoronCamp_mini,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "GoronCityMini_BeatGolem,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "GoronMini_ImportGem,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "GoronMini_WallCrackTBox,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "GoronsRock,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "GotoZoraVillage,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "HateeluMini_Treasure,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HatenoMini_BlueFire,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "HatenoMini_CameraBoy,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HatenoMini_DevilSeal,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HatenoMini_GoatThief,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HatenoMini_LoveInsects,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HatenoMini_MyHome,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HatenoMini_ThreeTree,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HatenoMini_WeaponMania,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HatenoMini_WhiteDragon,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Hateno_SheikPad_PowerUp,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HigakkareMini_RedDragon,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HigakkareMini_StrangeMan,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HorseAssociationCustum001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HorseAssociationCustum002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HorseAssociationCustum003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HorseAssociationCustum004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "HorseGod001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "HutagoHatago_Ch_001,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "HyruleCastle,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "HyruleDepthMini_WhiteHorse,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "HyrulePlainMini_Balloon,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "InitTalk,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "isso_treasure_mini,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Kakariko_Cha_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Kakariko_Cha_003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Kakariko_Ch_004,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Kakariko_Ch_005,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Kakariko_Ch_006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Kakariko_Ch_Cooking2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Kakariko_Ch_Cooking3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "Kakariko_Ch_Cooking4,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "KnightDoll,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "KorokMini_KorokShiren,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "KorokMini_RiddleShiren,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "KorokMini_RodShiren,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "KorokMini_UMAShiren,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "MacuseIseki,FP_OFF,PAN_ON,HND_OFF,CTRL_ON"
.string "MagneticFld,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Gerudo at east gate points you to the statues with the swords.
.string "MarittaMini_BigWhales,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "MinakkareMini_Dragonfly,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "MinamihateeluMini_touzoku,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "MiniGame100enemy,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "MiniGame_Bowling,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "MiniGame_Crosscountry,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "MiniGame_GambleTreasureBox,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "MiniGame_HillTower_BirdMan,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "MiniGame_HorsebackArchery,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "MiniGame_HorseRace,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "MiniGame_KitakkareBF,FP_ON,PAN_ON,HND_OFF,CTRL_ON"
.string "MiniGame_ParasailArchery,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "MiniGame_ParasailRide,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "MiniGame_ShieldSurfing,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "MiniGame_SmashGolf,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "MiniGame_TimeLimitHunting,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "MouthofDragon,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Kass in the Pagos Woods
.string "MtMotelMini_Landscape,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "My_Hero,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NakedIsland,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_AncientAssistant001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_AncientAssistant003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_AncientAssistant004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_AncientDoctor,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Purah after u upgrade the slate to the max.
.string "Npc_AncientDoctor_Hateno,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_000,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_008,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "NPC_artist_009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_artist_011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Assassin_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Assassin_002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Assassin_003,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Assassin_004,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Assassin_005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Assassin_006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Assassin_007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Assassin_008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Assassin_009,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Assassin_010,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Attacked_012,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Bottle_Mes001,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "Npc_Bowling,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Challenge093Lady,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_DeathEastHatago_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_DeathEastHatago_002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_DeathEastHatago_003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_DeathMt001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_DeathWestHatago_002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_DressFairy_00,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_DressFairy_01,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_DressFairy_02,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_DressFairy_03,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_DungeonPriest001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_FaronHatago_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_FaronWoods001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_FaronWoods003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_FaronWoods006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; Brigo on the bridge. The NPC points towards the dueling peaks so the additional camera pan is not really required. And immersion is more important, here.
.string "Npc_FaronWoods007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_FaronWoods008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_FaronWoods009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_FaronWoods010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_FirstColony004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_FollowGoron,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Gaman01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Gaman02,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Gaman03,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_GerudoCanyonHatago001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_GerudoDesert001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_GerudoDesert002,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "Npc_GerudoDesert003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_GerudoDesert004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_gerudovalley001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_goron001,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Tray Goron city inn
.string "Npc_goron002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_goron003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_goron004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron006,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Goron009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron012,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron013,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron014,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Goron015,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron016,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron017,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron018,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron020,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron020_2,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Goron021,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron022,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron023,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Goron025,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_goron026,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Goron030,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Goron031,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron032,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron033,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron109,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron110,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron_Camp001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron_Camp002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Goron_Camp003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoGate001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoGate002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoGate003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage012,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage013,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage014,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage015,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage016,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage017,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage018,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage019,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage020,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage021,FP_ON,PAN_ON,HND_OFF,CTRL_ON"
.string "Npc_HatenoVillage022,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage023,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage024,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage025,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage026,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage027,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "Npc_HatenoVillage028,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage029,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage030,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage031,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Hudson
.string "Npc_HatenoVillage032,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HatenoVillage033,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HiddenKorokFly,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HiddenKorokGround,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain001,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Tulin of Rito Village
.string "Npc_HighMountain003,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain004,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain010,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain012,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain013,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain020,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain021,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain025,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain030,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain031,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain032,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain033,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain034,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain035,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain036,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain037,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain040,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HighMountain041,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HorseBuyer,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Hunters_Lost,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HutagoHatago_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HutagoHatago_002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HutagoHatago_Woman_01,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "Npc_HyruleDepthHatago001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleDepthHatago002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleDepthHatago003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleDepthHatago004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleDepthHatago005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleDepthHatago006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleDepthHatago007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyrulePlain004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyrulePlain005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyrulePlain006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleWestHatago001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleWestHatago002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleWestHatago003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleWestHatago004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleWestHatago005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_HyruleWestHatago006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_IceVillage011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko001,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Impa
.string "Npc_Kakariko002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko006,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "Npc_Kakariko007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko008,FP_ON,PAN_ON,HND_OFF,CTRL_ON"
.string "Npc_Kakariko010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko012,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko013,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "Npc_Kakariko014,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko015,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kakariko016,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_King_Common,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_King_Parasail002,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_King_Parasail004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_King_Parasail005,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_King_Vagrant001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_King_Vagrant003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_King_Vagrant004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_King_Vagrant005,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_King_Vagrant006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_kokiri001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_kokiri002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_kokiri003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_kokiri004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_kokiri005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_kokiri006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kokiri007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kokiri008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_kokiri009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kokiri010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Kokiri011,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_kokiri012,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_LakeSideHatago004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_LakeSideHatago005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_LakeSideHatago006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_Lanayru001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_Lanayru002,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Lanayru003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Lanayru004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_MamonoShop,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Minigame_BirdMan001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_MiniGame_Golf,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_006,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Musician_007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_012,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_014,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_015,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_AoC_BalladOfHeroes,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_AoC_BalladOfHeroes_Last,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_AoC_HeroGerudo,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_AoC_HeroGerudoRelief,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_AoC_HeroGoron,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_AoC_HeroGoronRelief,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_AoC_HeroRito,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_AoC_HeroRitoRelief,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_AoC_HeroZora,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Musician_AoC_HeroZoraRelief,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_NorthHatelHatago001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_NorthHatelHatago002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_NorthHatelHatago006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_NorthHateru003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_NorthHateru005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_NorthHateru_around001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis003,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "NPC_oasis004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_oasis004_fr01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_oasis004_fr02,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "NPC_oasis004_fr03,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "NPC_oasis004_fr04,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis009,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_oasis010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis012,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis013,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis014,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis015,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis016,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis017,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis018,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis019,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis020,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis021,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis022,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis023,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis024,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis025,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis026,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis027,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis028,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis029,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis032,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis036,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis037,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis038,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis039,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis040,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis041,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis042,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis043,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis044,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis045,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis046,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis047,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_oasis050,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_oasis050_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisGoddess,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisGoron_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisHighMountain_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisHylia_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisHylia_002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisHylia_003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisMilk_A,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisMilk_B,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisMilk_C,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisMilk_D,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisMilk_E,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisSoldier_A,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisSoldier_B,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisSoldier_C,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisStep_A,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisStep_B,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisStudent_A,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisStudent_B,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisStudent_C,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisTeacher,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OasisWaterChannel,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OldKorok,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OldKorok_Forest,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OldKorok_Help,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_OldKorok_Rvs,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_OldKorok_WetLand,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Remains_Fancier001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_RitoHatago001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_RitoHatago002,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Ariane Rito Stable
.string "Npc_RitoHatago003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_RitoHatago004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_RiverSideHatago001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_RiverSideHatago002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_RiverSideHatago003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_012,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_013,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_014,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_015,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_016,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_017,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_018,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_019,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_020,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_021,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_022,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_023,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_024,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_025,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_026,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_027,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_028,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_029,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_030,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_031,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_032,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_033,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_034,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_035,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_036,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_037,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_040,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_041,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_042,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_043,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_044,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_045,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_Common,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Road_Firone002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SmallOasis012,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage004,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "Npc_SouthernVillage005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage009,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage012,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage013,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage014,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage015,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthernVillage016,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthHateru005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthHateru006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_SouthHateru007,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Hino
.string "Npc_Strange_Beacon,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "Npc_TabantaBridgeHatago001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TabantaBridgeHatago002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TabantaBridgeHatago003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TabantaBridgeHatago004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TabantaBridgeHatago005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TabantaBridgeHatago006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TabantaHatago001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TabantaHatago002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TabantaHatago003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TabantaHatago004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TamourHatago001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TamourHatago002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TamourHatago003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TamourHatago004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TamourHatago005,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TamourHatago2_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TamourPlainHatago_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TamourPlainHatago_002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TamourPlainHatago_003,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_TamourPlainHatago_004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_00,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_02,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_03,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_04,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_05,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_06,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_07,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_08,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_09,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_10,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_11,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_12,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_13,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_14,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TripMaster_15,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_TutorialDungeon,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_UMiiVillage003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_UMiiVillage007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_UMiiVillage008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_UMiiVillage009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_UMiiVillage010,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_UMiiVillage030,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_UMiiVillage031,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_UMiiVillage031_2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_UMiiVillage032,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Bolson
.string "Npc_ValleyVillage010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_ValleyVillage031,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora002,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora003,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora004,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora005,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Zora006,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora007,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora008,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora009,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora010,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora011,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora012,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_Zora013,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora014,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "Npc_Zora015,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora020,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora025,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora026,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora027,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora028,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora029,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora031,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora032,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora033,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora034,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora035,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora036,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora037,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_Zora126,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Npc_ZoraB001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Npc_ZoraGoron_001,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Oasis_Drug_Challenge,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryGerudo_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryHateno003_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryHateno029_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryIceVillage011_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryKing_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryMusician_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryRito001_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryTamul_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryTamul_A_02,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryTamul_A_03,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryTamul_A_04,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DiaryTamul_A_05,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_Diary_touzoku_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_Diary_touzoku_02,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Obj_DRStone_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; sheikah slate require
.string "OldKorok_Help,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "OneHundred,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "OperationGuide,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "PictureMemory,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "Relief_Landing,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "Remains_Fancier,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Loone is crazy about relics
.string "RememberTag,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "RinelSearch,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "RitoMini_Cook,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "RitoMini_Flint,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "RitoMini_IceGolem,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "RitoRabitMountain,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "RitoSongMystery,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; After completing the puzzle, a shrine appears not too far away. Probably cool in 1st person.
.string "RitoUmayadoMini_HotRecipe,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Rito_BrosRock,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Rito_KeelSearch,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; shrine quest find kheel
.string "RiversideMini_CastleWeapon,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "RiversideMini_RoyalRecipe,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "RockyStrike,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "SandStorm,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "SanrokuMini_Lizard,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "SDemo_A-7,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "SDemo_B-6,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "SDemo_B-7,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "SDemo_D-6,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; Zelda telling Link to follow the marker on his map to the first, buried, tower.
.string "SDemo_F-2,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "SearchStone,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "SecretofObject,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; shrine quest
.string "SeekerEye,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "SetugenUmayadoMini_Umahonephoto,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Shadow_Sign,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; shrine quest
.string "ShieldofKolog,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "ShieldofKolog_mini,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "SignalFlowchart,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "SignboardStone,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "SignboardWood,FP_ON,PAN_OFF,HND_OFF,CTRL_ON" ; When Link reads signs
.string "SignboardWood_Mamono,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "SnowBall,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Open door with snowball
.string "Solitary_Maze,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "StatueofZora,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Anyone you talk to thats near the Mipha statue.
.string "StolenBook,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "Strange_Beacon_CookSet,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "subchallnpc000,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "subchallnpc000_twin,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "SunazarashiRace,FP_ON,PAN_ON,HND_OFF,CTRL_ON"
.string "SunazarashiRace_mini,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TabantaBridgeMini_Sundial,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Test_Npc_UMii_Hylia_M_Normal_Middle,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Threebros_giant,FP_ON,PAN_ON,HND_OFF,CTRL_ON"
.string "Thunder_Sword,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "TipsCommon,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TipsDungeon,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TipsField,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TipsGameOver,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TipsItem,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TipsNotify,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TipsPlayer,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TreasureHunt01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TreasureHunt02,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TreasureHunt03,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TreasureHunt04,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TreasureHunt_touzoku01,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "TreasureHunt_touzoku02,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TreasureHunt_touzoku03,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TreasureHunt_touzoku04,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TreasureHunt_touzoku05,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TreasureHunt_touzoku06,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TreasureHunt_touzoku07,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "TreasureHunt_touzoku08,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TutorialDungeon,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObjVillage_ZoraSignboard,FP_ON,PAN_OFF,HND_OFF,CTRL_ON"
.string "TwnObj_AncientOven_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_City_GerudoThunderHelmet_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_DemonStatue_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_GerudoTown_RougeDiary_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_GoddesStatue_A_01,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "TwnObj_GoddesStatue_A_02,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "TwnObj_GoddesStatue_A_10,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "TwnObj_HyruleCastleBook_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_HyruleCastleBook_A_02,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_HyruleCastleBook_A_03,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_HyruleCastleBook_A_04,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_KarakaraBook_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_SuperGoddesStatue_A_01,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoBook_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoBook_02,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoBook_03,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoBook_04,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoBook_05,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoBook_06,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoBook_07,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoBook_08,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoBook_09,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoHeroBook_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoHeroBook_02,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoHeroBook_03,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_UmayadoHeroBook_04,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_Village_Korok_DekuTree_A_01,FP_ON,PAN_ON,HND_OFF,CTRL_OFF"
.string "TwnObj_Village_SheikerBook_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_Village_SheikerHeirDiary_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_Village_SheikerHeirDiary_A_02,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwnObj_ZoraLithograph_A_01,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "TwoWheels,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; this should probably stay 3rd person
.string "UMiiMini_GiveCake,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "UMiiMini_MakeVillage,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "UMiiMini_RichmansHobby,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "UotoriMini_RecipeSea,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "UotoriMini_RecoverBay,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "UotoriMini_SinkTreasure,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Water_Relic,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF" ; Sidon and Muzu talk in front of Miphas statue.
.string "Wind_Relic,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; Saki and Keneli of Rito village both use this call
.string "XXX,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Yorozuya_Kaiwa,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "ZoraMini_DiveChallenge,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF"
.string "ZoraMini_FlowedWife,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "ZoraMini_HarvestingStone,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "ZoraMini_ReliefSearch,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Zora_FlogMini,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF"
.string "Book,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "BrothersGiant,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Challenge_Learning_Konishi,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Challenge_Learning_Konishi2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Gerudo_G_N,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Gerudo_G_P,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Gerudo_N_D,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Gerudo_N_N,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Gerudo_N_P,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Gerudo_S_D,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Gerudo_S_P,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Hylia_W_D,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Hylia_W_N,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Hylia_W_P,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Rito_G_N,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Rito_G_P,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Rito_M_D,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Rito_M_N,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Rito_M_P,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Rito_W_D,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Rito_W_N,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Rito_W_P,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Zora_B_N,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Zora_B_P,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Zora_M_D,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Zora_M_N,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Zora_M_P,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Zora_W_D,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Zora_W_N,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "CurveSpeed_Zora_W_P,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo100_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo101_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo101_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo101_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo101_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo101_4,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo101_5,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo101_6,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo101_7,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo101_8,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo101_9,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo102_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo103_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo104_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo104_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo105_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo105_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo109_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo109_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo109_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo110_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo111_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo112_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo113_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo114_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo115_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo116_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo117_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo118_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo119_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo120_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo121_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo122_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo123_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo124_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo125_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo126_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo127_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo128_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo129_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo130_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo131_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo132_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo133_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo134_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo135_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo136_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo137_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo138_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo139_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo140_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo141_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo141_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo141_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo141_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo141_4,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo142_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo143_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo143_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo143_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo143_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo143_4,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo145_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo146_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo146_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo147_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo148_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo149_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo150_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo151_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo152_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo153_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo154_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo154_1,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo154_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo155_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo156_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom | Revalis gift scene
.string "Demo158_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo159_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom | Urbosas gift
.string "Demo160_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo161_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo162_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo163_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo164_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo165_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo165_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo165_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo165_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo165_4,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo165_5,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo166_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo167_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo169_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo173_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo173_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo173_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo173_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo173_4,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo173_5,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo173_6,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo173_7,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo173_8,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo173_9,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo174_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo175_0,FP_ON,PAN_OFF,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo176_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo177_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo177_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo800_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo801_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo802_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo803_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo804_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo805_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo806_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo807_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo808_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo809_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo809_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo810_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo811_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo812_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo813_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo814_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo815_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo816_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo817_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo818_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo819_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo820_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo821_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo822_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo823_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo824_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo825_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo826_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo827_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo828_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo829_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo830_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo831_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo832_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo865_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo866_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo867_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo868_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo869_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo870_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo871_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo872_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo873_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo874_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo875_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo876_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo877_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo878_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo879_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo880_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo881_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo882_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo883_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo884_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo885_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo886_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo887_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo888_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo889_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo890_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo891_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo892_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo893_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo894_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo895_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo896_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo897_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo898_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo899_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo900_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo940_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo941_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo942_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo943_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo944_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo945_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo946_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo947_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo948_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo949_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo950_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo970_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo981_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo982_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo982_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo982_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo982_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo982_4,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo982_5,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo983_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo983_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo983_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo983_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo983_4,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo983_5,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo983_6,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo983_7,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo983_8,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo983_9,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo984_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo984_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo984_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo984_4,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo984_6,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo984_7,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo984_8,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo984_9,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo986_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo987_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo988_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo988_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo989_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo989_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo990_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo990_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo991_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo992_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo992_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo992_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo992_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo994_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo994_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo994_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo994_3,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo994_4,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo994_5,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo994_6,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo994_7,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo994_8,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo994_9,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo995_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF,PROMPT_ON" ; custom
.string "Demo996_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo996_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo998_0,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo998_1,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Demo998_2,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Dm_Npc_Gerdo_Hero,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Dm_Npc_Goron_Hero,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Dm_Npc_Rito_Hero,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Dm_Npc_Zelda_Prince,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Dm_Npc_Zelda_Scientist,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Dm_Npc_Zelda_Sibyl,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Dm_Npc_Zora_Hero,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "FldObj_BurnMeat,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "FldObj_SignboardStone_A_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "GetHeroSoul,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "HamauraTestNpc00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Imatest_son,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "ImaTestNPC_T,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "kawazoe_FujitaLearning001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "kkawazoeTestNPC01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "MamonoShop_BigEnemy_Giant,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "MamonoShop_BigEnemy_Golem,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "MamonoShop_BigEnemy_Sandworm,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Monolith_NPC_Test_H5_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Monolith_NPC_Test_H5_02,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Monolith_NPC_Test_H5_03,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_EventTest_Oyaji,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_FaronWoods002,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_ForestVillage001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_HiddenKorok,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "npc_ImaTest003,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "npc_ImaTest004,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Musician_004,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Musician_013,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "NPC_nym01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "NPC_nym02,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_Konishi_B,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_Konishi001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_tamura_001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_tamura_002,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_tamura_003,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_tamura_004,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_tamura_005,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_tamura_007,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_tamura_008,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_tamura_009,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_Test_Tamura_010,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "NPC_testes01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "NPC_testes02_nakayama,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "NPC_testHatago_Mono_kwz,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "NPC_TestHiramuki_HyruleMan000,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "NPC_testNPC_Mono_kwz,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "NPC_testQuest_kwz,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Npc_TominagaTest003,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Obj_LetterSetFirst_A_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "PrezentationOkamura,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Rito_NPC01_Area,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "SignboardWood_02,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Teat_Npc_Zora_M_Warrior_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Nakayama00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_nakayama01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_AncientDoctor,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_G_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_G_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_Hero,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_W_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_W_Normal_Middle,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_W_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_W_Positive_Middle,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_W_Warrior_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_W_Warrior_Normal_Sword,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_W_Warrior_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Gerudo_Y_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Goron_B_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Goron_M_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Goron_M_Warrior,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Goron_X_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Goron006,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Goron020,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_HatenoVillage001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_HatenoVillage031,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_HatenoVillage032,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_HighMountain004,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_HighMountain005,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_B_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_B_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_G_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_G_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_M_Fat,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_M_Gay,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_M_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_M_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_M_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_M_Slim,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_W_Fat,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_W_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_W_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_W_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_W_Slim,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_X_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Hylia_Y_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Kakariko001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Kakariko002,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_kakariko005,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Kakariko007,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_King_Vagrant001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_MamonoShop,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_oasis001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_OldKorok,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Rito_B_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Rito_G_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Rito_G_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Rito_Hero,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Rito_M_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Rito_M_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Rito_Moto,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Rito_W_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Rito_W_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Road,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Scientist_M,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_TripMaster_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_B_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_B_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_B_05,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_B_06,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_B_07,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_B_09,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Goron_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Goron_B00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Goron_X00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Goron01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Goron09,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_02,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_03,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_04,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_05,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_06,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_07,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_08,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_09,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_M_Gatekeeper,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Rito_G_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Rito_M_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Rito_M_Gatekeeper,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Rito_M_Musician,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Rito_W_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Shiekah_M_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Shiekah_M_Artist,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_W_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_W_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_W_02,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_W_03,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_W_04,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_W_05,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_W_06,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_W_07,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_W_08,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_W_09,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_X_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_X_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_X_09,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Y_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Y_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Zora_B_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Zora_G_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Zora_G_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Zora_M_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Zora_W_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Common_Zora_X_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Gerudo_G_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Gerudo_G_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Gerudo_General,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Gerudo_M_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Gerudo_Queen,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Gerudo_W_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Gerudo_W_Soldier,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Gerudo_Y_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_B_Normal_Fat,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_B_Normal_Middle,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_B_Normal_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_B_Normal_Thin,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_B_Tall_Fat,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_B_Tall_Middle,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_B_Tall_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_B_Tall_Thin,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_M_Normal_Fat,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_M_Normal_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_M_Normal_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_M_Normal_Thin,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_M_Tall_Fat,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_M_Tall_Middle,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_M_Tall_Thin,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_W_Face_Fukuchi,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_W_Normal_Fat,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_W_Normal_Middle,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_W_Normal_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_W_Normal_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_W_Normal_Thin,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_W_Tall_Fat,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_W_Tall_Middle,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Hylia_W_Tall_Thin,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Korogu_M_00,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Korogu_M_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Korogu_M_02,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Korogu_M_03,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Korogu_M_04,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Korogu_M_05,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Korogu_M_06,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Korogu_M_07,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_UMii_Korogu_M_08,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_ValleyVillage010,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_B_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_B_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_G_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_G_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_Hero,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_M_Negative,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_M_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_M_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_M_Warrior_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_W_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_W_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_W_Warrior_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_W_Warrior_Positive,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_Zora_X_Normal,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_Npc_ZoraB001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Test_sima,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "TestDungeonNPC001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "TestObj_Stone_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "TestObj_Stone_02,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "TestObj_Stone_03,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "TestQuest_kwz001,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "TestQuest_shimizu01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "TominagaTest,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "TurnTestQuest,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "TwnObj_Village_HatenoSignboard_A_01,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Umii05,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Umii06,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Umii08,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Umii10,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "YmdQueTest,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "Z_test_npc02,FP_OFF,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "SeekDungeon_Npc_oasis032_Talk,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; custom
.string "SeekDungeon_Npc_oasis037_Talk,FP_ON,PAN_ON,HND_OFF,CTRL_OFF" ; custom


; End of table
.int 0, 0, 0, 0
