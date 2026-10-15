[BotW_GUIAspectRatio_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

const_AspectRatio:
.float ($gameWidth/$gameHeight)/($width/$height)

const_AspectRatioHalf:
.float (($gameWidth/$gameHeight)/($width/$height))/2

const_ReverseAspectRatio:
.float ($width/$height)/($gameWidth/$gameHeight)

const_ReverseAspectRatio2x:
.float (($width/$height)/($gameWidth/$gameHeight))*2

const_PaddingLeftSide:
.float (($gameWidth/$gameHeight)/($width/$height)) * ((($width/($height/720))/2) - 640) * (-1)

const_PaddingRightSide:
.float (($gameWidth/$gameHeight)/($width/$height)) * ((($width/($height/720))/2) - 640)

const_AltPaddingLeftSide:
.float (($gameWidth/$gameHeight)/($width/$height)) * (($width/2) - 640) * (-1)

const_AltPaddingRightSide:
.float (($gameWidth/$gameHeight)/($width/$height)) * (($width/2) - 640)


newLineFormatScreen:
.string "Loading screen %s...%c"
newLineFormatPanes:
.string "    Pane: %s...%c"
newLineCharacter:
.int 10

copyScreenString:
.string "PaAppDungeonBtnNextMoveFinalDLC_00" ; longest .blyt name
.align 4
copyScreenStringLen:
.int 34

copySubPanelString:
.string "PaAppDungeonBtnNextMoveFinalDLC_00" ; longest .blyt name
.align 4
copySubPanelStringLen:
.int 34

; free registers = r12, r10, r9, r11, r8 maybe
; r31 has the name of the pane at an offset at 0x80
_scalePaneGUI:
mflr r0

li r10, $ultrawideHUDMode
cmpwi r10, 0
beq exitScale

; Log currently loaded pane to register if logging is enabled
li r10, $enableUltrawideDebugLogging
cmpwi r10, 2
blt skipPaneLogging

mr r10, r3
mr r11, r4
mr r12, r5
crxor 4*cr1+eq, 4*cr1+eq, 4*cr1+eq
lis r3, newLineFormatPanes@ha
addi r3, r3, newLineFormatPanes@l
addi r4, r31, 0x80
lis r5, newLineCharacter@ha
lwz r5, newLineCharacter@l(r5)
bl import.coreinit.OSReport
mr r3, r10
mr r4, r11
mr r5, r12

skipPaneLogging:

; ------------------------------------------------------------------------------------------
; Store whether there's a subpanel name

addi r5, r31, 0x80

lbz r11, 0x0(r5)
cmpwi r11, 0x50
bne checkScreenNames
lbz r11, 0x1(r5)
cmpwi r11, 0x61
bne checkScreenNames

lis r11, copySubPanelStringLen@ha
lwz r11, copySubPanelStringLen@l(r11)
lis r12, copySubPanelString@ha
addi r12, r12, copySubPanelString@l

copySubPanelLoop:
lbzx r10, r5, r11
stbx r10, r12, r11
addi r11, r11, -1
cmpwi r11, -1
bne copySubPanelLoop


; ------------------------------------------------------------------------------------------
; Check which screen is getting loaded by using the copied screen name

checkScreenNames:
lis r5, copyScreenString@ha
addi r5, r5, copyScreenString@l

lis r10, scr_BootUp_00@ha
addi r10, r10, scr_BootUp_00@l
bla _compareString
beq scaleInOutToCenter

lis r10, scr_ChangeController_00@ha
addi r10, r10, scr_ChangeController_00@l
bla _compareString
beq scaleInOutScreenToRightSide

lis r10, scr_ChangeControllerDRC_00@ha
addi r10, r10, scr_ChangeControllerDRC_00@l
bla _compareString
beq scaleInOutScreenToRightSide

lis r10, scr_GamePadBG_00@ha ; Can't be scaled properly, even when scaling individually
addi r10, r10, scr_GamePadBG_00@l
bla _compareString
beq scaleRootToCenter

lis r10, scr_Title_00@ha
addi r10, r10, scr_Title_00@l
bla _compareString
beq scaleTitlePanes

lis r10, scr_SystemWindow_01@ha
addi r10, r10, scr_SystemWindow_01@l
bla _compareString
beq scaleSystemWindowPanes

lis r10, scr_AppSystemWindow_00@ha
addi r10, r10, scr_AppSystemWindow_00@l
bla _compareString
beq scaleAppSystemWindowPanes

lis r10, scr_AppSystemWindowNoBtn_00@ha
addi r10, r10, scr_AppSystemWindowNoBtn_00@l
bla _compareString
beq scaleInOutScreenToRightSide

lis r10, scr_OptionWindow_00@ha
addi r10, r10, scr_OptionWindow_00@l
bla _compareString
beq scaleOptionsMenuPanes

lis r10, scr_AmiiboWindow_00@ha
addi r10, r10, scr_AmiiboWindow_00@l
bla _compareString
beq scaleInOutToCenter

lis r10, scr_Fade@ha
addi r10, r10, scr_Fade@l
bla _compareString
beq scaleLoadingPanes

lis r10, scr_GameOver_00@ha
addi r10, r10, scr_GameOver_00@l
bla _compareString
beq scaleGameOverPanes

lis r10, scr_FadeStatus_00@ha
addi r10, r10, scr_FadeStatus_00@l
bla _compareString
beq scaleLoadingStatusPanes

lis r10, scr_Skip_00@ha
addi r10, r10, scr_Skip_00@l
bla _compareString
beq scaleSkipButtonPanes

lis r10, scr_LoadSaveIcon_00@ha
addi r10, r10, scr_LoadSaveIcon_00@l
bla _compareString
beq scaleLoadSaveIconPanes

lis r10, scr_MainScreen_00@ha
addi r10, r10, scr_MainScreen_00@l
bla _compareString
beq scaleMainScreenPanes

lis r10, scr_MainDungeon_00@ha
addi r10, r10, scr_MainDungeon_00@l
bla _compareString
beq scaleMainDungeonPanes

lis r10, scr_MainScreenMS_00@ha
addi r10, r10, scr_MainScreenMS_00@l
bla _compareString
beq scaleMainScreenMasterSwordPanes

lis r10, scr_WolfLinkHeartGauge_00@ha
addi r10, r10, scr_WolfLinkHeartGauge_00@l
bla _compareString
beq scaleWolfLinkHeartPanes

lis r10, scr_Rupee_00@ha
addi r10, r10, scr_Rupee_00@l
bla _compareString
beq scaleSimpleItemGetPanes

lis r10, scr_AkashNum_00@ha
addi r10, r10, scr_AkashNum_00@l
bla _compareString
beq scaleSimpleItemGetPanes

lis r10, scr_KologNum_00@ha
addi r10, r10, scr_KologNum_00@l
bla _compareString
beq scaleSimpleItemGetPanes

lis r10, scr_MamoNum_00@ha
addi r10, r10, scr_MamoNum_00@l
bla _compareString
beq scaleSimpleItemGetPanes

lis r10, scr_KeyNum_00@ha
addi r10, r10, scr_KeyNum_00@l
bla _compareString
beq scaleSimpleItemGetPanes

lis r10, scr_Time_00@ha
addi r10, r10, scr_Time_00@l
bla _compareString
beq scaleSimpleItemGetPanes

lis r10, scr_MainScreen3D_00@ha
addi r10, r10, scr_MainScreen3D_00@l
bla _compareString
beq scaleMainScreen3DPanes

lis r10, scr_MainShortCut_00@ha
addi r10, r10, scr_MainShortCut_00@l
bla _compareString
beq scaleMainShortCutPanes

lis r10, scr_MainHorse_00@ha
addi r10, r10, scr_MainHorse_00@l
bla _compareString
beq scaleHorseStaminaPanes

lis r10, scr_MainHardMode_00@ha
addi r10, r10, scr_MainHardMode_00@l
bla _compareString
beq scaleMainHardModePanes

lis r10, scr_PauseMenu_00@ha
addi r10, r10, scr_PauseMenu_00@l
bla _compareString
beq scalePauseMenuPanes

lis r10, scr_AppHome_00@ha
addi r10, r10, scr_AppHome_00@l
bla _compareString
beq scalePauseHomePanes

lis r10, scr_AppTool_00@ha
addi r10, r10, scr_AppTool_00@l
bla _compareString
beq scalePauseRunePanes

lis r10, scr_AppAlbum_00@ha
addi r10, r10, scr_AppAlbum_00@l
bla _compareString
beq scalePauseAlbumPanes

lis r10, scr_AppPictureBook_00@ha
addi r10, r10, scr_AppPictureBook_00@l
bla _compareString
beq scalePausePictureBookPanes

lis r10, scr_SystemWindow_00@ha
addi r10, r10, scr_SystemWindow_00@l
bla _compareString
beq scaleSystemWindowInventoryPanes

lis r10, scr_ControllerWindow_00@ha
addi r10, r10, scr_ControllerWindow_00@l
bla _compareString
beq scalePauseMenuAbilityControlsPanes

lis r10, scr_PauseMenuBG_00@ha
addi r10, r10, scr_PauseMenuBG_00@l
bla _compareString
beq scalePauseMenuBGPanes

lis r10, scr_PauseMenuInfo_00@ha
addi r10, r10, scr_PauseMenuInfo_00@l
bla _compareString
beq scalePauseMenuInfoPanes

lis r10, scr_PauseMenuRecipe_00@ha
addi r10, r10, scr_PauseMenuRecipe_00@l
bla _compareString
beq scaleTopInOutToCenterAndPos

lis r10, scr_PauseMenuMantan_00@ha
addi r10, r10, scr_PauseMenuMantan_00@l
bla _compareString
beq scaleTopInOutToCenterAndPos

lis r10, scr_ShopHorse_00@ha
addi r10, r10, scr_ShopHorse_00@l
bla _compareString
beq scaleShopHorsePanes

lis r10, scr_ShopBtnList5_00@ha
addi r10, r10, scr_ShopBtnList5_00@l
bla _compareString
beq scaleShop05Panes

lis r10, scr_ShopBtnList15_00@ha
addi r10, r10, scr_ShopBtnList15_00@l
bla _compareString
beq scaleTopInOutToCenter

lis r10, scr_ShopBtnList20_00@ha
addi r10, r10, scr_ShopBtnList20_00@l
bla _compareString
beq scaleShop20Panes

lis r10, scr_ShopInfo_00@ha
addi r10, r10, scr_ShopInfo_00@l
bla _compareString
beq scaleTopInOutToCenter

lis r10, scr_AppMap_00@ha
addi r10, r10, scr_AppMap_00@l
bla _compareString
beq scaleMapPanes

lis r10, scr_AppMapDungeon_00@ha
addi r10, r10, scr_AppMapDungeon_00@l
bla _compareString
beq scaleDungeonMapPanes

lis r10, scr_AppCamera_00@ha
addi r10, r10, scr_AppCamera_00@l
bla _compareString
beq scaleCameraPanes

lis r10, scr_DoCommand_00@ha
addi r10, r10, scr_DoCommand_00@l
bla _compareString
beq scaleDoCommandPanes

lis r10, scr_PickUp_00@ha
addi r10, r10, scr_PickUp_00@l
bla _compareString
beq scalePickupPanes

lis r10, scr_SousaGuide_00@ha
addi r10, r10, scr_SousaGuide_00@l
bla _compareString
beq scaleRuneGuidePanes

lis r10, scr_MessageDialog@ha
addi r10, r10, scr_MessageDialog@l
bla _compareString
beq scaleMessageDialogPanes

lis r10, scr_MessageSp_00@ha
addi r10, r10, scr_MessageSp_00@l
bla _compareString
beq scaleMessageSpPanes

lis r10, str_MessageSp_00_NoTop@ha
addi r10, r10, str_MessageSp_00_NoTop@l
bla _compareString
beq scaleMessageSpPanes

lis r10, scr_MessageGet_00@ha
addi r10, r10, scr_MessageGet_00@l
bla _compareString
beq scaleMessageGetPanes

lis r10, scr_Message3D_00@ha
addi r10, r10, scr_Message3D_00@l
bla _compareString
beq scaleMessage3DPanes

lis r10, scr_DemoMessage@ha
addi r10, r10, scr_DemoMessage@l
bla _compareString
beq scaleInOutToCenter

lis r10, scr_DemoMessage_00@ha
addi r10, r10, scr_DemoMessage_00@l
bla _compareString
beq scaleInOutToCenter

lis r10, scr_DemoName@ha
addi r10, r10, scr_DemoName@l
bla _compareString
beq scaleInOutToCenter

lis r10, scr_DemoName_00@ha
addi r10, r10, scr_DemoName_00@l
bla _compareString
beq scaleInOutToCenter

lis r10, scr_DemoNameEnemy@ha
addi r10, r10, scr_DemoNameEnemy@l
bla _compareString
beq scaleDemoNameEnemyPanes

lis r10, scr_DemoNameEnemy_00@ha
addi r10, r10, scr_DemoNameEnemy_00@l
bla _compareString
beq scaleDemoNameEnemyPanes

lis r10, scr_LastComplete_00@ha
addi r10, r10, scr_LastComplete_00@l
bla _compareString
beq scaleLastCompletePanes

lis r10, scr_StaffRoll_00@ha
addi r10, r10, scr_StaffRoll_00@l
bla _compareString
beq scaleCreditPanes

lis r10, scr_StaffRollDLC_00@ha
addi r10, r10, scr_StaffRollDLC_00@l
bla _compareString
beq scaleDLCCreditPanes

lis r10, scr_Thanks_00@ha
addi r10, r10, scr_Thanks_00@l
bla _compareString
beq scaleThanksPanes

lis r10, scr_End_00@ha
addi r10, r10, scr_End_00@l
bla _compareString
beq scaleInOutToCenter

lis r10, scr_HardMode_00@ha
addi r10, r10, scr_HardMode_00@l
bla _compareString
beq scaleInOutToCenter

lis r10, scr_DLCWindow_00@ha
addi r10, r10, scr_DLCWindow_00@l
bla _compareString
beq scaleDLCWindowPanes

lis r10, scr_BoxCursorTV@ha
addi r10, r10, scr_BoxCursorTV@l
bla _compareString
beq scaleCursorPanes

lis r10, scr_KeyBoradTextArea_00@ha
addi r10, r10, scr_KeyBoradTextArea_00@l
bla _compareString
beq scaleInOutToCenter

lis r10, scr_GameTitle_00@ha
addi r10, r10, scr_GameTitle_00@l
bla _compareString
beq scaleGameTitlePanes

lis r10, scr_HardModeTextDLC_00@ha
addi r10, r10, scr_HardModeTextDLC_00@l
bla _compareString
beq scaleInOutScreenToLeftSide

lis r10, scr_MessageTipsRunTime_00@ha
addi r10, r10, scr_MessageTipsRunTime_00@l
bla _compareString
beq scaleRuntimeMessagePanes

lis r10, scr_MessageTips_00@ha
addi r10, r10, scr_MessageTips_00@l
bla _compareString
beq scaleMessageTipsPanes

lis r10, scr_OPtext_00@ha
addi r10, r10, scr_OPtext_00@l
bla _compareString
beq scaleInOutToCenter

lis r10, scr_ChallengeWin_00@ha
addi r10, r10, scr_ChallengeWin_00@l
bla _compareString
beq scaleInOutScreenToRightSide

lis r10, scr_EnergyMeterDLC_00@ha
addi r10, r10, scr_EnergyMeterDLC_00@l
bla _compareString
beq scaleEnergyMeterDLCPanes

; lis r10, scr_Message_00@ha
; addi r10, r10, scr_Message_00@l
; bla _compareString
; beq scaleInOutToCenter

; If nothing matched, exit without setting the size or position of this pane
b exitScale

; ------------------------------------------------------------------------------------------
; Methods used to scale a specific pane type

scaleMainScreenPanes:
lis r5, copySubPanelString@ha
addi r5, r5, copySubPanelString@l
lis r10, str_Pa_ArrowPointer_00@ha
addi r10, r10, str_Pa_ArrowPointer_00@l
bla _compareString
beq scaleInOutToCenter
lis r10, str_Pa_ItemPointer_00@ha
addi r10, r10, str_Pa_ItemPointer_00@l
bla _compareString
beq scaleMainScreenRunePointingPanes
lis r10, str_Pa_ThrowingPointer_00@ha
addi r10, r10, str_Pa_ThrowingPointer_00@l
bla _compareString
beq scaleInOutToCenter
lis r10, str_Pa_CameraPointer_00@ha
addi r10, r10, str_Pa_CameraPointer_00@l
bla _compareString
beq scaleMainScreenCameraPointerPanes
lis r10, str_Pa_BtnZR_00@ha
addi r10, r10, str_Pa_BtnZR_00@l
bla _compareString
beq scaleMainScreenScopeFramePanes
lis r10, str_Pa_Deco_00@ha
addi r10, r10, str_Pa_Deco_00@l
bla _compareString
beq scaleMainScreenScopeFramePanes
lis r10, str_Pa_Gauge_00@ha
addi r10, r10, str_Pa_Gauge_00@l
bla _compareString
beq scaleMainScreenGaugePanes

lis r10, str_Pa_Time_00@ha
addi r10, r10, str_Pa_Time_00@l
bla _compareString
beq scaleMainScreenTimePanes
lis r10, str_Pa_Sensor_00@ha
addi r10, r10, str_Pa_Sensor_00@l
bla _compareString
beq scaleSensorMeterPanes
lis r10, str_Pa_SoundGauge_00@ha
addi r10, r10, str_Pa_SoundGauge_00@l
bla _compareString
beq scaleSoundMeterPanes
lis r10, str_Pa_TempMeter_00@ha
addi r10, r10, str_Pa_TempMeter_00@l
bla _compareString
beq scaleTempMeterPanes
lis r10, str_Pa_Weather_00@ha
addi r10, r10, str_Pa_Weather_00@l
bla _compareString
beq scaleMainScreenWeatherPanes

lis r10, str_Pa_Information_00@ha
addi r10, r10, str_Pa_Information_00@l
bla _compareString
beq scaleMainScreenInformationTextPanes
lis r10, str_Pa_LocationName_00@ha
addi r10, r10, str_Pa_LocationName_00@l
bla _compareString
beq scaleMainScreenLocationTextPanes
lis r10, str_Pa_LocationNameS_00@ha
addi r10, r10, str_Pa_LocationNameS_00@l
bla _compareString
beq scaleMainScreenSmallLocationTextPanes
lis r10, str_Pa_QuestName_00@ha
addi r10, r10, str_Pa_QuestName_00@l
bla _compareString
beq scaleMainScreenQuestTextPanes
lis r10, str_Pa_BossGauge_00@ha
addi r10, r10, str_Pa_BossGauge_00@l
bla _compareString
beq scaleMainScreenBossGaugePanes

addi r5, r31, 0x80
lis r10, str_N_State_00@ha
addi r10, r10, str_N_State_00@l
bla _compareString
beq scalePaneToLeftSideIf
lis r10, str_Pa_SinJu_00@ha
addi r10, r10, str_Pa_SinJu_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_Pa_SinJu_01@ha
addi r10, r10, str_Pa_SinJu_01@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_Pa_SinJu_02@ha
addi r10, r10, str_Pa_SinJu_02@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_Pa_SinJu_03@ha
addi r10, r10, str_Pa_SinJu_03@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_Pa_SinJu_03@ha
addi r10, r10, str_Pa_SinJu_03@l
bla _compareString
beq scalePaneAndKeepPosIf
b exitScale

const_SensorsOffset:
.float 392.0
scaleSoundMeterPanes:
addi r5, r31, 0x80
lis r10, const_SensorsOffset@ha
lfs f12, const_SensorsOffset@l(r10)
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndKeepCustomPosIf
b exitScale
scaleSensorMeterPanes:
addi r5, r31, 0x80
lis r10, const_SensorsOffset@ha
lfs f12, const_SensorsOffset@l(r10)
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndKeepCustomPosIf
b exitScale
scaleTempMeterPanes:
addi r5, r31, 0x80
lis r10, const_SensorsOffset@ha
lfs f12, const_SensorsOffset@l(r10)
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndKeepCustomPosIf
b exitScale
const_TimeOffset:
.float (($ultrawideHUDMode == 1) * (342*0.85)) + (($ultrawideHUDMode == 2)*(342*(1/0.85)))
scaleMainScreenTimePanes:
addi r5, r31, 0x80
lis r10, const_TimeOffset@ha
lfs f12, const_TimeOffset@l(r10)
lis r10, str_N_All_00@ha
addi r10, r10, str_N_All_00@l
bla _compareString
beq scalePaneAndKeepCustomPosIf
b exitScale
scaleMainScreenWeatherPanes:
addi r5, r31, 0x80
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndKeepPosIf
b exitScale
scaleMainScreenInformationTextPanes:
addi r5, r31, 0x80
lis r10, str_Pa_Information_00@ha
addi r10, r10, str_Pa_Information_00@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleMainScreenLocationTextPanes:
addi r5, r31, 0x80
lis r10, str_Pa_LocationName_00@ha
addi r10, r10, str_Pa_LocationName_00@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleMainScreenQuestTextPanes:
addi r5, r31, 0x80
lis r10, str_Pa_QuestName_00@ha
addi r10, r10, str_Pa_QuestName_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Base_01@ha
addi r10, r10, str_P_Base_01@l
bla _compareString
beq scalePaneReverse
lis r10, str_P_Base_00@ha
addi r10, r10, str_P_Base_00@l
bla _compareString
beq scalePaneReverse
b exitScale

scaleMainScreenSmallLocationTextPanes:
addi r5, r31, 0x80
lis r10, str_Pa_LocationNameS_00@ha
addi r10, r10, str_Pa_LocationNameS_00@l
bla _compareString
beq scalePaneToLeftSideIf
b exitScale
scaleMainScreenBossGaugePanes:
addi r5, r31, 0x80
lis r10, str_N_All_00@ha
addi r10, r10, str_N_All_00@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleMainScreenGaugePanes:
addi r5, r31, 0x80
lis r10, str_N_All_00@ha
addi r10, r10, str_N_All_00@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleMainScreenCameraPointerPanes:
addi r5, r31, 0x80
lis r10, str_N_InOut_01@ha
addi r10, r10, str_N_InOut_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Dungeon_00@ha
addi r10, r10, str_N_Dungeon_00@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleMainScreenScopeFramePanes:
addi r5, r31, 0x80
lis r10, str_N_TimeLineDemo_00@ha
addi r10, r10, str_N_TimeLineDemo_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_IconSeek_00@ha
addi r10, r10, str_N_IconSeek_00@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleMainScreenRunePointingPanes:
addi r5, r31, 0x80
lis r10, str_N_SunLight_00@ha
addi r10, r10, str_N_SunLight_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleMainDungeonPanes:
addi r5, r31, 0x80
lis r10, str_Pa_LocationNameS_00@ha
addi r10, r10, str_Pa_LocationNameS_00@l
bla _compareString
beq scalePaneToLeftSideIf
lis r10, str_Pa_Message_00@ha
addi r10, r10, str_Pa_Message_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_Pa_Title_00@ha
addi r10, r10, str_Pa_Title_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleMainDungeonLocationNames:
addi r5, r31, 0x80
lis r10, str_N_In_00@ha
addi r10, r10, str_N_In_00@l
bla _compareString
beq scalePaneToLeftSideIf
b exitScale

scaleSpiritOrbPanes:
addi r5, r31, 0x80
lis r10, str_W_Base_00@ha
addi r10, r10, str_W_Base_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_T_Time_00@ha
addi r10, r10, str_T_Time_00@l
bla _compareString
beq scalePaneAndKeepPosIf
b exitScale

scaleSimpleItemGetPanes:
lis r5, copySubPanelString@ha
lbz r5, copySubPanelString@l(r5)
cmpwi r5, 0
beq scaleItemGetNoPanes

continueSimpleItemGetPanes:

lis r5, copySubPanelString@ha
addi r5, r5, copySubPanelString@l
lis r10, scr_PaPlusMinus_00@ha
addi r10, r10, scr_PaPlusMinus_00@l
bla _compareString
beq scaleItemGetPlusMinusPanes

addi r5, r31, 0x80
lis r10, str_W_Base_00@ha
addi r10, r10, str_W_Base_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_T_Num_00@ha
addi r10, r10, str_T_Num_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_T_Time_00@ha
addi r10, r10, str_T_Time_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_T_Rupee_00@ha
addi r10, r10, str_T_Rupee_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_T_KeyNum_00@ha
addi r10, r10, str_T_KeyNum_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_P_Icon_00@ha
addi r10, r10, str_P_Icon_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_N_Icon_00@ha
addi r10, r10, str_N_Icon_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_P_KologNuts_00@ha
addi r10, r10, str_P_KologNuts_00@l
bla _compareString
beq scalePaneAndKeepPosIf
b exitScale

const_ItemGetPlusMinusOffset:
.float -514.0
scaleItemGetPlusMinusPanes:
addi r5, r31, 0x80
lis r10, const_ItemGetPlusMinusOffset@ha
lfs f12, const_ItemGetPlusMinusOffset@l(r10)
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndKeepCustomPosIf
b exitScale

scaleItemGetNoPanes:
addi r5, r31, 0x80
lis r10, str_N_Capture_00@ha
addi r10, r10, str_N_Capture_00@l
bla _compareString
beq scalePaneAndKeepPosIf
b continueSimpleItemGetPanes

scaleWolfLinkHeartPanes:
lis r5, copySubPanelString@ha
addi r5, r5, copySubPanelString@l

lis r10, str_Pa_HeartGauge_00@ha
addi r10, r10, str_Pa_HeartGauge_00@l
bla _compareString
beq scaleWolfLinkHeartsPanes

lis r5, copySubPanelString@ha
lbz r5, copySubPanelString@l(r5)
cmpwi r5, 0
beq scaleWolfLinkHeartNoPanes
b exitScale

scaleWolfLinkHeartNoPanes:
addi r5, r31, 0x80
lis r10, str_P_Sh_00@ha
addi r10, r10, str_P_Sh_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_P_Illust_00@ha
addi r10, r10, str_P_Illust_00@l
bla _compareString
beq scalePaneAndKeepPosIf
b exitScale

const_WolfHeartOffset:
.float -468.0
scaleWolfLinkHeartsPanes:
addi r5, r31, 0x80
lis r10, str_T_Name_00@ha
addi r10, r10, str_T_Name_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, const_WolfHeartOffset@ha
lfs f12, const_WolfHeartOffset@l(r10)
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndKeepCustomPosIf
b exitScale

scaleMainScreenMasterSwordPanes:
lis r5, copySubPanelString@ha
lbz r5, copySubPanelString@l(r5)
cmpwi r5, 0
beq scaleMainScreenMasterSwordNoPanes
b exitScale

scaleMainScreenMasterSwordNoPanes:
addi r5, r31, 0x80
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleMainScreen3DPanes:
lis r5, copySubPanelString@ha
addi r5, r5, copySubPanelString@l

lis r10, str_Pa_NoticeItem_00@ha
addi r10, r10, str_Pa_NoticeItem_00@l
bla _compareString
beq scaleMainScreen3DPickupPanes
lis r10, str_Pa_StaminaGauge_00@ha
addi r10, r10, str_Pa_StaminaGauge_00@l
bla _compareString
beq scaleMainScreen3DStaminaPanes
lis r10, str_Pa_EnemyMark_00@ha
addi r10, r10, str_Pa_EnemyMark_00@l
bla _compareString
beq scaleMainScreen3DEnemyNoticePanes
lis r10, str_Pa_EnemyGauge_00@ha
addi r10, r10, str_Pa_EnemyGauge_00@l
bla _compareString
beq scaleMainScreen3DEnemyGaugePanes
lis r10, str_Pa_NoticeZ_00@ha
addi r10, r10, str_Pa_NoticeZ_00@l
bla _compareString
beq scaleInOutToCenter
lis r10, str_Pa_BonusStaminaGauge_00@ha
addi r10, r10, str_Pa_BonusStaminaGauge_00@l
bla _compareString
beq scaleMainScreen3DBonusStaminaPanes
b exitScale

scaleMainScreen3DStaminaPanes:
addi r5, r31, 0x80
lis r10, str_N_All_00@ha
addi r10, r10, str_N_All_00@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleMainScreen3DBonusStaminaPanes:
addi r5, r31, 0x80
lis r10, str_N_Size_05@ha
addi r10, r10, str_N_Size_05@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleMainScreen3DPickupPanes:
addi r5, r31, 0x80
lis r10, str_N_Capture_00@ha
addi r10, r10, str_N_Capture_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_ModeItem_00@ha
addi r10, r10, str_N_ModeItem_00@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleMainScreen3DEnemyNoticePanes:
addi r5, r31, 0x80
lis r10, str_N_Change_00@ha
addi r10, r10, str_N_Change_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Change_01@ha
addi r10, r10, str_N_Change_01@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleMainScreen3DEnemyGaugePanes:
addi r5, r31, 0x80
lis r10, str_N_Gauge_00@ha
addi r10, r10, str_N_Gauge_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleHorseStaminaPanes:
addi r5, r31, 0x80
lis r10, str_N_ExUse_01@ha
addi r10, r10, str_N_ExUse_01@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleMainShortCutPanes:
addi r5, r31, 0x80
lis r10, str_N_MainAll_00@ha ; scales the DPAD guide on the right
addi r10, r10, str_N_MainAll_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_N_Cut_00@ha ; scales the item/rune selection bar
addi r10, r10, str_N_Cut_00@l
bla _compareString
beq scalePaneNormal

lis r10, str_N_Capture_00@ha
addi r10, r10, str_N_Capture_00@l
bla _compareString
beq scalePaneAndPos
lis r10, str_N_Capture_01@ha
addi r10, r10, str_N_Capture_01@l
bla _compareString
beq scalePaneAndPos

lis r10, str_P_CaptureMask_00@ha
addi r10, r10, str_P_CaptureMask_00@l
bla _compareString
beq scalePaneAndPos
lis r10, str_P_CaptureMask_01@ha
addi r10, r10, str_P_CaptureMask_01@l
bla _compareString
beq scalePaneAndPos

lis r10, str_N_PartsSize_00@ha ; reverse scaling on pane that decides spacing between runes/items
addi r10, r10, str_N_PartsSize_00@l
bla _compareString
beq scalePaneReverse
b exitScale

scaleMainHardModePanes:
addi r5, r31, 0x80
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_N_Glow_00@ha
addi r10, r10, str_N_Glow_00@l
bla _compareString
beq scalePaneAndKeepPos
b exitScale

scaleMapPanes:
lis r5, copySubPanelString@ha
lbz r5, copySubPanelString@l(r5)
cmpwi r5, 0
beq scaleMapNoPanes

lis r5, copySubPanelString@ha
addi r5, r5, copySubPanelString@l

lis r10, str_Pa_Map_00@ha
addi r10, r10, str_Pa_Map_00@l
bla _compareString
beq scaleMapTilesPanes
; Scale sensors background back
# lis r10, str_Pa_SensorBox_00@ha
# addi r10, r10, str_Pa_SensorBox_00@l
# bla _compareString
# beq scaleMapSettingsPanes
# lis r10, str_Pa_SubBtnChange_00@ha
# addi r10, r10, str_Pa_SubBtnChange_00@l
# bla _compareString
# beq scaleMapSettingsPanes
# ; Scale stamp box background back
# lis r10, str_Pa_StampBox_00@ha
# addi r10, r10, str_Pa_StampBox_00@l
# bla _compareString
# beq scaleMapSettingsPanes
# lis r10, str_Pa_StampNum_00@ha
# addi r10, r10, str_Pa_StampNum_00@l
# bla _compareString
# beq scaleMapSettingsPanes

addi r5, r31, 0x80

# lis r10, str_N_Cut_01@ha
# addi r10, r10, str_N_Cut_01@l
# bla _compareString
# beq scalePaneReverse

# lis r10, str_N_Sunaarashi_00@ha
# addi r10, r10, str_N_Sunaarashi_00@l
# bla _compareString
# beq scalePaneReverse

# lis r10, str_Pa_MapOpen_00@ha
# addi r10, r10, str_Pa_MapOpen_00@l
# bla _compareString
# beq scalePaneNormal
b exitScale

scaleMapNoPanes:
addi r5, r31, 0x80

lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scaleMapAndStore

lis r10, str_N_Cut_00@ha
addi r10, r10, str_N_Cut_00@l
bla _compareString
beq scaleOnlySizeReverse

lis r10, str_Black8_01@ha
addi r10, r10, str_Black8_01@l
bla _compareString
beq scalePaneReverse
b exitScale

scaleMapTilesPanes:
addi r5, r31, 0x80

lis r10, str_B_MapCapture_00@ha
addi r10, r10, str_B_MapCapture_00@l
bla _compareString
beq scaleOnlySizeReverse

# lis r10, str_N_MiniMap_00@ha
# addi r10, r10, str_N_MiniMap_00@l
# bla _compareString
# beq movePaneToRightSide
b exitScale

scaleMapSettingsPanes:
addi r5, r31, 0x80

lis r10, str_P_FBLayout_00@ha
addi r10, r10, str_P_FBLayout_00@l
bla _compareString
beq scalePaneReverse
lis r10, str_White8_00@ha
addi r10, r10, str_White8_00@l
bla _compareString
beq scalePaneReverse
lis r10, str_White8_01@ha
addi r10, r10, str_White8_01@l
bla _compareString
beq scalePaneReverse
lis r10, str_P_BG_03@ha
addi r10, r10, str_P_BG_03@l
bla _compareString
beq scalePaneReverse
lis r10, str_P_Abstract_00@ha
addi r10, r10, str_P_Abstract_00@l
bla _compareString
beq scalePaneReverse
lis r10, str_P_Abstract_05@ha
addi r10, r10, str_P_Abstract_05@l
bla _compareString
beq scalePaneReverse
lis r10, str_P_SSFrameCore_00@ha
addi r10, r10, str_P_SSFrameCore_00@l
bla _compareString
beq scalePaneReverse
b exitScale

scaleDungeonMapPanes:
; could-fix: hook dungeon map changing too
lis r5, copySubPanelString@ha
lbz r5, copySubPanelString@l(r5)
cmpwi r5, 0
beq scaleDungeonMapNoPanes

lis r5, copySubPanelString@ha
addi r5, r5, copySubPanelString@l
lis r10, str_Pa_SensorBox_00@ha
addi r10, r10, str_Pa_SensorBox_00@l
bla _compareString
beq scaleDungeonMapSensorBoxPanes
lis r10, str_Pa_SubBtnChange_00@ha
addi r10, r10, str_Pa_SubBtnChange_00@l
bla _compareString
beq scaleDungeonMapSensorBoxPanes
lis r10, str_Pa_Btn_00@ha
addi r10, r10, str_Pa_Btn_00@l
bla _compareString
beq scaleDungeonMapSensorBoxPanes

addi r5, r31, 0x80
b exitScale

scaleDungeonMapNoPanes:
addi r5, r31, 0x80
lis r10, str_N_Born_00@ha
addi r10, r10, str_N_Born_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleDungeonMapSensorBoxPanes:
addi r5, r31, 0x80
lis r10, str_Pa_SensorBox_00@ha
addi r10, r10, str_Pa_SensorBox_00@l
bla _compareString
beq scalePaneReverse
lis r10, str_N_BtnPos@ha
addi r10, r10, str_N_BtnPos@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_SubIn_00@ha
addi r10, r10, str_N_SubIn_00@l
bla _compareString
beq scalePaneAndPos
lis r10, str_P_Sh_00@ha
addi r10, r10, str_P_Sh_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_T_SensorTitle_00@ha
addi r10, r10, str_T_SensorTitle_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleCameraPanes:
lis r5, copySubPanelString@ha
addi r5, r5, copySubPanelString@l
lis r10, str_Pa_BtnZR_00@ha
addi r10, r10, str_Pa_BtnZR_00@l
bla _compareString
beq scaleCameraFramePanes
lis r10, str_Pa_Deco_00@ha
addi r10, r10, str_Pa_Deco_00@l
bla _compareString
beq scaleCameraFramePanes

addi r5, r31, 0x80
lis r10, str_N_ModeCamera_00@ha
addi r10, r10, str_N_ModeCamera_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_FirstOut_00@ha
addi r10, r10, str_N_FirstOut_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Limit_00@ha
addi r10, r10, str_N_Limit_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Capture_00@ha
addi r10, r10, str_P_Capture_00@l
bla _compareString
beq scalePaneReverse
b exitScale
scaleCameraFramePanes:
addi r5, r31, 0x80
lis r10, str_N_TimeLineDemo_00@ha
addi r10, r10, str_N_TimeLineDemo_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_IconSeek_00@ha
addi r10, r10, str_N_IconSeek_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleDoCommandPanes:
addi r5, r31, 0x80
lis r10, str_N_MS_00@ha
addi r10, r10, str_N_MS_00@l
bla _compareString
beq scalePaneNormal
b exitScale

const_590:
.float 25

scalePickupPanes:
addi r5, r31, 0x80
lis r10, const_590@ha
lfs f12, const_590@l(r10)
# lis r10, str_RootPane@ha
# addi r10, r10, str_RootPane@l
# bla _compareString
# beq movePaneToCustomSize
lis r10, str_Pa_PickUpWin_00@ha
addi r10, r10, str_Pa_PickUpWin_00@l
bla _compareString
beq scalePaneAndKeepPosIf
b exitScale

scaleRuneGuidePanes:
addi r5, r31, 0x80
lis r10, str_N_Align_00@ha
addi r10, r10, str_N_Align_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleMessageSpPanes:
addi r5, r31, 0x80
lis r10, str_N_Seeker_00@ha
addi r10, r10, str_N_Seeker_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Pos_00@ha
addi r10, r10, str_N_Pos_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_BtnNum_00@ha
addi r10, r10, str_N_BtnNum_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Cursor_00@ha
addi r10, r10, str_N_Cursor_00@l
bla _compareString
beq movePaneToRightSideAlt
lis r10, str_B_Hit_00@ha
addi r10, r10, str_B_Hit_00@l
bla _compareString
beq movePaneToRightSideAlt
b exitScale

scaleMessageDialogPanes:
addi r5, r31, 0x80
lis r10, str_N_Pos_00@ha
addi r10, r10, str_N_Pos_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Cursor_00@ha
addi r10, r10, str_N_Cursor_00@l
bla _compareString
beq movePaneToRightSideAlt
lis r10, str_B_Hit_00@ha
addi r10, r10, str_B_Hit_00@l
bla _compareString
beq movePaneToRightSideAlt
b exitScale

scaleMessageGetPanes:
lis r5, copySubPanelString@ha
addi r5, r5, copySubPanelString@l
lis r10, str_Pa_Star_00@ha
addi r10, r10, str_Pa_Star_00@l
bla _compareString
beq scaleMessageGetBackgroundAndIconPanes

addi r5, r31, 0x80
lis r10, str_N_DecideOut_00@ha
addi r10, r10, str_N_DecideOut_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Capture_01@ha
addi r10, r10, str_P_Capture_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Capture_04@ha
addi r10, r10, str_P_Capture_04@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Capture_02@ha
addi r10, r10, str_P_Capture_02@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleMessageGetBackgroundAndIconPanes:
addi r5, r31, 0x80
lis r10, str_W_Base_00@ha
addi r10, r10, str_W_Base_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Icon_00@ha
addi r10, r10, str_N_Icon_00@l
bla _compareString
beq scalePaneAndPos
lis r10, str_N_State_00@ha
addi r10, r10, str_N_State_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Capture_00@ha
addi r10, r10, str_P_Capture_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Capture_03@ha
addi r10, r10, str_P_Capture_03@l
bla _compareString
beq scalePaneNormal
lis r10, str_Nt_DecoLT_00@ha
addi r10, r10, str_Nt_DecoLT_00@l
bla _compareString
beq scalePaneAndPos
lis r10, str_Nt_DecoLT_01@ha
addi r10, r10, str_Nt_DecoLT_01@l
bla _compareString
beq scalePaneAndPos
lis r10, str_Nt_DecoRT_00@ha
addi r10, r10, str_Nt_DecoRT_00@l
bla _compareString
beq scalePaneAndPos
lis r10, str_Nt_DecoRT_01@ha
addi r10, r10, str_Nt_DecoRT_01@l
bla _compareString
beq scalePaneAndPos
lis r10, str_Nt_DecoLB_00@ha
addi r10, r10, str_Nt_DecoLB_00@l
bla _compareString
beq scalePaneAndPos
lis r10, str_Nt_DecoLB_01@ha
addi r10, r10, str_Nt_DecoLB_01@l
bla _compareString
beq scalePaneAndPos
lis r10, str_Nt_DecoRB_00@ha
addi r10, r10, str_Nt_DecoRB_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_Nt_DecoRB_01@ha
addi r10, r10, str_Nt_DecoRB_01@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleMessage3DPanes:
addi r5, r31, 0x80
lis r10, str_N_Capture_00@ha
addi r10, r10, str_N_Capture_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_All_00@ha
addi r10, r10, str_N_All_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleRuntimeMessagePanes:
addi r5, r31, 0x80
lis r10, str_W_Base_00@ha
addi r10, r10, str_W_Base_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_T_Text_00@ha
addi r10, r10, str_T_Text_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_T_Text_00_JPja@ha
addi r10, r10, str_T_Text_00_JPja@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_N_GuideOn_00@ha
addi r10, r10, str_N_GuideOn_00@l
bla _compareString
beq scalePaneAndKeepPosIf
b exitScale

scaleCursorPanes:
addi r5, r31, 0x80
lis r10, str_N_Porch_00@ha
addi r10, r10, str_N_Porch_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Seek_00@ha
addi r10, r10, str_N_Seek_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Porch_01@ha
addi r10, r10, str_N_Porch_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Seek_01@ha
addi r10, r10, str_N_Seek_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Porch_02@ha
addi r10, r10, str_N_Porch_02@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Seek_02@ha
addi r10, r10, str_N_Seek_02@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Porch_03@ha
addi r10, r10, str_N_Porch_03@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Seek_03@ha
addi r10, r10, str_N_Seek_03@l
bla _compareString
beq scalePaneNormal
b exitScale

scalePauseMenuPanes:
addi r5, r31, 0x80
lis r5, copySubPanelString@ha
lbz r5, copySubPanelString@l(r5)
cmpwi r5, 0
beq scalePauseMenuNoPanes

lis r5, copySubPanelString@ha
addi r5, r5, copySubPanelString@l

lis r10, str_Pa_Quest_00@ha
addi r10, r10, str_Pa_Quest_00@l
bla _compareString
beq scalePauseMenuQuestPanes

addi r5, r31, 0x80
lis r10, str_N_Slide_02@ha ; scales save/controller screen fully
addi r10, r10, str_N_Slide_02@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_InCap_01@ha ; scales masks for inventory tabs
addi r10, r10, str_N_InCap_01@l
bla _compareString
beq scalePaneAndPos
lis r10, str_N_InOutGear_01@ha ; scales category and tabs in inventory screen
addi r10, r10, str_N_InOutGear_01@l
bla _compareString
beq scalePaneAndPos

lis r10, str_N_InOutQuest_02@ha ; scales quest slide
addi r10, r10, str_N_InOutQuest_02@l
bla _compareString
beq scalePaneNormal
lis r10, str_Pa_Page_00@ha
addi r10, r10, str_Pa_Page_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scalePauseMenuNoPanes:
addi r5, r31, 0x80
lis r10, str_B_Page20_00@ha ; Scales inventory placement
addi r10, r10, str_B_Page20_00@l
bla _compareString
beq scalePaneAndPos
lis r10, str_N_InOut_00@ha ; Scales inventory screen
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndPos
b exitScale

scalePauseMenuQuestPanes:
addi r5, r31, 0x80
lis r10, str_N_Cut_00@ha
addi r10, r10, str_N_Cut_00@l
bla _compareString
beq scalePaneReverse
b exitScale

scalePauseMenuBGPanes: ; TODO: Maybe could look a bit nicer
addi r5, r31, 0x80
; lis r10, str_W_BlackBG_00@ha
; addi r10, r10, str_W_BlackBG_00@l
; bla _compareString
; beq scalePaneAndSize
; lis r10, str_N_Slide_01@ha
; addi r10, r10, str_N_Slide_01@l
; bla _compareString
; beq scalePaneAndSize
; lis r10, str_N_Slide_02@ha
; addi r10, r10, str_N_Slide_02@l
; bla _compareString
; beq scalePaneAndSize
b exitScale

scalePauseMenuInfoPanes:
lis r5, copySubPanelString@ha
lbz r5, copySubPanelString@l(r5)
cmpwi r5, 0
beq scalePauseMenuInfoNoPanes

addi r5, r31, 0x80
lis r10, str_N_InOut_01@ha
addi r10, r10, str_N_InOut_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Slide_00@ha
addi r10, r10, str_N_Slide_00@l
bla _compareString
beq scaleOnlyPos
lis r10, str_Pa_RotateGuide_00@ha
addi r10, r10, str_Pa_RotateGuide_00@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_N_State_00@ha
addi r10, r10, str_N_State_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_State_03@ha
addi r10, r10, str_N_State_03@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_TitleBar_00@ha
addi r10, r10, str_N_TitleBar_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_TitleBarDown_00@ha
addi r10, r10, str_N_TitleBarDown_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scalePauseMenuInfoNoPanes:
addi r5, r31, 0x80
; lis r10, str_N_InOut_00@ha
; addi r10, r10, str_N_InOut_00@l
; bla _compareString
; beq scalePaneNormal
b exitScale

scalePauseHomePanes:
addi r5, r31, 0x80
lis r10, str_N_IconSeek_00@ha
addi r10, r10, str_N_IconSeek_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_TimeLineDemo_00@ha
addi r10, r10, str_N_TimeLineDemo_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scalePauseRunePanes:
addi r5, r31, 0x80
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_NoContents_00@ha
addi r10, r10, str_N_NoContents_00@l
bla _compareString
beq movePaneToRightSide
b exitScale

scalePauseAlbumPanes:
addi r5, r31, 0x80
lis r10, str_Pa_Page_00@ha
addi r10, r10, str_Pa_Page_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_InOut_02@ha
addi r10, r10, str_N_InOut_02@l
bla _compareString
beq scalePaneNormal
lis r10, str_Pa_Photo_00@ha
addi r10, r10, str_Pa_Photo_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_NoContents_00@ha
addi r10, r10, str_N_NoContents_00@l
bla _compareString
beq movePaneToRightSide
b exitScale

scalePausePictureBookPanes:
addi r5, r31, 0x80
lis r10, str_Pa_Page_00@ha
addi r10, r10, str_Pa_Page_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Contents_00@ha
addi r10, r10, str_N_Contents_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_PartsSize_00@ha
addi r10, r10, str_N_PartsSize_00@l
bla _compareString
beq scalePaneReverse

lis r10, str_N_InOut_02@ha
addi r10, r10, str_N_InOut_02@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_NoContents_00@ha
addi r10, r10, str_N_NoContents_00@l
bla _compareString
beq movePaneToRightSide
lis r10, str_N_MapPos_00@ha
addi r10, r10, str_N_MapPos_00@l
bla _compareString
beq moveKeepPos
b exitScale

scaleShopHorsePanes:
lis r5, copySubPanelString@ha
lbz r5, copySubPanelString@l(r5)
cmpwi r5, 0
beq scaleShopHorseNoPanes

addi r5, r31, 0x80
lis r10, str_N_Info_00@ha
addi r10, r10, str_N_Info_00@l
bla _compareString
beq scalePaneNormal
b exitScale
scaleShopHorseNoPanes:
addi r5, r31, 0x80
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleShop05Panes:
addi r5, r31, 0x80
lis r10, str_N_InOut_01@ha
addi r10, r10, str_N_InOut_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_InOut_02@ha
addi r10, r10, str_N_InOut_02@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleShop20Panes:
addi r5, r31, 0x80
lis r10, str_N_Cut_00@ha
addi r10, r10, str_N_Cut_00@l
bla _compareString
beq scaleOnlyPos
lis r10, str_Pa_Page_4x5_00@ha
addi r10, r10, str_Pa_Page_4x5_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_InOut_02@ha
addi r10, r10, str_N_InOut_02@l
bla _compareString
beq scalePaneNormal
# lis r10, str_N_Cat_01@ha
# addi r10, r10, str_N_Cat_01@l
# bla _compareString
# beq scalePaneReverse
b exitScale

const_SkipButtonOffset:
.float 560

scaleSkipButtonPanes:
addi r5, r31, 0x80
lis r10, const_SkipButtonOffset@ha
lfs f12, const_SkipButtonOffset@l(r10)
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndKeepCustomPosIf
lis r10, str_N_Glow_00@ha
addi r10, r10, str_N_Glow_00@l
bla _compareString
beq scalePaneAndKeepCustomPosIf
b exitScale

scaleLoadSaveIconPanes:
addi r5, r31, 0x80
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_N_Glow_00@ha
addi r10, r10, str_N_Glow_00@l
bla _compareString
beq scalePaneAndKeepPos
b exitScale

scaleLoadingPanes: ; works, but current code is very janky
addi r5, r31, 0x80

lis r10, str_N_RegionTypePos_00@ha
addi r10, r10, str_N_RegionTypePos_00@l
bla _compareString
beq scalePaneToLeftSide
lis r10, str_Pa_GuideA_00@ha
addi r10, r10, str_Pa_GuideA_00@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_N_LogoInOut_02@ha
addi r10, r10, str_N_LogoInOut_02@l
bla _compareString
beq scalePaneToRightSide

lis r10, str_T_TipsTitle_00@ha
addi r10, r10, str_T_TipsTitle_00@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_T_TipsTitle_00_JPja@ha
addi r10, r10, str_T_TipsTitle_00_JPja@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_T_TipsTitle_00_KR@ha
addi r10, r10, str_T_TipsTitle_00_KR@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_T_TipsTitle_00_CN@ha
addi r10, r10, str_T_TipsTitle_00_CN@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_T_TipsTitle_00_TW@ha
addi r10, r10, str_T_TipsTitle_00_TW@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_T_Tips_00@ha
addi r10, r10, str_T_Tips_00@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_T_Tips_00_JPja@ha
addi r10, r10, str_T_Tips_00_JPja@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_T_Tips_00_KR@ha
addi r10, r10, str_T_Tips_00_KR@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_T_Tips_00_CN@ha
addi r10, r10, str_T_Tips_00_CN@l
bla _compareString
beq scalePaneAndKeepPos
lis r10, str_T_Tips_00_TW@ha
addi r10, r10, str_T_Tips_00_TW@l
bla _compareString
beq scalePaneAndKeepPos
b exitScale

scaleLoadingStatusPanes:
addi r5, r31, 0x80
lis r10, str_N_ParamInOut_00@ha
addi r10, r10, str_N_ParamInOut_00@l
bla _compareString
beq scalePaneToRightSide
b exitScale

scaleGameOverPanes:
addi r5, r31, 0x80
lis r10, str_N_Aka_00@ha
addi r10, r10, str_N_Aka_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Ao_00@ha
addi r10, r10, str_N_Ao_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Kiiro_00@ha
addi r10, r10, str_N_Kiiro_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_Pa_Btn_00@ha
addi r10, r10, str_Pa_Btn_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_Pa_Btn_02@ha
addi r10, r10, str_Pa_Btn_02@l
bla _compareString
beq scalePaneNormal
lis r10, str_T_SubText_00@ha
addi r10, r10, str_T_SubText_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleSystemWindowInventoryPanes:
addi r5, r31, 0x80
lis r10, str_N_TypeBtn_01@ha
addi r10, r10, str_N_TypeBtn_01@l
bla _compareString
beq scalePaneAndPos
lis r10, str_N_InOut_01@ha
addi r10, r10, str_N_InOut_01@l
bla _compareString
beq scalePaneNormal
b exitScale

scalePauseMenuAbilityControlsPanes:
addi r5, r31, 0x80
lis r10, str_N_Option_00@ha
addi r10, r10, str_N_Option_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Align_00@ha
addi r10, r10, str_N_Align_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleSystemWindowPanes:
addi r5, r31, 0x80
lis r10, str_N_SlideArea_00@ha
addi r10, r10, str_N_SlideArea_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_InOut_03@ha
addi r10, r10, str_N_InOut_03@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_FileSelect_01@ha
addi r10, r10, str_N_FileSelect_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_HardModePos_00@ha
addi r10, r10, str_N_HardModePos_00@l
bla _compareString
beq scalePaneToLeftSide
b exitScale

scaleAppSystemWindowPanes:
addi r5, r31, 0x80
lis r10, str_N_BtnNum_00@ha
addi r10, r10, str_N_BtnNum_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_BtnPos_00@ha
addi r10, r10, str_N_BtnPos_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleOptionsMenuPanes:
addi r5, r31, 0x80
lis r10, str_N_Option_00@ha
addi r10, r10, str_N_Option_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Option_01@ha
addi r10, r10, str_N_Option_01@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleDLCWindowPanes:
addi r5, r31, 0x80
lis r10, str_N_Option_00@ha
addi r10, r10, str_N_Option_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleTitlePanes:
addi r5, r31, 0x80
lis r10, str_N_RegionType_00@ha ; Logo
addi r10, r10, str_N_RegionType_00@l
bla _compareString
beq scalePaneToRightSide
lis r10, str_N_RegionType_00_JPja@ha ; Logo
addi r10, r10, str_N_RegionType_00_JPja@l
bla _compareString
beq scalePaneToRightSide
lis r10, str_P_Blur_08@ha ; Logo Blur
addi r10, r10, str_P_Blur_08@l
bla _compareString
beq scalePaneToRightSide
lis r10, str_N_HardMode_03@ha ; Hard Mode
addi r10, r10, str_N_HardMode_03@l
bla _compareString
beq scalePaneToLeftSide
lis r10, str_N_CopyRight_00@ha ; Copyright and version number
addi r10, r10, str_N_CopyRight_00@l
bla _compareString
beq scalePaneToRightSide
lis r10, str_N_BtnIn_00@ha ; Menu box
addi r10, r10, str_N_BtnIn_00@l
bla _compareString
beq scalePaneToRightSide
lis r10, str_N_1_Arrow_00@ha ; Backgrounds that have have no edges
addi r10, r10, str_N_1_Arrow_00@l
bla _compareString
beq scalePaneToLeftSide
lis r10, str_N_5_Gardian_00@ha
addi r10, r10, str_N_5_Gardian_00@l
bla _compareString
beq scalePaneToLeftSide
lis r10, str_N_2_Horse_00@ha
addi r10, r10, str_N_2_Horse_00@l
bla _compareString
beq scalePaneToLeftSide
lis r10, str_N_4_SenakaLink_00@ha
addi r10, r10, str_N_4_SenakaLink_00@l
bla _compareString
beq scalePaneToLeftSide
lis r10, str_N_3_Gake_00@ha ; Big backgrounds that have visible borders
addi r10, r10, str_N_3_Gake_00@l
bla _compareString
beq scalePaneOnYScale
lis r10, str_N_0_MainV_00@ha
addi r10, r10, str_N_0_MainV_00@l
bla _compareString
beq scalePaneOnYScale
lis r10, str_N_6_MainVE_00@ha
addi r10, r10, str_N_6_MainVE_00@l
bla _compareString
beq scalePaneOnYScale
b exitScale

scaleDemoNameEnemyPanes:
addi r5, r31, 0x80
lis r10, str_N_Capture_00@ha
addi r10, r10, str_N_Capture_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Capture_01@ha
addi r10, r10, str_N_Capture_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_In_00@ha
addi r10, r10, str_N_In_00@l
bla _compareString
beq scalePaneNormal

lis r10, str_P_BigTextSh_00@ha
addi r10, r10, str_P_BigTextSh_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_BigText_00@ha
addi r10, r10, str_P_BigText_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Light_01@ha
addi r10, r10, str_P_Light_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_BigText_01@ha
addi r10, r10, str_P_BigText_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_BigText_02@ha
addi r10, r10, str_P_BigText_02@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleLastCompletePanes:
addi r5, r31, 0x80
lis r10, str_N_Capture_00@ha
addi r10, r10, str_N_Capture_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Base_03@ha
addi r10, r10, str_P_Base_03@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Base_01@ha
addi r10, r10, str_P_Base_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_In_00@ha
addi r10, r10, str_N_In_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_In_01@ha
addi r10, r10, str_N_In_01@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleCreditPanes:
addi r5, r31, 0x80
lis r10, str_N_PicFade_00@ha
addi r10, r10, str_N_PicFade_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_PicFade_01@ha
addi r10, r10, str_N_PicFade_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Sh_00@ha
addi r10, r10, str_P_Sh_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Sh_01@ha
addi r10, r10, str_P_Sh_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_Text_01@ha
addi r10, r10, str_P_Text_01@l
bla _compareString
beq scalePaneNormal

lis r10, str_N_StaffRoll_00@ha
addi r10, r10, str_N_StaffRoll_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_PostOne_03@ha
addi r10, r10, str_N_PostOne_03@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_PostOne_02@ha
addi r10, r10, str_N_PostOne_02@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_PostOne_01@ha
addi r10, r10, str_N_PostOne_01@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_PostOne_00@ha
addi r10, r10, str_N_PostOne_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_NameTriple_00@ha
addi r10, r10, str_N_NameTriple_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_NameOne_00@ha
addi r10, r10, str_N_NameOne_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_CopyRight_00@ha
addi r10, r10, str_N_CopyRight_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleDLCCreditPanes:
addi r5, r31, 0x80
lis r10, str_N_CaptureText_00@ha
addi r10, r10, str_N_CaptureText_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_In_00@ha
addi r10, r10, str_N_In_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_P_BG_00@ha
addi r10, r10, str_P_BG_00@l
bla _compareString
beq scalePaneReverse
b exitScale

scaleThanksPanes:
addi r5, r31, 0x80
lis r10, str_N_In_00@ha
addi r10, r10, str_N_In_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Capture_00@ha
addi r10, r10, str_N_Capture_00@l
bla _compareString
beq scalePaneNormal
lis r10, str_N_Capture_01@ha
addi r10, r10, str_N_Capture_01@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleGameTitlePanes:
addi r5, r31, 0x80
lis r10, str_N_Region_01@ha
addi r10, r10, str_N_Region_01@l
bla _compareString
beq scalePaneAndKeepPosIf
lis r10, str_N_Region_01_JPja@ha
addi r10, r10, str_N_Region_01_JPja@l
bla _compareString
beq scalePaneAndKeepPosIf
b exitScale

scaleMessageTipsPanes:
lis r5, copySubPanelString@ha
addi r5, r5, copySubPanelString@l

lis r10, str_Pa_Tips_00@ha
addi r10, r10, str_Pa_Tips_00@l
bla _compareString
beq scaleMessageTipsSubTipsPanes
lis r10, str_Pa_TipsAmiibo_00@ha
addi r10, r10, str_Pa_TipsAmiibo_00@l
bla _compareString
beq scaleMessageTipsSubTipsPanes

addi r5, r31, 0x80
lis r10, str_N_DecideOut_00@ha
addi r10, r10, str_N_DecideOut_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleMessageTipsSubTipsPanes:
addi r5, r31, 0x80
lis r10, str_N_InOut_01@ha
addi r10, r10, str_N_InOut_01@l
bla _compareString
beq scalePaneNormal
b exitScale

const_EnergyMeterOffset:
.float 392

scaleEnergyMeterDLCPanes:
addi r5, r31, 0x80
lis r10, const_EnergyMeterOffset@ha
lfs f12, const_EnergyMeterOffset@l(r10)
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneNormal
b exitScale

; ------------------------------------------------------------------------------------------
; Generic methods used to scale a specific pane type

scaleRootToCenter:
addi r5, r31, 0x80 ; address to first character of the pane name that's getting loaded
lis r10, str_RootPane@ha
addi r10, r10, str_RootPane@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleInOutToCenter:
addi r5, r31, 0x80
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleTopInOutToCenter:
lis r5, copySubPanelString@ha
lbz r5, copySubPanelString@l(r5)
cmpwi r5, 0
bne exitScale
addi r5, r31, 0x80
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleTopInOutToCenterAndPos:
lis r5, copySubPanelString@ha
lbz r5, copySubPanelString@l(r5)
cmpwi r5, 0
bne exitScale
addi r5, r31, 0x80
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneAndPos
b exitScale

scaleRootScreenToLeftSide:
addi r5, r31, 0x80
lis r10, str_RootPane@ha
addi r10, r10, str_RootPane@l
bla _compareString
beq scalePaneToLeftSide
b exitScale

scaleInOutScreenToLeftSide:
addi r5, r31, 0x80
lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneToLeftSide
b exitScale

scaleRootScreenToRightSide:
addi r5, r31, 0x80
lis r10, str_RootPane@ha
addi r10, r10, str_RootPane@l
bla _compareString
beq scalePaneToRightSide
b exitScale

scaleInOutScreenToRightSide:
addi r5, r31, 0x80
lis r10, str_RootPane@ha
addi r10, r10, str_RootPane@l
bla _compareString
beq movePaneToRightSide

lis r10, str_N_InOut_00@ha
addi r10, r10, str_N_InOut_00@l
bla _compareString
beq scalePaneNormal
b exitScale

scaleAllPanes:
addi r5, r31, 0x80
lis r10, str_RootPane@ha
addi r10, r10, str_RootPane@l
bla _compareString
bne scalePaneNormal
b exitScale

; ------------------------------------------------------------------------------------------
; Methods used to scale a single pane

scalePaneAndSize:
lis r10, const_AspectRatio@ha
lfs f9, const_AspectRatio@l(r10)
lfs f0, 0x0(r27)
fmuls f0, f0, f9
stfs f0, 0x0(r27)

lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)
b exitScale

scaleOnlySize:
lis r10, const_AspectRatio@ha
lfs f9, const_AspectRatio@l(r10)
lfs f0, 0x0(r27)
fmuls f0, f0, f9
stfs f0, 0x0(r27)
b exitScale

scaleOnlySizeReverse:
lis r10, const_ReverseAspectRatio@ha
lfs f9, const_ReverseAspectRatio@l(r10)
lfs f0, 0x0(r27)
fmuls f0, f0, f9
stfs f0, 0x0(r27)
b exitScale

; Scales pane to left side
scalePaneToLeftSide:
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

lis r10, const_PaddingLeftSide@ha
lfs f9, const_PaddingLeftSide@l(r10)
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

scalePaneToLeftSideIf:
li r10, $ultrawideHUDMode
cmpwi r10, 1
beq scalePaneToLeftSide 
bne scalePaneNormal

movePaneToLeftSide:
lis r10, const_PaddingLeftSide@ha
lfs f9, const_PaddingLeftSide@l(r10)
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

movePaneToLeftSideAlt:
lis r10, const_AltPaddingLeftSide@ha
lfs f9, const_AltPaddingLeftSide@l(r10)
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

; Scales pane to right side
scalePaneToRightSide:
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

lis r10, const_PaddingRightSide@ha
lfs f9, const_PaddingRightSide@l(r10)
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

movePaneToRightSide:
lis r10, const_PaddingRightSide@ha
lfs f9, const_PaddingRightSide@l(r10)
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

movePaneToRightSideAlt:
lis r10, const_AltPaddingRightSide@ha
lfs f9, const_AltPaddingRightSide@l(r10)
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

scalePaneToRightSideAlt:
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

lis r10, const_AltPaddingRightSide@ha
lfs f9, const_AltPaddingRightSide@l(r10)
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

scalePaneReverseToLeft:
lis r10, const_ReverseAspectRatio@ha
lfs f0, const_ReverseAspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

lis r10, const_AltPaddingLeftSide@ha
lfs f9, const_AltPaddingLeftSide@l(r10)
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

scalePaneReverseToRight:
lis r10, const_ReverseAspectRatio@ha
lfs f0, const_ReverseAspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

lis r10, const_AltPaddingRightSide@ha
lfs f9, const_AltPaddingRightSide@l(r10)
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

; Divides the width of this element with the change in aspect ratio
scalePaneNormal:
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)
b exitScale

scalePaneAndPos:
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x1C(r31)
fmuls f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

scaleOnlyPos:
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x1C(r31)
fmuls f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

const_640:
.float $gameWidth/2

const_1:
.float 1.0

scalePaneAndKeepPosIf:
li r10, $ultrawideHUDMode
cmpwi r10, 1
beq scalePaneAndKeepPos 
bne scalePaneAndPos

scalePaneAndKeepPos:
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

; [XPositionOfPane] + ((1280/2 - [XPositionOfPane]) * (1-[AspectRatio])))
lis r10, const_0@ha
lfs f11, const_0@l(r10)
lfs f9, 0x1C(r31)
fcmpu f9, f11
lis r10, const_640@ha
lfs f0, const_640@l(r10)
lfs f9, 0x1C(r31)
bge .+0x08
fsubs f9, f11, f9
fsubs f0, f0, f9
lis r10, const_1@ha
lfs f13, const_1@l(r10)
lis r10, const_AspectRatio@ha
lfs f9, const_AspectRatio@l(r10)
fsubs f13, f13, f9
fmuls f0, f0, f13
lfs f9, 0x1C(r31)
bge .+0x08
fsubs f9, f11, f9
fadds f0, f9, f0
bge .+0x08
fsubs f0, f11, f0
stfs f0, 0x1C(r31)
b exitScale

; Pass custom pos as f12
scalePaneAndKeepCustomPosIf:
li r10, $ultrawideHUDMode
cmpwi r10, 1
beq scalePaneAndKeepCustomPos 
bne scalePaneAndCustomPos

scalePaneAndKeepCustomPos:
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

; [XPositionOfPane] + ((1280/2 - [XPositionOfPane]) * (1-[AspectRatio])))
lis r10, const_0@ha
lfs f11, const_0@l(r10)
lfs f9, 0x1C(r31)
fadds f9, f9, f12
fcmpu f9, f11
lis r10, const_640@ha
lfs f0, const_640@l(r10)
lfs f9, 0x1C(r31)
fadds f9, f9, f12
bge .+0x08
fsubs f9, f11, f9
fsubs f0, f0, f9
lis r10, const_1@ha
lfs f13, const_1@l(r10)
lis r10, const_AspectRatio@ha
lfs f9, const_AspectRatio@l(r10)
fsubs f13, f13, f9
fmuls f0, f0, f13
lfs f9, 0x1C(r31)
fadds f9, f9, f12
bge .+0x08
fsubs f9, f11, f9
fadds f0, f9, f0
bge .+0x08
fsubs f0, f11, f0
fsubs f0, f0, f12
stfs f0, 0x1C(r31)
b exitScale

# ; [XPositionOfPane] + ((1280/2 - [XPositionOfPane]) * (1-[AspectRatio])))
# lis r10, const_640@ha
# lfs f0, const_640@l(r10) ; (1280/2)
# lfs f9, 0x1C(r31)
# fadds f9, f9, f12        ; Create XPositionOfPane (add position of current pane + custom position of parent pane)
# fsubs f0, f0, f9         ; (1280/2) - XPositionOfPane

# lis r10, const_1@ha
# lfs f13, const_1@l(r10)
# lis r10, const_AspectRatio@ha
# lfs f9, const_AspectRatio@l(r10)
# fsubs f13, f13, f9       ; (1-[AspectRatio])

# fmuls f0, f0, f13        ; ((1280/2) - XPositionOfPane) * (1-[AspectRatio])

# lfs f9, 0x1C(r31)
# fadds f9, f9, f12        ; [XPositionOfPane]

# fadds f0, f9, f0         ;  [XPositionOfPane] + ((1280/2 - [XPositionOfPane]) * (1-[AspectRatio])))
# fsubs f0, f0, f12

; Explanation: The current position is 3x as big as it should be (let's take 3x at 48:9), so to correct that you scale it back by 
; XPositionOfPane*(1-AspectRatio)
scalePaneAndCustomPos:
lis r10, const_AspectRatio@ha
lfs f0,  const_AspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

lis r10, const_1@ha
lfs f9, const_1@l(r10)
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
fsubs f0, f9, f0
lfs f9, 0x1C(r31)
fsubs f9, f9, f12
fmuls f0, f9, f0
stfs f0, 0x1C(r31)
b exitScale

moveKeepPos:
lis r10, const_0@ha
lfs f11, const_0@l(r10)
lfs f9, 0x1C(r31)
fcmpu f9, f11
lis r10, const_640@ha
lfs f0, const_640@l(r10)
lfs f9, 0x1C(r31)
bge .+0x08
fsubs f9, f11, f9
fsubs f0, f0, f9
lis r10, const_1@ha
lfs f13, const_1@l(r10)
lis r10, const_AspectRatio@ha
lfs f9, const_AspectRatio@l(r10)
fsubs f13, f13, f9
fmuls f0, f0, f13
lfs f9, 0x1C(r31)
bge .+0x08
fsubs f9, f11, f9
fadds f0, f9, f0
bge .+0x08
fsubs f0, f11, f0
stfs f0, 0x1C(r31)
b exitScale

; Pass the scale in f0 and position in f12
scalePaneWithCustomPosAndScale:
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

lfs f9, 0x1C(r31)
fsubs f0, f9, f12
stfs f12, 0x1C(r31)
b exitScale

movePaneToCustomPos:
lfs f9, 0x1C(r31)
fsubs f0, f9, f12
stfs f12, 0x1C(r31)
b exitScale

scalePaneHalf:
lis r10, const_AspectRatioHalf@ha
lfs f0, const_AspectRatioHalf@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)
b exitScale

scalePaneReverse:
lis r10, const_ReverseAspectRatio@ha
lfs f0, const_ReverseAspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)
b exitScale

scalePaneAndPosReverse:
lis r10, const_ReverseAspectRatio@ha
lfs f0, const_ReverseAspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

lis r10, const_ReverseAspectRatio@ha
lfs f0, const_ReverseAspectRatio@l(r10)
lfs f9, 0x1C(r31)
fmuls f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

scalePaneReverse2x:
lis r10, const_ReverseAspectRatio2x@ha
lfs f0, const_ReverseAspectRatio2x@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)
b exitScale

scalePaneOnYScale:
lis r10, const_ReverseAspectRatio@ha
lfs f0, const_ReverseAspectRatio@l(r10)
lfs f9, 0x38(r31)
fmuls f0, f0, f9
stfs f0, 0x38(r31)
b exitScale

scalePaneOnYScaleReverse:
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x38(r31)
fmuls f0, f0, f9
stfs f0, 0x38(r31)
b exitScale

scaleReversePaneToRightSide:
lis r10, const_ReverseAspectRatio2x@ha
lfs f0, const_ReverseAspectRatio2x@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)

lis r10, const_AltPaddingRightSide@ha
lfs f9, const_AltPaddingRightSide@l(r10)
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)
b exitScale

exitScale:
mtlr r0
lwz r0, 0x24(r30)
blr

0x03C496B8 = bla _scalePaneGUI


; ------------------------------------------------------------------------------------------
; Specific code required for switching the positions of the minimap and the AppMap

const_0:
.float 0.0

mapXPositionAddr:
.int 0

scaleMapAndStore:
; scale pane and move to right on initialization
lis r10, const_AspectRatio@ha
lfs f0, const_AspectRatio@l(r10)
lfs f9, 0x34(r31)
fmuls f0, f0, f9
stfs f0, 0x34(r31)
; if the ultrawide mode is 1 (edge HUD), add padding so that the map gets initialized at the right edge
li r10, $ultrawideHUDMode
cmpwi r4, 1
lis r10, const_0@ha
lfs f9, const_0@l(r10)
bne noMapPadding
lis r10, const_PaddingRightSide@ha
lfs f9, const_PaddingRightSide@l(r10)
noMapPadding:
lfs f0, 0x1C(r31)
fadds f0, f0, f9
stfs f0, 0x1C(r31)

addi r5, r31, 0x1C
lis r10, mapXPositionAddr@ha
stw r5, mapXPositionAddr@l(r10)
b exitScale

setMapPosition:
li r4, $ultrawideHUDMode
cmpwi r4, 0
beq setNothing

li r4, $ultrawideHUDMode ; if ultramode is centered, keep the map centered
cmpwi r4, 2
beq setToMap

cmpwi r31, 0
beq setToMinimap

setToMap:
cmpwi r3, 0
lis r3, const_0@ha
lfs f12, const_0@l(r3)
lis r3, mapXPositionAddr@ha
lwz r3, mapXPositionAddr@l(r3)
stfs f12, 0(r3)
blr
setToMinimap:
cmpwi r3, 0
lis r3, const_PaddingRightSide@ha
lfs f12, const_PaddingRightSide@l(r3)
lis r3, mapXPositionAddr@ha
lwz r3, mapXPositionAddr@l(r3)
stfs f12, 0(r3)
blr
setNothing:
cmpwi r3, 0
blr

0x02F60204 = bla setMapPosition

; ------------------------------------------------------------------------------------------
; Specific code required for scaling a specific effect called PaneBasedProjection used on the weather panes, which breaks while scaling

proj_PosX:
.float 0.0

proj_PosY:
.float 0.0

proj_ScaleX:
.float ($gameWidth/$gameHeight)/($width/$height)

proj_ScaleY:
.float 1.0

proj_flags:
.int 0x01000000

; other registers used: r11, r9, r12, r11

; empty registers: r5, r6, r8, r0, r9, r11
setPaneBasedProjection:
stw r0, 0x10(r4)

mr r6, r12
mr r8, r10
mflr r0

; if ultrawide mode is set to 0, return early
li r10, $ultrawideHUDMode
cmpwi r10, 0
beq exitPaneBasedProjection

lis r10, copySubPanelString@ha
addi r10, r10, copySubPanelString@l
lis r5, str_Pa_Weather_00@ha
addi r5, r5, str_Pa_Weather_00@l
bla _compareString
bne exitPaneBasedProjection

# lis r11, proj_PosX@ha
# lfs f3, proj_PosX@l(r11)
# stfs f3, 0x0(r4)
# lis r11, proj_PosY@ha
# lfs f3, proj_PosY@l(r11)
# stfs f3, 0x4(r4)
lis r11, proj_ScaleX@ha
lfs f3, proj_ScaleX@l(r11)
stfs f3, 0x8(r4)
# lis r11, proj_ScaleY@ha
# lfs f3, proj_ScaleY@l(r11)
# stfs f3, 0xC(r4)
# lis r11, proj_flags@ha
# lwz r11, proj_flags@l(r11)
# stw r11, 0x10(r4)

exitPaneBasedProjection:
mtlr r0
mr r12, r6
mr r10, r8
blr


0x03C56CD0 = bla setPaneBasedProjection


; ------------------------------------------------------------------------------------------

_createNewScreenHook:
; Copy screen name to buffer
lis r11, copyScreenStringLen@ha
lwz r11, copyScreenStringLen@l(r11)
lis r12, copyScreenString@ha
addi r12, r12, copyScreenString@l

copyNameLoop:
lbzx r10, r24, r11
stbx r10, r12, r11
addi r11, r11, -1
cmpwi r11, -1
bne copyNameLoop

; Erase SubPanel string when a new screen got loaded
lis r11, copySubPanelStringLen@ha
lwz r11, copySubPanelStringLen@l(r11)
lis r12, copySubPanelString@ha
addi r12, r12, copySubPanelString@l

eraseSubPanelLoop:
li r10, 0
stbx r10, r12, r11
addi r11, r11, -1
cmpwi r11, -1
bne eraseSubPanelLoop

li r10, $enableUltrawideDebugLogging
cmpwi r10, 1
blt skipLayoutFileLogging

crxor 4*cr1+eq, 4*cr1+eq, 4*cr1+eq
lis r3, newLineFormatScreen@ha
addi r3, r3, newLineFormatScreen@l
lis r4, copyScreenString@ha
addi r4, r4, copyScreenString@l
lis r5, newLineCharacter@ha
lwz r5, newLineCharacter@l(r5)
mflr r10
bl import.coreinit.OSReport
mtlr r10
mr r3, r11

skipLayoutFileLogging:

lwz r11, 0xC(r30)
blr

0x03A3EDC4 = bla _createNewScreenHook

; ------------------------------------------------------------------------------------------

; compares the string from r5 and r10
; r5 is untouched, so use that to do multiple comparisons
; other registers used: r11, r9, r12, r11
_compareString:
mr r11, r5

startLoop:
lbz r9, 0(r11)
lbz r12, 0(r10)
 
cmpwi r9, 0
bne checkForMatch
cmpwi r12, 0
bne checkForMatch
li r10, 1
cmpwi r10, 1
blr
 
checkForMatch:
cmpw r9, r12
bne noMatch
addi r11, r11, 1
addi r10, r10, 1
b startLoop
 
noMatch:
li r10, 0
cmpwi r10, 1
blr