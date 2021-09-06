#include "layer.h"
#include "memory.h"

std::atomic<SWAP_SIDE> currSwapSide = SWAP_SIDE::LEFT;
std::atomic<RENDER_STATUS> currRenderStatus = RENDER_STATUS::WAITING;

typedef void (*osLib_registerHLEFunctionType)(const char* libraryName, const char* functionName, void(*osFunction)(PPCInterpreter_t* hCPU));

struct graphicPackData {
	int32_t swappedFlipSideSetting;
	float eyeSeparation;
	float headPositionSensitivitySetting;
	float heightPositionOffsetSetting;
	float hudScaleSetting;
	float menuScaleSetting;
	float zoomOutLevel;
	// input
	float oldPosX;
	float oldPosY;
	float oldPosZ;
	float oldTargetX;
	float oldTargetY;
	float oldTargetZ;
	float oldFOV;
	// output
	float newPosX;
	float newPosY;
	float newPosZ;
	float newTargetX;
	float newTargetY;
	float newTargetZ;
	float newRotX;
	float newRotY;
	float newRotZ;
	float newFOV;
	float newAspectRatio;
};

std::mutex currGraphicPackSettingsLock;
graphicPackData currGraphicPackSettingsData = {};

graphicPackData cameraGetGraphicPackSettings() {
	graphicPackData retCopy;
	scoped_lock l(currGraphicPackSettingsLock);
	retCopy = currGraphicPackSettingsData;
	return retCopy;
}

void cameraInitialize() {
	osLib_registerHLEFunctionType osLib_registerHLEFunction = (osLib_registerHLEFunctionType)GetProcAddress(GetModuleHandleA("Cemu.exe"), "osLib_registerHLEFunction");

	osLib_registerHLEFunction("coreinit", "cameraHookFrame", &cameraHookFrame);
	logPrint("Registered cameraHookFrame function to Cemu so that it can be called in the patch");

	osLib_registerHLEFunction("coreinit", "cameraHookUpdate", &cameraHookUpdate);
	logPrint("Registered cameraHookUpdate function to Cemu so that it can be called in the patch");

	osLib_registerHLEFunction("coreinit", "cameraHookInterface", &cameraHookInterface);
	logPrint("Registered cameraHookInterface function to Cemu so that it can be called in the patch");
}

void swapGraphicPackDataEndianness(graphicPackData* data) {
	swapEndianness(data->swappedFlipSideSetting);
	swapEndianness(data->eyeSeparation);
	swapEndianness(data->headPositionSensitivitySetting);
	swapEndianness(data->heightPositionOffsetSetting);
	swapEndianness(data->hudScaleSetting);
	swapEndianness(data->menuScaleSetting);
	swapEndianness(data->zoomOutLevel);
	swapEndianness(data->oldPosX);
	swapEndianness(data->oldPosY);
	swapEndianness(data->oldPosZ);
	swapEndianness(data->oldTargetX);
	swapEndianness(data->oldTargetY);
	swapEndianness(data->oldTargetZ);
	swapEndianness(data->oldFOV);
	swapEndianness(data->newPosX);
	swapEndianness(data->newPosY);
	swapEndianness(data->newPosZ);
	swapEndianness(data->newTargetX);
	swapEndianness(data->newTargetY);
	swapEndianness(data->newTargetZ);
	swapEndianness(data->newRotX);
	swapEndianness(data->newRotY);
	swapEndianness(data->newRotZ);
	swapEndianness(data->newFOV);
	swapEndianness(data->newAspectRatio);
};

uint64_t framesSinceLastCameraUpdate = 0;

void cameraHookInterface(PPCInterpreter_t* hCPU) {
	hCPU->instructionPointer = hCPU->gpr[8];

	char screenNamePrintBuffer[200];
	snprintf(screenNamePrintBuffer, sizeof(screenNamePrintBuffer), "Created new \"%s\" screen with an ID of 0x%08x", (const char*)(memoryBaseAddress + hCPU->gpr[7]), hCPU->gpr[5]);
	logPrint(screenNamePrintBuffer);

	ScreenId newScreenId = (ScreenId)hCPU->gpr[5];
	switch (newScreenId) {
		case ScreenId::MainScreen_00:
		case ScreenId::Message3D_00:
		case ScreenId::AppHome_00:
		case ScreenId::MainScreenMS_00:
		case ScreenId::MainDungeon_00:
		case ScreenId::AppMenuBtn_00:
		case ScreenId::PickUp_00:
		case ScreenId::DoCommand_00:
			writeMemoryBE(0x102F8CC8, cameraGetGraphicPackSettings().hudScaleSetting);
			break;
		default:
			writeMemoryBE(0x102F8CC8, 1.0f);
			break;
	}

	if (newScreenId == ScreenId::LoadSaveIcon_00 && cameraIsInGame()) {
		writeMemoryBE(0x102F8CC8, cameraGetGraphicPackSettings().hudScaleSetting);
	}
	else if (newScreenId == ScreenId::LoadSaveIcon_00) {
		writeMemoryBE(0x102F8CC8, 1.0f);
	}

	uint32_t screenID = hCPU->gpr[5];
}

void cameraHookFrame(PPCInterpreter_t* hCPU) {
	//logPrint("BotW rendered a frame");
	hCPU->instructionPointer = hCPU->sprNew.LR;

	uint32_t graphicPackDataOffset = hCPU->gpr[5];

	scoped_lock l(currGraphicPackSettingsLock);
	readMemory(graphicPackDataOffset, &currGraphicPackSettingsData);
	swapGraphicPackDataEndianness(&currGraphicPackSettingsData);

	framesSinceLastCameraUpdate++;
}
int thing = 0;
void cameraHookUpdate(PPCInterpreter_t* hCPU) {
	logPrint(std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - thing));
	thing = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	//logPrint("Updated the camera positions");
	hCPU->instructionPointer = hCPU->gpr[7]; // r7 will have the instruction that should be returned to

	framesSinceLastCameraUpdate = 0;

	// r30 has the graphicPackData offset in memory
	uint32_t graphicPackDataOffset = hCPU->gpr[30];

	graphicPackData inputData = {};
	readMemory(graphicPackDataOffset, &inputData);
	swapGraphicPackDataEndianness(&inputData);

	XrView* currView = currSwapSide == SWAP_SIDE::RIGHT ? &leftView : &rightView;
	glm::fvec3 eyePos(currView->pose.position.x, currView->pose.position.y, currView->pose.position.z);

	glm::fvec3 hmdPos((leftView.pose.position.x + rightView.pose.position.x) / 2, (leftView.pose.position.y + rightView.pose.position.y) / 2, (leftView.pose.position.z + rightView.pose.position.z) / 2);

	glm::fvec3 oldPosition(inputData.oldPosX, inputData.oldPosY, inputData.oldPosZ);
	glm::fvec3 oldTarget(inputData.oldTargetX, inputData.oldTargetY, inputData.oldTargetZ);
	float originalCameraDistance = glm::distance(oldPosition, oldTarget);

	glm::fvec3 forwardVector = oldTarget - oldPosition;
	forwardVector = glm::normalize(forwardVector);
	glm::fquat lookAtQuat = glm::quatLookAtRH(forwardVector, {0.0, 1.0, 0.0});
	glm::fquat hmdQuat = glm::fquat(currView->pose.orientation.w, currView->pose.orientation.x, currView->pose.orientation.y, currView->pose.orientation.z);

	glm::fquat combinedQuat = lookAtQuat * hmdQuat;
	glm::fmat3 combinedMatrix = glm::toMat3(hmdQuat);

	glm::fvec3 rotatedHmdPos = glm::toMat3(combinedQuat) * eyePos;

	//TODO: Add graphic pack check for first person
	if (true)
	{
		readMemoryBE(0x113444F0 + 0x50, &inputData.newPosX);
		readMemoryBE(0x113444F0 + 0x54, &inputData.newPosY);
		readMemoryBE(0x113444F0 + 0x58, &inputData.newPosZ);

		swapEndianness(inputData.newPosX);
		swapEndianness(inputData.newPosY);
		swapEndianness(inputData.newPosZ);

		if (inputData.newPosX != 0 && inputData.newPosY != 0 && inputData.newPosZ != 0)
		{
			inputData.newPosX += (hmdPos.x * inputData.headPositionSensitivitySetting) - (hmdPos.x - eyePos.x);
			inputData.newPosY += (hmdPos.y * inputData.headPositionSensitivitySetting) - (hmdPos.y - eyePos.y);
			inputData.newPosZ += (hmdPos.z * inputData.headPositionSensitivitySetting) - (hmdPos.z - eyePos.z);
		}
	}
	else
	{
		inputData.newPosX = inputData.oldPosX + (hmdPos.x * inputData.headPositionSensitivitySetting) + (hmdPos.x - eyePos.x);
		inputData.newPosY = inputData.oldPosY + (hmdPos.y * inputData.headPositionSensitivitySetting) + (hmdPos.y - eyePos.y);
		inputData.newPosZ = inputData.oldPosZ + (hmdPos.z * inputData.headPositionSensitivitySetting) + (hmdPos.z - eyePos.z);
	}

	logPrint(std::to_string(inputData.newPosX) + " | " + std::to_string(inputData.newPosY) + " | " + std::to_string(inputData.newPosZ));

	inputData.newTargetX = inputData.newPosX + ((combinedMatrix[2][0] * -1.0f) * originalCameraDistance);
	inputData.newTargetY = inputData.newPosY + ((combinedMatrix[2][1] * -1.0f) * originalCameraDistance);
	inputData.newTargetZ = inputData.newPosZ + ((combinedMatrix[2][2] * -1.0f) * originalCameraDistance);

	inputData.newRotX = combinedMatrix[1][0];
	inputData.newRotY = combinedMatrix[1][1];
	inputData.newRotZ = combinedMatrix[1][2];

	// Set FOV
	float horizontalFOV = (leftView.fov.angleRight - leftView.fov.angleLeft);
	float verticalFOV = (leftView.fov.angleUp - leftView.fov.angleDown);



	//float horizontalFullFovInRadians = 2.0f * atanf(combinedTanHalfFovHorizontal);

	float leftHalfFOV = glm::degrees(leftView.fov.angleLeft);
	float rightHalfFOV = glm::degrees(leftView.fov.angleRight);
	float upHalfFOV = glm::degrees(leftView.fov.angleUp);
	float downHalfFOV = glm::degrees(leftView.fov.angleDown);

	float horizontalHalfFOV = (abs(leftView.fov.angleLeft) + abs(leftView.fov.angleRight)) * 0.5;
	float verticalHalfFOV = (abs(leftView.fov.angleUp) + abs(leftView.fov.angleDown)) * 0.5;

	//float horizontalFullFovInRadians = 2.0f * atanf(combinedTanHalfFovHorizontal);

	float diagonalFOV = sqrtf(horizontalFOV * horizontalFOV + verticalFOV * verticalFOV);
	float horizontalFullFovInRadians = 2.0f * atanf(horizontalHalfFOV);
	float aspectRatioCompare = tanf(diagonalFOV / 2.0f);
	float aspectRatio = horizontalHalfFOV / verticalHalfFOV; //horizontalHalfFOV / verticalHalfFOV;


	//logPrint(std::string("Horizontal FOV in degrees is ") + std::to_string(glm::degrees(horizontalFOV)) + std::string(" and in radians (as used by OpenXR) ") + std::to_string(horizontalFOV));
	//logPrint(std::string("Vertical FOV in degrees is ") + std::to_string(glm::degrees(verticalFOV)) + std::string(" and in radians (as used by OpenXR) ") + std::to_string(verticalFOV));
	//logPrint(std::string("Diagonal FOV in degrees is ") + std::to_string(glm::degrees(diagonalFOV)) + std::string(" and in radians (as used by OpenXR) ") + std::to_string(diagonalFOV));
	//logPrint(std::string("Calculated aspect ratio is ") + std::to_string(aspectRatio));

	inputData.newFOV = horizontalFullFovInRadians;
	inputData.newAspectRatio = aspectRatio;

	swapGraphicPackDataEndianness(&inputData);
	writeMemory(graphicPackDataOffset, inputData);
}

float cameraGetMenuZoom() {
	return cameraGetGraphicPackSettings().menuScaleSetting;
}

bool cameraUseSwappedFlipMode() {
	return cameraGetGraphicPackSettings().swappedFlipSideSetting == 1;
}

float cameraGetEyeSeparation() {
	return cameraGetGraphicPackSettings().eyeSeparation;
}

float cameraGetZoomOutLevel() {
	return cameraGetGraphicPackSettings().zoomOutLevel;
}

bool cameraIsInGame() {
	return framesSinceLastCameraUpdate < 2;
}