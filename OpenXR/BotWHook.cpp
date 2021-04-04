#include "pch.h"
#include "BotWHook.h"


// Development Mode Selector
#define GFX_PACK_PASSTHROUGH 0      // Used to make this app as a way to asynchronously feed headset data to the graphic pack which will then calculate the headset position each frame.
#define APP_CALC_CONVERT 1          // Used as an intermediary for converting the library-dependent code into native graphic pack code.
#define APP_CALC_LIBRARY 2          // Used for easily testing code using libraries.

#define CALC_MODE APP_CALC_LIBRARY


HWND cemuHWND = NULL;
DWORD cemuProcessID = NULL;
HANDLE cemuHandle = NULL;
std::filesystem::path cemuPath = "";

uint64_t baseAddress = NULL;
uint64_t vrAddress = NULL;

int retryFindingCemu = 0;

float reverseFloatEndianess(float inFloat) {
    float retVal;
    char* floatToConvert = (char*)&inFloat;
    char* returnFloat = (char*)&retVal;

    // swap the bytes into a temporary buffer
    returnFloat[0] = floatToConvert[3];
    returnFloat[1] = floatToConvert[2];
    returnFloat[2] = floatToConvert[1];
    returnFloat[3] = floatToConvert[0];

    return retVal;
}


struct inputDataBuffer {
    HOOK_MODE status;
    XrQuaternionf headsetQuaternion;
    XrVector3f headsetPosition;
};

struct copyDataBuffer {
    XrVector3f pos;
    XrVector3f target;
    XrVector3f rot;
};

void reverseInputBufferEndianess(inputDataBuffer* data) {
    data->headsetQuaternion.x = reverseFloatEndianess(data->headsetQuaternion.x);
    data->headsetQuaternion.y = reverseFloatEndianess(data->headsetQuaternion.y);
    data->headsetQuaternion.z = reverseFloatEndianess(data->headsetQuaternion.z);
    data->headsetQuaternion.w = reverseFloatEndianess(data->headsetQuaternion.w);
    data->headsetPosition.x = reverseFloatEndianess(data->headsetPosition.x);
    data->headsetPosition.y = reverseFloatEndianess(data->headsetPosition.y);
    data->headsetPosition.z = reverseFloatEndianess(data->headsetPosition.z);
}

void reverseCopyBufferEndianess(copyDataBuffer* data) {
    data->pos.x = reverseFloatEndianess(data->pos.x);
    data->pos.y = reverseFloatEndianess(data->pos.y);
    data->pos.z = reverseFloatEndianess(data->pos.z);
    data->target.x = reverseFloatEndianess(data->target.x);
    data->target.y = reverseFloatEndianess(data->target.y);
    data->target.z = reverseFloatEndianess(data->target.z);
    data->rot.x = reverseFloatEndianess(data->rot.x);
    data->rot.y = reverseFloatEndianess(data->rot.y);
    data->rot.z = reverseFloatEndianess(data->rot.z);
}

bool cemuFullscreen = false;
bool leftSideRender = true;
HOOK_MODE hookStatus;
inputDataBuffer inputCamera;
copyDataBuffer copyCamera;

BOOL CALLBACK findCemuWindowCallback(HWND hWnd, LPARAM lparam) {
    UNUSED(lparam);

    if (baseAddress != NULL && vrAddress != NULL)
        return TRUE;

    int windowTextLength = GetWindowTextLengthA(hWnd);
    std::string windowText;
    windowText.resize((size_t)windowTextLength + 1);
    GetWindowTextA(hWnd, windowText.data(), windowTextLength);

    if (!windowText.empty() && windowText.size() >= sizeof("Cemu ") && windowText.compare(0, sizeof("Cemu ") - 1, "Cemu ") == 0) {
        GetWindowThreadProcessId(hWnd, &cemuProcessID);
        cemuHandle = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ | PROCESS_VM_WRITE, FALSE, cemuProcessID);

        CHAR filePath[MAX_PATH];
        GetModuleFileNameExA(cemuHandle, NULL, filePath, MAX_PATH);

        std::filesystem::path tempPath = std::filesystem::path(filePath);

        if (tempPath.filename().string() != "Cemu.exe") {
            cemuProcessID = NULL;
            cemuHandle = NULL;
            tempPath.clear();
            return TRUE;
        }

        cemuPath = tempPath;
        cemuHWND = hWnd;

        tempPath.remove_filename();
        tempPath += "log.txt";

        std::ifstream logFile(tempPath, std::ios_base::in);
        std::stringstream logFileStream;
        while (logFile >> logFileStream.rdbuf());

        logFile.close();

        std::string line;
        bool runningTitle = false;

        while (std::getline(logFileStream, line)) {
            auto baseMemoryPos = line.find("Init Wii U memory space (base: ");
            auto vrMemoryPos = line.find("Applying patch group 'BotW_BetterVR_V208' (Codecave: ");
            if (baseMemoryPos != std::string::npos) {
                baseAddress = std::stoull(line.substr(baseMemoryPos + sizeof("Init Wii U memory space (base: ") - 1), nullptr, 16);
            }
            else if (vrMemoryPos != std::string::npos) {
                vrAddress = std::stoull(line.substr(vrMemoryPos + sizeof("Applying patch group 'BotW_BetterVR_V208' (Codecave: ") - 1), nullptr, 16);
            }
            else if (line.find("Applying patch group 'BotW_BetterVR_V208' (Codecave: ") != std::string::npos) {
                runningTitle = true;
            }
        }

        if (runningTitle && baseAddress != NULL && vrAddress == NULL) {
            MessageBoxA(NULL, "BetterVR graphic pack isn't enabled!", "You need to have the BetterVR graphic pack enabled to enable the VR from working.", MB_OK | MB_ICONWARNING);
        }
    }
    return TRUE;
}

void InitializeCemuHooking() {
    EnumWindows(findCemuWindowCallback, NULL);
    if (baseAddress != NULL || cemuProcessID != NULL) {
        return;
    } else {
        retryFindingCemu = 20; // Every 10 seconds it can recheck for Cemu
    }
}

int countdownPrint = 0;
constexpr float const_HeadPositionalMovementSensitivity = 1.0f;

void SetBotWPositions(XrView leftScreen, XrView rightScreen) {
    UNUSED(leftScreen);
    UNUSED(rightScreen);

    leftSideRender = !leftSideRender;
    XrPosef middleScreen = leftSideRender ? leftScreen.pose : rightScreen.pose;
    //XrPosef middleScreen = xr::math::Pose::Slerp(leftScreen.pose, rightScreen.pose, 0.5);

    if (baseAddress != NULL && vrAddress != NULL) {
        DWORD cemuExitCode = NULL;
        if (GetExitCodeProcess(cemuHandle, &cemuExitCode) == 0 || cemuExitCode != STILL_ACTIVE) {
            cemuHWND = NULL;
            cemuProcessID = NULL;
            cemuHandle = NULL;
            cemuPath = "";
            baseAddress = NULL;
            vrAddress = NULL;
            retryFindingCemu = 0;
            hookStatus = HOOK_MODE::DISABLED;
            cemuFullscreen = false;
            return;
        }

        SIZE_T writtenSize = 0;
#if CALC_MODE == GFX_PACK_PASSTHROUGH
        ReadProcessMemory(cemuHandle, (void*)(baseAddress + vrAddress), (void*)&hookStatus, sizeof(HOOK_MODE), &writtenSize);
        inputCamera = {};

        if (hookStatus == HOOK_MODE::DISABLED) {
            cemuHWND = NULL;
            cemuProcessID = NULL;
            cemuHandle = NULL;
            cemuPath = "";
            baseAddress = NULL;
            vrAddress = NULL;
            retryFindingCemu = 0;
            cemuFullscreen = false;
            return;
        }
        else if (hookStatus == HOOK_MODE::GFX_PACK_ENABLED || hookStatus == HOOK_MODE::BOTH_ENABLED_GFX_CALC) {
            inputCamera.status = HOOK_MODE::BOTH_ENABLED_GFX_CALC;
            inputCamera.headsetQuaternion.x = middleScreen.orientation.x;
            inputCamera.headsetQuaternion.y = middleScreen.orientation.y;
            inputCamera.headsetQuaternion.z = middleScreen.orientation.z;
            inputCamera.headsetQuaternion.w = middleScreen.orientation.w;
            inputCamera.headsetPosition.x = middleScreen.position.x;
            inputCamera.headsetPosition.y = middleScreen.position.y;
            inputCamera.headsetPosition.z = middleScreen.position.z;
        }

        reverseInputBufferEndianess(&inputCamera);
        WriteProcessMemory(cemuHandle, (void*)(baseAddress + vrAddress), (const void*)&inputCamera, sizeof(inputDataBuffer), &writtenSize);
#elif CALC_MODE == APP_CALC_CONVERT || CALC_MODE == APP_CALC_LIBRARY
        ReadProcessMemory(cemuHandle, (void*)(baseAddress + vrAddress), (void*)&hookStatus, sizeof(HOOK_MODE), &writtenSize);

        ReadProcessMemory(cemuHandle, (void*)(baseAddress + vrAddress + sizeof(inputDataBuffer)), (void*)&copyCamera, sizeof(copyDataBuffer) - sizeof(XrVector3f), &writtenSize);
        reverseCopyBufferEndianess(&copyCamera);

        if (hookStatus == HOOK_MODE::DISABLED) {
            cemuHWND = NULL;
            cemuProcessID = NULL;
            cemuHandle = NULL;
            cemuPath = "";
            baseAddress = NULL;
            vrAddress = NULL;
            retryFindingCemu = 0;
            cemuFullscreen = false;
            return;
        }
        else if (copyCamera.pos.x == 0.0f || copyCamera.pos.y == 0.0f || copyCamera.pos.z == 0.0f) {
            hookStatus = HOOK_MODE::BOTH_ENABLED_PRECALC;
            reverseCopyBufferEndianess(&copyCamera);
            WriteProcessMemory(cemuHandle, (void*)(baseAddress + vrAddress + sizeof(inputDataBuffer) + sizeof(copyDataBuffer) - sizeof(XrVector3f)), (const void*)&copyCamera, sizeof(copyDataBuffer), &writtenSize);
        }
        else if (hookStatus == HOOK_MODE::GFX_PACK_ENABLED || hookStatus == HOOK_MODE::BOTH_ENABLED_PRECALC) {
            hookStatus = HOOK_MODE::BOTH_ENABLED_PRECALC;
#if CALC_MODE == APP_CALC_CONVERT
            // Define the "global" variables
            float oldTargetX = copyCamera.target.x;
            float oldTargetY = copyCamera.target.y;
            float oldTargetZ = copyCamera.target.z;
            float oldPosX = copyCamera.pos.x;
            float oldPosY = copyCamera.pos.y;
            float oldPosZ = copyCamera.pos.z;

            float hmdQuatX = middleScreen.orientation.x;
            float hmdQuatY = middleScreen.orientation.y;
            float hmdQuatZ = middleScreen.orientation.z;
            float hmdQuatW = middleScreen.orientation.w;
            float hmdPosX = middleScreen.position.x;
            float hmdPosY = middleScreen.position.y;
            float hmdPosZ = middleScreen.position.z;

            float newTargetX = 0.0f;
            float newTargetY = 0.0f;
            float newTargetZ = 0.0f;
            float newPosX = 0.0f;
            float newPosY = 0.0f;
            float newPosZ = 0.0f;
            float newRotX = 0.0f;
            float newRotY = 0.0f;
            float newRotZ = 0.0f;

            float setting_heightPositionOffset = 0.8f;
            float setting_headPositionSensitivity = 2.0f;

            // Lambda functions
            struct Vec3 {
                float x;
                float y;
                float z;
            };

            auto squareRoot = [](float n) -> float {
                float x = n;
                x = 0.5 * (x + (n / x));
                x = 0.5 * (x + (n / x));
                x = 0.5 * (x + (n / x));
                x = 0.5 * (x + (n / x));
                x = 0.5 * (x + (n / x));
                x = 0.5 * (x + (n / x));
                x = 0.5 * (x + (n / x));
                return x;
            };

            auto crossProduct = [](Vec3 a, Vec3 b) -> Vec3 {
                Vec3 retVec;
                retVec.x = a.y * b.z - b.y * a.z;
                retVec.y = a.z * b.x - b.z * a.x;
                retVec.z = a.x * b.y - b.x * a.y;
                return retVec;
            };

            auto multiplyVector = [](Vec3 a, float multiplier) -> Vec3 {
                return {
                    a.x * multiplier,
                    a.y * multiplier,
                    a.z * multiplier
                };
            };

            auto normalize = [&](Vec3* a) {
                float vectorLength = squareRoot(a->x*a->x + a->y*a->y + a->z*a->z);
                a->x = a->x / vectorLength;
                a->y = a->y / vectorLength;
                a->z = a->z / vectorLength;
            };


            // Check if calculation should be done
            if (oldTargetX == 0.0f && oldTargetY == 0.0f && oldTargetZ == 0.0f) {
                newPosX = oldPosX;
                newPosY = oldPosY;
                newPosZ = oldPosZ;
                newTargetX = oldTargetX;
                newTargetY = oldTargetY;
                newTargetZ = oldTargetZ;
                newRotX = 0.0f;
                newRotY = 1.0f;
                newRotZ = 0.0f;
                return;
            }

            // Calculate direction vector by subtraction
            Vec3 directionVector = { oldTargetX - oldPosX, oldTargetY - oldPosY, oldTargetZ - oldPosZ };
            normalize(&directionVector);

            Vec3 viewMatrix[3];
            viewMatrix[2].x = -directionVector.x;
            viewMatrix[2].y = -directionVector.y;
            viewMatrix[2].z = -directionVector.z;

            // Cross product to get right row
            Vec3 lookUpVector = {0.0f, 1.0f, 0.0f};
            Vec3 rightVector = crossProduct(lookUpVector, viewMatrix[2]);

            float dotRow = rightVector.x*rightVector.x + rightVector.y*rightVector.y + rightVector.z*rightVector.z;
            float multiplier = 1.0f / squareRoot(dotRow < 0.00000001f ? 0.00000001f : dotRow);
            viewMatrix[0] = multiplyVector(rightVector, multiplier);

            // Cross product to get upwards row
            viewMatrix[1] = crossProduct(viewMatrix[2], viewMatrix[0]);

            // Convert rotation matrix to quaternion
            float fourXSquaredMinus1 = viewMatrix[0].x - viewMatrix[1].y - viewMatrix[2].z;
            float fourYSquaredMinus1 = viewMatrix[1].y - viewMatrix[0].x - viewMatrix[2].z;
            float fourZSquaredMinus1 = viewMatrix[2].z - viewMatrix[0].x - viewMatrix[1].y;
            float fourWSquaredMinus1 = viewMatrix[0].x + viewMatrix[1].y + viewMatrix[2].z;

            int biggestIndex = 0;
            float fourBiggestSquaredMinus1 = fourWSquaredMinus1;
            if (fourXSquaredMinus1 > fourBiggestSquaredMinus1) {
                fourBiggestSquaredMinus1 = fourXSquaredMinus1;
                biggestIndex = 1;
            }
            if (fourYSquaredMinus1 > fourBiggestSquaredMinus1) {
                fourBiggestSquaredMinus1 = fourYSquaredMinus1;
                biggestIndex = 2;
            }
            if (fourZSquaredMinus1 > fourBiggestSquaredMinus1) {
                fourBiggestSquaredMinus1 = fourZSquaredMinus1;
                biggestIndex = 3;
            }

            float biggestValue = squareRoot(fourBiggestSquaredMinus1 + 1.0f) * 0.5f;
            float mult = 0.25f / biggestValue;

            float q_w = 0.0f;
            float q_x = 0.0f;
            float q_y = 0.0f;
            float q_z = 0.0f;

            if (biggestIndex == 0) {
                q_w = biggestValue;
                q_x = (viewMatrix[1].z - viewMatrix[2].y) * mult;
                q_y = (viewMatrix[2].x - viewMatrix[0].z) * mult;
                q_z = (viewMatrix[0].y - viewMatrix[1].x) * mult;
            }
            else if (biggestIndex == 1) {
                q_w = (viewMatrix[1].z - viewMatrix[2].y) * mult;
                q_x = biggestValue;
                q_y = (viewMatrix[0].y + viewMatrix[1].x) * mult;
                q_z = (viewMatrix[2].x + viewMatrix[0].z) * mult;
            }
            else if (biggestIndex == 2) {
                q_w = (viewMatrix[2].x - viewMatrix[0].z) * mult;
                q_x = (viewMatrix[0].y + viewMatrix[1].x) * mult;
                q_y = biggestValue;
                q_z = (viewMatrix[1].z + viewMatrix[2].y) * mult;
            }
            else if (biggestIndex == 3) {
                q_w = (viewMatrix[0].y - viewMatrix[1].x) * mult;
                q_x = (viewMatrix[2].x + viewMatrix[0].z) * mult;
                q_y = (viewMatrix[1].z + viewMatrix[2].y) * mult;
                q_z = biggestValue;
            }

            // Multiply the game's rotation quaternion with the headset's rotation quaternion
            float new_q_x = q_x * hmdQuatW + q_y * hmdQuatZ - q_z * hmdQuatY + q_w * hmdQuatX;
            float new_q_y = -q_x * hmdQuatZ + q_y * hmdQuatW + q_z * hmdQuatX + q_w * hmdQuatY;
            float new_q_z = q_x * hmdQuatY - q_y * hmdQuatX + q_z * hmdQuatW + q_w * hmdQuatZ;
            float new_q_w = -q_x * hmdQuatX - q_y * hmdQuatY - q_z * hmdQuatZ + q_w * hmdQuatW;

            // Convert new quaterion into a 3x3 rotation matrix
            float qxx = new_q_x * new_q_x;
            float qyy = new_q_y * new_q_y;
            float qzz = new_q_z * new_q_z;
            float qxz = new_q_x * new_q_z;
            float qxy = new_q_x * new_q_y;
            float qyz = new_q_y * new_q_z;
            float qwx = new_q_w * new_q_x;
            float qwy = new_q_w * new_q_y;
            float qwz = new_q_w * new_q_z;

            Vec3 newViewMatrix[3];
            newViewMatrix[0].x = 1.0f - 2.0f * (qyy + qzz);
            newViewMatrix[0].y = 2.0f * (qxy + qwz);
            newViewMatrix[0].z = 2.0f * (qxz - qwy);

            newViewMatrix[1].x = 2.0f * (qxy - qwz);
            newViewMatrix[1].y = 1.0f - 2.0f * (qxx + qzz);
            newViewMatrix[1].z = 2.0f * (qyz + qwx);

            newViewMatrix[2].x = 2.0f * (qxz + qwy);
            newViewMatrix[2].y = 2.0f * (qyz - qwx);
            newViewMatrix[2].z = 1.0f - 2.0f * (qxx + qyy);

            // Calculate rotation offset
            float distance = squareRoot(((oldTargetX - oldPosX) * (oldTargetX - oldPosX)) + ((oldTargetY - oldPosY) * (oldTargetY - oldPosY)) + ((oldTargetZ - oldPosZ) * (oldTargetZ - oldPosZ)));
            Vec3 rotationOffset = { (newViewMatrix[2].x * -1.0f) * distance, (newViewMatrix[2].y * -1.0f) * distance , (newViewMatrix[2].z * -1.0f) * distance };

            // Calculate headset position offset
            Vec3 positionOffset;
            Vec3 hmdPosVector = { hmdPosX, hmdPosY, hmdPosZ };
            Vec3 quatVector = { new_q_x, new_q_y, new_q_z };

            Vec3 midResult = crossProduct(quatVector, hmdPosVector);
            Vec3 addResult = multiplyVector(hmdPosVector, new_q_w);
            midResult.x += addResult.x;
            midResult.y += addResult.y;
            midResult.z += addResult.z;

            positionOffset = multiplyVector(crossProduct(quatVector, midResult), 2.0f);
            positionOffset.x += hmdPosVector.x;
            positionOffset.y += hmdPosVector.y;
            positionOffset.z += hmdPosVector.z;

            // Set final values
            newTargetX = oldPosX + rotationOffset.x + (positionOffset.x * setting_headPositionSensitivity);
            newTargetY = oldPosY + rotationOffset.y + (positionOffset.y * setting_headPositionSensitivity) - setting_heightPositionOffset;
            newTargetZ = oldPosZ + rotationOffset.z + (positionOffset.z * setting_headPositionSensitivity);

            newPosX = oldPosX + (positionOffset.x * setting_headPositionSensitivity);
            newPosY = oldPosY + (positionOffset.y * setting_headPositionSensitivity) - setting_heightPositionOffset;
            newPosZ = oldPosZ + (positionOffset.z * setting_headPositionSensitivity);

            newRotX = newViewMatrix[1].x;
            newRotY = newViewMatrix[1].y;
            newRotZ = newViewMatrix[1].z;

            // Set the output data to the final values
            copyCamera.target.x = newTargetX;
            copyCamera.target.y = newTargetY;
            copyCamera.target.z = newTargetZ;

            copyCamera.pos.x = newPosX;
            copyCamera.pos.y = newPosY;
            copyCamera.pos.z = newPosZ;

            copyCamera.rot.x = newRotX;
            copyCamera.rot.y = newRotY;
            copyCamera.rot.z = newRotZ;
#elif CALC_MODE == APP_CALC_LIBRARY
            Eigen::Vector3f forwardVector = Eigen::Vector3f(copyCamera.target.x, copyCamera.target.y, copyCamera.target.z) - Eigen::Vector3f(copyCamera.pos.x, copyCamera.pos.y, copyCamera.pos.z);
            forwardVector.normalize();

            glm::vec3 direction(forwardVector.x(), forwardVector.y(), forwardVector.z());
            glm::fquat tempGameQuat = glm::quatLookAtRH(direction, glm::vec3(0.0, 1.0, 0.0));
            Eigen::Quaternionf gameQuat = Eigen::Quaternionf(tempGameQuat.w, tempGameQuat.x, tempGameQuat.y, tempGameQuat.z);
            Eigen::Quaternionf hmdQuat = Eigen::Quaternionf(middleScreen.orientation.w, middleScreen.orientation.x, middleScreen.orientation.y, middleScreen.orientation.z);
            Eigen::Quaternionf differenceQuat = Eigen::Quaternionf(middleScreen.orientation.w, middleScreen.orientation.x, middleScreen.orientation.y, middleScreen.orientation.z);

            gameQuat = gameQuat * hmdQuat;

            Eigen::Matrix3f rotMatrix = hmdQuat.toRotationMatrix();

            Eigen::Vector3f targetVec = rotMatrix * Eigen::Vector3f(0.0, 0.0, -1.0);
            float distance = (float)std::hypot(copyCamera.target.x - copyCamera.pos.x, copyCamera.target.y - copyCamera.pos.y, copyCamera.target.z - copyCamera.pos.z);
            float targetVecX = targetVec.x() * distance;
            float targetVecY = targetVec.y() * distance;
            float targetVecZ = targetVec.z() * distance;

            Eigen::Vector3f hmdPos(middleScreen.position.x, middleScreen.position.y, middleScreen.position.z);
            hmdPos = gameQuat * hmdPos;

            copyCamera.target.x = copyCamera.pos.x + targetVecX + (hmdPos.x() * const_HeadPositionalMovementSensitivity);
            copyCamera.target.y = copyCamera.pos.y + targetVecY + (hmdPos.y() * const_HeadPositionalMovementSensitivity);
            copyCamera.target.z = copyCamera.pos.z + targetVecZ + (hmdPos.z() * const_HeadPositionalMovementSensitivity);

            copyCamera.pos.x = copyCamera.pos.x + hmdPos.x() * const_HeadPositionalMovementSensitivity;
            copyCamera.pos.y = copyCamera.pos.y + hmdPos.y() * const_HeadPositionalMovementSensitivity;
            copyCamera.pos.z = copyCamera.pos.z + hmdPos.z() * const_HeadPositionalMovementSensitivity;

            copyCamera.rot.x = rotMatrix.data()[3];
            copyCamera.rot.y = rotMatrix.data()[4];
            copyCamera.rot.z = rotMatrix.data()[5];
#endif
        }
        reverseCopyBufferEndianess(&copyCamera);
        WriteProcessMemory(cemuHandle, (void*)(baseAddress + vrAddress), (const void*)&hookStatus, sizeof(HOOK_MODE), &writtenSize);
        WriteProcessMemory(cemuHandle, (void*)(baseAddress + vrAddress + sizeof(inputDataBuffer) + sizeof(copyDataBuffer) - sizeof(XrVector3f)), (const void*)&copyCamera, sizeof(copyDataBuffer), &writtenSize);
#endif
    }
    else if (retryFindingCemu > 0)
        retryFindingCemu--;
    else
        InitializeCemuHooking();
}

HWND getCemuHWND()
{
    return cemuHWND;
}

HOOK_MODE getHookMode()
{
    return hookStatus;
}

void setCemuFullScreen(bool enabled) {
    cemuFullscreen = enabled;
}

bool getCemuFullScreen() {
    return cemuFullscreen;
}

bool getRenderSide() {
    return leftSideRender;
}