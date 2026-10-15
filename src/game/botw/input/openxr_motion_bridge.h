#pragma once

#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "rendering/openxr.h"
#include <glm/gtx/quaternion.hpp>

struct WiiUMotionData {
    glm::vec3 acc;
    glm::vec3 gyro;
    glm::vec3 orientation; // Euler angles in Revolutions (x=Roll, y=Pitch, z=Yaw)
    float jerk;
    struct {
        float w, x, y, z;
    } quad;
};

class OpenXRMotionBridge {
private:
    glm::vec3 lastEuler = { 0.0f, 0.0f, 0.0f };
    glm::ivec3 winding = { 0, 0, 0 };
    glm::vec3 lastAcc = { 0.0f, 0.0f, 0.0f };
    bool firstFrame = true;

    static constexpr float PIf = glm::pi<float>();
    static constexpr float TWO_PIf = glm::two_pi<float>();

public:
    WiiUMotionData Process(const glm::quat& xrPose, const glm::vec3& xrAngVel, const glm::vec3& xrAccel) {
        WiiUMotionData out;

        // Based on SDL: Gyro passed as (x, -y, -z). Accel passed as (-x, y, z).
        out.acc = glm::vec3(-xrAccel.x, xrAccel.y, xrAccel.z);
        out.gyro = glm::vec3(xrAngVel.x, -xrAngVel.y, -xrAngVel.z);

        if (firstFrame) {
            out.jerk = 0.0f;
            lastAcc = xrAccel; // Keep jerk calculation in raw space to avoid mixups
        }
        else {
            out.jerk = glm::length(xrAccel - lastAcc);
            lastAcc = xrAccel;
        }

        // Cemu's Mahony filter assumes a rest pose of (0.707, 0.707, 0, 0)
        static const glm::quat offsetQ = glm::quat(0.70710678f, 0.70710678f, 0.0f, 0.0f);
        glm::quat q = offsetQ * xrPose;

        float w = q.w;
        float x = q.x;
        float y = q.y;
        float z = q.z;

        out.quad = { w, x, y, z };


        // Mahony Roll (Corresponds to VPAD Roll / Z-axis)
        float rollY = 2.0f * (z * w + x * y);
        float rollX = 1.0f - 2.0f * (w * w + x * x);
        float currentRoll = std::atan2(rollY, rollX);

        // Mahony Pitch (Corresponds to VPAD Pitch / Y-axis)
        float pitchVal = 2.0f * (z * x - y * w);
        if (pitchVal > 1.0f) pitchVal = 1.0f;
        if (pitchVal < -1.0f) pitchVal = -1.0f;
        float currentPitch = std::asin(pitchVal);

        // Mahony Yaw (Corresponds to VPAD Yaw / X-axis)
        float yawY = 2.0f * (z * y + w * x);
        float yawX = 1.0f - 2.0f * (x * x + y * y);
        float currentYaw = std::atan2(yawY, yawX);

        if (firstFrame) {
            lastEuler = { currentRoll, currentPitch, currentYaw };
            firstFrame = false;
        }

        auto updateWinding = [&](float current, float& last, int& windingVal) {
            float diff = current - last;
            if (diff > PIf) {
                windingVal--;
            }
            else if (diff < -PIf) {
                windingVal++;
            }
            last = current;
        };

        updateWinding(currentRoll, lastEuler.x, winding.x);
        updateWinding(currentPitch, lastEuler.y, winding.y);
        updateWinding(currentYaw, lastEuler.z, winding.z);

        // Index 0 (Yaw):   -YawRad   - 0.5
        // Index 1 (Pitch): -PitchRad - 0.5
        // Index 2 (Roll):   RollRad

        float vpadYaw = (-currentYaw / TWO_PIf) - winding.z - 0.5;
        float vpadPitch = (-currentPitch / TWO_PIf) - winding.y - 0.5;
        float vpadRoll = (currentRoll / TWO_PIf) + winding.x;

        out.orientation = { vpadYaw, vpadPitch, vpadRoll };

        return out;
    }

    static void UpdateVPADStatus(const OpenXR::InputState& inputs, VPADStatus& vpadStatus) {
        static OpenXRMotionBridge motionBridge;
        static glm::vec3 lastWorldVelocity = { 0.0f, 0.0f, 0.0f };
        static XrTime lastInputTime = 0;

        // Using Right Hand (index 1) for motion controls
        const int handIdx = 1;
        auto& poseState = inputs.shared.poseLocation[handIdx];
        auto& velState = inputs.shared.poseVelocity[handIdx];

        if ((poseState.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) && inputs.shared.in_game) {
            glm::quat orientation = ToGLM(poseState.pose.orientation);
            glm::vec3 angularVel = ToGLM(velState.angularVelocity);
            glm::vec3 linearVel = ToGLM(velState.linearVelocity);

            float dt = 1.0f / 60.0f;
            if (lastInputTime != 0) {
                dt = (float)(inputs.shared.inputTime - lastInputTime) * 1e-9f;
            }
            if (dt <= 1e-5f) dt = 1.0f / 60.0f;
            if (dt > 0.1f) dt = 0.1f;
            lastInputTime = inputs.shared.inputTime;

            glm::vec3 accWorld = (linearVel - lastWorldVelocity) / dt;
            accWorld += glm::vec3(0.0f, 9.81f, 0.0f);
            lastWorldVelocity = linearVel;

            glm::quat invRot = glm::inverse(orientation);
            glm::vec3 accLocal = invRot * accWorld;
            glm::vec3 gyroLocal = invRot * angularVel;

            WiiUMotionData motion = motionBridge.Process(orientation, gyroLocal, accLocal);

            vpadStatus.acc = motion.acc;
            vpadStatus.accMagnitude = glm::length(motion.acc);
            vpadStatus.accAcceleration = motion.jerk;
            vpadStatus.gyroChange = motion.gyro;
            vpadStatus.gyroOrientation = motion.orientation;
            vpadStatus.accXY = { motion.acc.x, motion.acc.y };

            // Construct corrected orientation for 'dir' to match Bridge logic
            // Bridge Logic: Invert Pitch, Invert Roll, Keep Yaw.
            glm::vec3 euler = glm::eulerAngles(orientation); // Pitch(x), Yaw(y), Roll(z)
            glm::quat corrected = glm::quat(glm::vec3(-euler.x, euler.y, -euler.z));

            vpadStatus.dir.x = corrected * glm::vec3(1, 0, 0);
            vpadStatus.dir.y = corrected * glm::vec3(0, 1, 0);
            vpadStatus.dir.z = corrected * glm::vec3(0, 0, 1);
        }
        else {
            vpadStatus.dir.x = glm::fvec3{ 1, 0, 0 };
            vpadStatus.dir.y = glm::fvec3{ 0, 1, 0 };
            vpadStatus.dir.z = glm::fvec3{ 0, 0, 1 };
            vpadStatus.accXY = { 1.0f, 0.0f };
        }
    }
};
