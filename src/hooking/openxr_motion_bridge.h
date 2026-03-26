#pragma once

#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
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

    // Constants
    static constexpr float PIf = glm::pi<float>();
    static constexpr float TWO_PIf = glm::two_pi<float>();

public:
    // Process OpenXR motion data and convert to Wii U format
    // xrPose: World space pose (Standard Right-Handed, Y-Up)
    // xrAngVel: Angular velocity in Radians/sec
    // xrAccel: Acceleration in Meters/sec^2

    WiiUMotionData Process(const glm::quat& xrPose, const glm::vec3& xrAngVel, const glm::vec3& xrAccel) {
        WiiUMotionData out;

        // 1. Map inputs to Cemu Coordinate System
        // Based on SDL: Gyro passed as (x, -y, -z). Accel passed as (-x, y, z).
        out.acc = glm::vec3(-xrAccel.x, xrAccel.y, xrAccel.z);
        out.gyro = glm::vec3(xrAngVel.x, -xrAngVel.y, -xrAngVel.z);

        // 2. Jerk calculation
        if (firstFrame) {
            out.jerk = 0.0f;
            lastAcc = xrAccel; // Keep jerk calculation in raw space to avoid mixups
        }
        else {
            out.jerk = glm::length(xrAccel - lastAcc);
            lastAcc = xrAccel;
        }

        // 3. Transformation to Cemu/Mahony Reference Frame
        // Cemu's Mahony filter assumes a rest pose of (0.707, 0.707, 0, 0)
        // implying a 90 degree rotation around X relative to the identity.
        // We apply this offset to the OpenXR pose.
        static const glm::quat offsetQ = glm::quat(0.70710678f, 0.70710678f, 0.0f, 0.0f);
        glm::quat q = offsetQ * xrPose;

        // User Requirement: "quad: Quaternion (reordered {w, x, y, z})" for output.
        // We output the transformed quaternion to match the Euler angles derived from it.
        float w = q.w;
        float x = q.x;
        float y = q.y;
        float z = q.z;

        out.quad = { w, x, y, z };

        // 4. Euler Extraction (Exact Mahony Formulas)
        // These formulas are specific to the coordinate system used in Cemu's Mahony filter.
        // They map the quaternion to (MahonyRoll, MahonyPitch, MahonyYaw).

        // Mahony Roll (Corresponds to VPAD Roll / Z-axis)
        // Formula: atan2(2(zw + xy), 1 - 2(w^2 + x^2))
        float rollY = 2.0f * (z * w + x * y);
        float rollX = 1.0f - 2.0f * (w * w + x * x);
        float currentRoll = std::atan2(rollY, rollX);

        // Mahony Pitch (Corresponds to VPAD Pitch / Y-axis)
        // Formula: asin(2(zx - yw))
        float pitchVal = 2.0f * (z * x - y * w);
        if (pitchVal > 1.0f) pitchVal = 1.0f;
        if (pitchVal < -1.0f) pitchVal = -1.0f;
        float currentPitch = std::asin(pitchVal);

        // Mahony Yaw (Corresponds to VPAD Yaw / X-axis)
        // Formula: atan2(2(zy + wx), 1 - 2(x^2 + y^2))
        float yawY = 2.0f * (z * y + w * x);
        float yawX = 1.0f - 2.0f * (x * x + y * y);
        float currentYaw = std::atan2(yawY, yawX);

        if (firstFrame) {
            lastEuler = { currentRoll, currentPitch, currentYaw };
            firstFrame = false;
        }

        // 5. Winding Logic
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

        // Track winding for each axis (Roll, Pitch, Yaw)
        // Note: lastEuler stores {Roll, Pitch, Yaw} in Mahony definitions
        updateWinding(currentRoll, lastEuler.x, winding.x);
        updateWinding(currentPitch, lastEuler.y, winding.y);
        updateWinding(currentYaw, lastEuler.z, winding.z);

        // 6. Final Value Mapping (MotionHandler to VPAD logic)
        // MotionHandler maps Mahony values to VPAD indices as follows:
        // Index 0 (Yaw):   -YawRad   - 0.5
        // Index 1 (Pitch): -PitchRad - 0.5
        // Index 2 (Roll):   RollRad

        float vpadYaw = (-currentYaw / TWO_PIf) - winding.z - 0.5;
        float vpadPitch = (-currentPitch / TWO_PIf) - winding.y - 0.5;
        float vpadRoll = (currentRoll / TWO_PIf) + winding.x;

        // Output vector: x=Yaw, y=Pitch, z=Roll
        out.orientation = { vpadYaw, vpadPitch, vpadRoll };

        return out;
    }

    static void UpdateVPADStatus(const OpenXR::InputState& inputs, VPADStatus& vpadStatus) {
        // motion
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

            // Calculate Acceleration (World Space) + Gravity
            glm::vec3 accWorld = (linearVel - lastWorldVelocity) / dt;
            accWorld += glm::vec3(0.0f, 9.81f, 0.0f);
            lastWorldVelocity = linearVel;

            // Convert to Local Space
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
        else if (GetSettings().GetCameraMode() == CameraMode::ORIGINAL) {
            if (GetSettings().gyroFlipYZOriginal.Get()) {
                const glm::fvec3 prevGyroChange = vpadStatus.gyroChange.getLE();
                const glm::fvec3 prevGyroOrientation = vpadStatus.gyroOrientation.getLE();

                vpadStatus.gyroChange = glm::fvec3{ prevGyroChange.x, prevGyroChange.z, prevGyroChange.y };
                vpadStatus.gyroOrientation = glm::fvec3{ prevGyroOrientation.x, prevGyroOrientation.z, prevGyroOrientation.y };
            }
        }
        else {
            vpadStatus.dir.x = glm::fvec3{ 1, 0, 0 };
            vpadStatus.dir.y = glm::fvec3{ 0, 1, 0 };
            vpadStatus.dir.z = glm::fvec3{ 0, 0, 1 };
            vpadStatus.accXY = { 1.0f, 0.0f };
        }
    }
};
