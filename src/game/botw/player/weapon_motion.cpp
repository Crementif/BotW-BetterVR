#include "pch.h"
#include "weapon_motion.h"

WeaponProfile WeaponProfile::ForSensitivity(SwingSensitivity sensitivity) {
    WeaponProfile p = {};

    p.stabSpeedThreshold = 0.05f;
    p.stabAccThreshold = 4.0f;
    p.stabLinearSteadinessThreshold = glm::cos(glm::pi<float>() / 3.0f); // ~60 deg cone to avoid over-penalizing wrist angle
    p.stabAngularSteadinessThreshold = 4.5f;
    p.stabTravelDistance = 0.12f;
    p.stabAccSmoothingTimeConstant = 0.050f;
    p.stabVelocityThreshold = 1.0f;

    p.slashSpeedThreshold = 1.0f;
    p.slashAccThreshold = 14.0f;
    p.slashVelocityThreshold = 5.0f;
    p.slashAccDriftThreshold = 10.0f;
    p.slashTravelAngle = glm::pi<float>() / 6.0f; // 30 deg

    p.maxBadDuration = 0.033f;
    p.goodSampleGracePeriod = (XrTime)80e6; // 80 ms
    p.minGoodSwingDuration = 0.025f;
    p.minGoodStabDuration = 0.025f;
    p.smoothingTimeConstant = 0.030f;
    p.angularDriftMinVelocity = 0.5f;

    p.referenceArmLength = 0.55f;

    switch (sensitivity) {
        case SwingSensitivity::SWING_EASY:
            break;

        case SwingSensitivity::SWING_NORMAL:
            p.stabLinearSteadinessThreshold = glm::cos(glm::pi<float>() / 4.0f); // ~45 deg cone, still stricter than relaxed
            p.slashAccThreshold = 20.0f;
            p.slashVelocityThreshold = 7.0f;
            p.slashSpeedThreshold = 1.5f;
            p.slashTravelAngle = glm::pi<float>() / 5.0f; // 36 deg
            p.maxBadDuration = 0.022f;
            p.goodSampleGracePeriod = (XrTime)40e6; // 40 ms
            p.minGoodSwingDuration = 0.040f;
            p.minGoodStabDuration = 0.030f;
            p.smoothingTimeConstant = 0.020f;
            break;
    }

    return p;
}

void WeaponMotionAnalyser::ApplyProfile(SwingSensitivity sensitivity) {
    m_profile = WeaponProfile::ForSensitivity(sensitivity);
}

void WeaponMotionAnalyser::computeLocalSignals(
    const glm::fvec3& linearVelocity,
    const glm::fvec3& angularVelocity,
    const glm::fquat& rotation,
    float dt)
{
    const glm::fquat invRotation = glm::inverse(rotation);
    m_localLinVel = invRotation * linearVelocity;
    m_localAngVel = invRotation * angularVelocity;

    m_localLinAccel = (dt > 0.0f) ? (m_localLinVel - m_prevLocalLinVel) / dt : glm::fvec3(0.0f);

    const glm::fvec3 flatAngVel = m_localAngVel - glm::fvec3(0.0f, 0.0f, m_localAngVel.z);
    m_flatAngAcc = (dt > 0.0f) ? glm::length(flatAngVel - m_prevFlatAngVel) / dt : 0.0f;
    m_prevFlatAngVel = flatAngVel;

    m_handSpeed = glm::length(linearVelocity);
}

float WeaponMotionAnalyser::computeAngularDrift(const glm::fvec3& angularVelocity, float dt) {
    const float magCurr = glm::length(angularVelocity);
    const float magPrev = glm::length(m_prevAngularVelocity);

    if (magCurr < m_profile.angularDriftMinVelocity || magPrev < m_profile.angularDriftMinVelocity || dt <= 0.0f) {
        return 0.0f;
    }

    const float cosAngle = glm::clamp(
        glm::dot(glm::normalize(angularVelocity), glm::normalize(m_prevAngularVelocity)),
        -1.0f, 1.0f);
    return std::acos(cosAngle) / dt;
}

void WeaponMotionAnalyser::computeSmoothedSignals(float dt) {
    const float alpha = (dt > 0.0f) ? 1.0f - std::exp(-dt / m_profile.smoothingTimeConstant) : 0.0f;
    m_smoothedLinAccel = glm::mix(m_smoothedLinAccel, m_localLinAccel, alpha);
    m_smoothedFlatAngAcc = std::lerp(m_smoothedFlatAngAcc, m_flatAngAcc, alpha);
    m_smoothedAngDrift = std::lerp(m_smoothedAngDrift, m_angularDrift, alpha);

    const float alphaSlow = (dt > 0.0f) ? 1.0f - std::exp(-dt / m_profile.stabAccSmoothingTimeConstant) : 0.0f;
    m_smoothedLinAccelSlow = glm::mix(m_smoothedLinAccelSlow, m_localLinAccel, alphaSlow);
}

void WeaponMotionAnalyser::updateArmCalibration(float handToHeadDist) {
    // slowly moves towards the longest reach seen, so shorter arms aren't punished
    if (handToHeadDist > m_calibratedArmLength) {
        m_calibratedArmLength = std::lerp(m_calibratedArmLength, handToHeadDist, 0.05f);
    }
    else {
        m_calibratedArmLength = std::lerp(m_calibratedArmLength, handToHeadDist, 0.002f);
    }
}

void WeaponMotionAnalyser::detectAttackType(const glm::fvec3& position, const glm::fquat& rotation, XrTime inputTime, float dt) {
    if (m_lockedAttackType != AttackType::None)
        return;

    // so pulling back after an attack doesn't count as a new one
    const bool inRefractory = inputTime < m_refractoryUntil;

    bool stabGateOk = false;
    if (glm::length2(m_localLinVel) >= 1e-8f && !inRefractory) {
        const glm::fvec3 stabDir = glm::normalize(m_localLinVel);
        const bool stabDirectionOk = std::abs(stabDir.z) > m_profile.stabLinearSteadinessThreshold;
        const bool stabAngularOk = std::abs(m_localAngVel.x) < m_profile.stabAngularSteadinessThreshold
                                && std::abs(m_localAngVel.y) < m_profile.stabAngularSteadinessThreshold;
        const bool stabImpulseOk = -m_smoothedLinAccelSlow.z > m_profile.stabAccThreshold
                                || -m_localLinVel.z > m_profile.stabVelocityThreshold;

        stabGateOk = stabDirectionOk && stabAngularOk && stabImpulseOk;
        if (stabGateOk) {
            if (m_goodStabAccumTime == 0.0f) {
                m_lockedPosition = position;
            }
            m_goodStabAccumTime += dt;
            m_lastGoodStabTime = inputTime;
            Log::print<CONTROLS>("Stab detect attack");
            if (m_goodStabAccumTime > m_profile.minGoodStabDuration) {
                m_lockedAttackType = AttackType::Stab;
                m_peakAttackVelocity = 0.0f;
                m_lockTime = inputTime;
            }
        }
        else if ((inputTime - m_lastGoodStabTime) > m_profile.goodSampleGracePeriod) {
            m_goodStabAccumTime = 0.0f;
        }
    }

    // either a quick spin up or a fast enough spin
    if (!stabGateOk && m_lockedAttackType != AttackType::Stab) {
        const float flatAngVelMag = glm::length(glm::fvec3(m_localAngVel.x, m_localAngVel.y, 0.0f));
        const bool slashByAccel = !inRefractory && m_smoothedFlatAngAcc > m_profile.slashAccThreshold;
        const bool slashByVelocity = flatAngVelMag > m_profile.slashVelocityThreshold;

        if (slashByAccel || slashByVelocity) {
            if (m_goodSwingAccumTime == 0.0f) {
                m_lockedAngle = rotation * glm::fvec3(0.0f, 0.0f, 1.0f);
            }
            m_goodSwingAccumTime += dt;
            m_lastGoodSwingTime = inputTime;
            Log::print<CONTROLS>("Slash attack detected (accel={}, vel={})", slashByAccel, slashByVelocity);
            if (m_goodSwingAccumTime > m_profile.minGoodSwingDuration) {
                m_lockedAttackType = AttackType::Slash;
                m_peakAttackVelocity = 0.0f;
                m_lockTime = inputTime;
            }
        }
        else if ((inputTime - m_lastGoodSwingTime) > m_profile.goodSampleGracePeriod) {
            m_goodSwingAccumTime = 0.0f;
        }
    }
}

void WeaponMotionAnalyser::checkAttackSteadiness(XrTime inputTime, float dt) {
    bool isBadSample = false;

    switch (m_lockedAttackType) {
        case AttackType::None:
            return;

        case AttackType::Stab: {
            if (glm::length2(m_localLinVel) < 1e-8f) {
                isBadSample = true;
            }
            else {
                const glm::fvec3 stabDir = glm::normalize(m_localLinVel);
                const float angXY = glm::length(glm::fvec3(m_localAngVel.x, m_localAngVel.y, 0.0f));

                isBadSample = std::abs(stabDir.z) < m_profile.stabLinearSteadinessThreshold
                           || -m_localLinVel.z < m_profile.stabSpeedThreshold
                           || angXY > m_profile.stabAngularSteadinessThreshold;
            }
            break;
        }

        case AttackType::Slash: {
            const float flatAngVelMag = glm::length(glm::fvec3(m_localAngVel.x, m_localAngVel.y, 0.0f));

            if (m_smoothedAngDrift > m_profile.slashAccDriftThreshold) {
                Log::print<CONTROLS>("[FAIL]: Drift: {}/{}", m_smoothedAngDrift, m_profile.slashAccDriftThreshold);
                isBadSample = true;
            }
            else if (flatAngVelMag < m_profile.slashSpeedThreshold) {
                Log::print<CONTROLS>("[FAIL]: Angular velocity: {}/{}", flatAngVelMag, m_profile.slashSpeedThreshold);
                isBadSample = true;
            }
            break;
        }
    }

    const XrTime lockGrace = std::max((XrTime)33e6, (XrTime)(2.5f * m_smoothedFrameDt * 1e9f));
    if (inputTime - m_lockTime < lockGrace) {
        isBadSample = false;
    }

    if (isBadSample) {
        m_badSampleAccumTime += dt;
    }
    else {
        m_badSampleAccumTime = std::max(0.0f, m_badSampleAccumTime - dt);
    }

    const float maxBadDuration = std::max(m_profile.maxBadDuration, 2.5f * m_smoothedFrameDt);
    if (m_badSampleAccumTime >= maxBadDuration) {
        m_badSampleAccumTime = 0.0f;
        if (m_attackActive) {
            m_refractoryUntil = inputTime + (XrTime)250e6;
        }
        if (m_lockedAttackType == AttackType::Slash) {
            m_goodSwingAccumTime = 0.0f;
            m_lastGoodSwingTime = 0;
        }
        else {
            m_goodStabAccumTime = 0.0f;
            m_lastGoodStabTime = 0;
        }
        m_lockedAttackType = AttackType::None;
    }
}

void WeaponMotionAnalyser::updateAttackActivity(const glm::fquat& rotation, const glm::fvec3& position) {
    if (m_lockedAttackType == AttackType::None) {
        m_attackActive = false;
        return;
    }

    if (m_attackActive)
        return;

    const float armScale = m_calibratedArmLength / m_profile.referenceArmLength;

    switch (m_lockedAttackType) {
        case AttackType::Stab: {
            const float travelDist = glm::length(position - m_lockedPosition);
            if (travelDist > m_profile.stabTravelDistance * armScale) {
                m_attackActive = true;
            }
            break;
        }
        case AttackType::Slash: {
            const glm::fvec3 zNow = rotation * glm::fvec3(0.0f, 0.0f, 1.0f);
            const float dotProduct = glm::clamp(glm::dot(zNow, m_lockedAngle), -1.0f, 1.0f);
            const float angDifference = std::acos(dotProduct);
            if (angDifference > m_profile.slashTravelAngle) {
                m_attackActive = true;
            }
            break;
        }
        default:
            break;
    }
}

void WeaponMotionAnalyser::updateSwingPower() {
    if (!m_attackActive) {
        m_swingPower = 0.0f;
        return;
    }

    float currentVelocity = 0.0f;
    float referenceVelocity = 1.0f;

    switch (m_lockedAttackType) {
        case AttackType::Slash:
            currentVelocity = glm::length(glm::fvec3(m_localAngVel.x, m_localAngVel.y, 0.0f));
            referenceVelocity = REFERENCE_SLASH_VELOCITY;
            break;
        case AttackType::Stab: {
            currentVelocity = std::abs(m_localLinVel.z);
            referenceVelocity = REFERENCE_STAB_VELOCITY;
            m_peakAttackVelocity = std::max(m_peakAttackVelocity, currentVelocity);

            float straightness = 0.0f;
            if (glm::length2(m_localLinVel) >= 1e-8f) {
                straightness = std::abs(glm::normalize(m_localLinVel).z);
            }

            const float normalizedStraightness = glm::clamp((straightness - m_profile.stabLinearSteadinessThreshold) / (1.0f - m_profile.stabLinearSteadinessThreshold), 0.0f, 1.0f);
            const float straightnessFactor = std::lerp(0.70f, 1.0f, normalizedStraightness);
            m_swingPower = std::clamp(m_peakAttackVelocity / referenceVelocity, 0.0f, 1.0f) * straightnessFactor;
            break;
        }
        default:
            break;
    }

    if (m_lockedAttackType != AttackType::Stab) {
        m_peakAttackVelocity = std::max(m_peakAttackVelocity, currentVelocity);
        m_swingPower = std::clamp(m_peakAttackVelocity / referenceVelocity, 0.0f, 1.0f);
    }
}

void WeaponMotionAnalyser::Update(
    const XrSpaceLocation& handLocation,
    const XrSpaceVelocity& handVelocity,
    const glm::fmat4& headsetMtx,
    XrTime inputTime)
{
    const glm::fvec3 linearVelocity = ToGLM(handVelocity.linearVelocity);
    const glm::fvec3 angularVelocity = ToGLM(handVelocity.angularVelocity);
    const glm::fquat rotation = ToGLM(handLocation.pose.orientation);
    const glm::fvec3 position = ToGLM(handLocation.pose.position);
    const glm::fvec3 headsetPosition = glm::fvec3(headsetMtx[3]);

    const float dt = (m_prevSampleTime > 0)
        ? static_cast<float>(inputTime - m_prevSampleTime) / 1e9f
        : 0.0f;

    // untracked samples have garbage positions
    constexpr XrSpaceLocationFlags requiredFlags = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_POSITION_TRACKED_BIT;
    if ((handLocation.locationFlags & requiredFlags) != requiredFlags) {
        computeLocalSignals(linearVelocity, angularVelocity, rotation, dt);
        m_prevAngularVelocity = angularVelocity;
        m_prevLocalLinVel = m_localLinVel;
        m_prevSampleTime = inputTime;
        return;
    }

    computeLocalSignals(linearVelocity, angularVelocity, rotation, dt);
    m_angularDrift = computeAngularDrift(angularVelocity, dt);
    computeSmoothedSignals(dt);

    if (dt > 0.0f) {
        m_smoothedFrameDt = (m_smoothedFrameDt == 0.0f) ? dt : std::lerp(m_smoothedFrameDt, dt, 1.0f - std::exp(-dt / 0.5f));
    }

    const float handToHeadDist = glm::distance(position, headsetPosition);
    if (handToHeadDist <= 1.2f) {
        updateArmCalibration(handToHeadDist);
    }

    m_samples[m_sampleIdx] = {
        .time = inputTime,
        .position = position,
        .rotation = rotation,
        .localLinearVelocity = m_localLinVel,
        .localAngularVelocity = m_localAngVel,
        .localLinearAcceleration = m_localLinAccel,
        .smoothedLinearAcceleration = m_smoothedLinAccel,
        .smoothedFlatAngAcc = m_smoothedFlatAngAcc,
        .smoothedAngularDrift = m_smoothedAngDrift,
        .attackType = AttackType::None,
        .handSpeed = m_handSpeed,
    };
    m_lastSampleIdx = m_sampleIdx;
    m_sampleIdx = (m_sampleIdx + 1) % MAX_SAMPLES;

    // clamped so a pause or lag spike doesn't count as a long swing
    const float dtAccum = std::min(dt, 0.05f);
    detectAttackType(position, rotation, inputTime, dtAccum);
    checkAttackSteadiness(inputTime, dtAccum);
    updateAttackActivity(rotation, position);
    updateSwingPower();

    m_samples[m_lastSampleIdx].attackType = IsAttacking() ? m_lockedAttackType : AttackType::None;

    m_prevAngularVelocity = angularVelocity;
    m_prevLocalLinVel = m_localLinVel;
    m_prevSampleTime = inputTime;
}

void WeaponMotionAnalyser::ResetSwing() {
    m_goodSwingAccumTime = 0.0f;
    m_lastGoodSwingTime = 0;
    if (m_lockedAttackType == AttackType::Slash) {
        m_lockedAttackType = AttackType::None;
    }
}

void WeaponMotionAnalyser::ResetStab() {
    m_goodStabAccumTime = 0.0f;
    m_lastGoodStabTime = 0;
    if (m_lockedAttackType == AttackType::Stab) {
        m_lockedAttackType = AttackType::None;
    }
}

void WeaponMotionAnalyser::Reset() {
    m_samples.fill({});
    m_sampleIdx = 0;
    m_lastSampleIdx = 0;
    m_badSampleAccumTime = 0.0f;
    m_attackActive = false;
    m_swingPower = 0.0f;
    m_peakAttackVelocity = 0.0f;
    m_smoothedLinAccel = {};
    m_smoothedLinAccelSlow = {};
    m_smoothedFlatAngAcc = 0.0f;
    m_smoothedAngDrift = 0.0f;
    m_lockTime = 0;
    m_prevLocalLinVel = {};
    m_prevFlatAngVel = {};
    m_prevAngularVelocity = {};
    m_prevSampleTime = 0;
    ResetSwing();
    ResetStab();
}

void WeaponMotionAnalyser::ResetIfWeaponTypeChanged(WeaponType weaponType) {
    if (m_weaponType != weaponType) {
        m_weaponType = weaponType;
        Reset();
    }
}

WeaponDebugSnapshot WeaponMotionAnalyser::GetDebugSnapshot() const {
    WeaponDebugSnapshot snapshot = {};
    snapshot.weaponType = m_weaponType;
    snapshot.lockedAttackType = m_lockedAttackType;
    snapshot.sampleTime = m_samples[m_lastSampleIdx].time;
    snapshot.attackActive = m_attackActive;
    snapshot.hitboxEnabled = m_isHitboxEnabled;
    snapshot.swingPower = m_swingPower;
    snapshot.badSampleAccumTime = m_badSampleAccumTime;
    snapshot.maxBadDuration = std::max(m_profile.maxBadDuration, 2.5f * m_smoothedFrameDt);
    snapshot.calibratedArmLength = m_calibratedArmLength;

    snapshot.forwardStabSpeed = -m_localLinVel.z;
    snapshot.stabSpeedThreshold = m_profile.stabSpeedThreshold;
    snapshot.forwardStabAcceleration = -m_smoothedLinAccelSlow.z;
    snapshot.stabAccThreshold = m_profile.stabAccThreshold;
    snapshot.stabAngularXY = glm::length(glm::fvec3(m_localAngVel.x, m_localAngVel.y, 0.0f));
    snapshot.stabAngularSteadinessThreshold = m_profile.stabAngularSteadinessThreshold;
    snapshot.goodStabAccumTime = m_goodStabAccumTime;
    snapshot.minGoodStabDuration = m_profile.minGoodStabDuration;

    if (glm::length2(m_localLinVel) >= 1e-8f) {
        snapshot.stabDirectionAbsZ = std::abs(glm::normalize(m_localLinVel).z);
    }
    snapshot.stabLinearSteadinessThreshold = m_profile.stabLinearSteadinessThreshold;

    snapshot.slashAngularSpeed = glm::length(glm::fvec3(m_localAngVel.x, m_localAngVel.y, 0.0f));
    snapshot.slashSpeedThreshold = m_profile.slashSpeedThreshold;
    snapshot.slashAngularAcceleration = m_smoothedFlatAngAcc;
    snapshot.slashAccThreshold = m_profile.slashAccThreshold;
    snapshot.slashVelocityThreshold = m_profile.slashVelocityThreshold;
    snapshot.slashAngularDrift = m_smoothedAngDrift;
    snapshot.slashAccDriftThreshold = m_profile.slashAccDriftThreshold;
    snapshot.goodSwingAccumTime = m_goodSwingAccumTime;
    snapshot.minGoodSwingDuration = m_profile.minGoodSwingDuration;

    snapshot.stabSpeedOk = snapshot.forwardStabSpeed >= snapshot.stabSpeedThreshold;
    snapshot.stabAccelOk = snapshot.forwardStabAcceleration >= snapshot.stabAccThreshold;
    snapshot.stabDirectionOk = snapshot.stabDirectionAbsZ >= snapshot.stabLinearSteadinessThreshold;
    snapshot.stabAngularOk = snapshot.stabAngularXY <= snapshot.stabAngularSteadinessThreshold;
    snapshot.stabDurationOk = snapshot.goodStabAccumTime >= snapshot.minGoodStabDuration;
    snapshot.slashSpeedOk = snapshot.slashAngularSpeed >= snapshot.slashSpeedThreshold;
    snapshot.slashAccelOk = snapshot.slashAngularAcceleration >= snapshot.slashAccThreshold;
    snapshot.slashVelocityOk = snapshot.slashAngularSpeed >= snapshot.slashVelocityThreshold;
    snapshot.slashDriftOk = snapshot.slashAngularDrift <= snapshot.slashAccDriftThreshold;
    snapshot.slashDurationOk = snapshot.goodSwingAccumTime >= snapshot.minGoodSwingDuration;
    return snapshot;
}
