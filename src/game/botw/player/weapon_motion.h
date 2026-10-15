#pragma once

#include "utils/mod_settings.h"
#include "rendering/openxr.h"

enum class AttackType {
    None = 0,
    Slash = 1,
    Stab = 2
};

struct MotionSample {
    XrTime time = 0;
    glm::fvec3 position = {};
    glm::fquat rotation = glm::identity<glm::fquat>();

    glm::fvec3 localLinearVelocity = {};
    glm::fvec3 localAngularVelocity = {};
    glm::fvec3 localLinearAcceleration = {};

    glm::fvec3 smoothedLinearAcceleration = {};
    float smoothedFlatAngAcc = 0.0f;
    float smoothedAngularDrift = 0.0f;

    AttackType attackType = AttackType::None;
    float handSpeed = 0.0f;
};

struct WeaponProfile {
    // Stab detection
    float stabSpeedThreshold;             // [m/s] minimum forward velocity to maintain stab
    float stabAccThreshold;               // [m/s^2] minimum forward acceleration to start stab
    float stabLinearSteadinessThreshold;   // [unitless] min Z-component ratio of velocity direction
    float stabAngularSteadinessThreshold;  // [rad/s] max angular velocity during stab
    float stabTravelDistance;              // [m] forward travel before stab activates hitbox
    float stabAccSmoothingTimeConstant;    // [s] EMA time constant for the stab acceleration gate
    float stabVelocityThreshold;           // [m/s] sustained forward speed that also opens the stab gate

    // Slash detection
    float slashSpeedThreshold;      // [rad/s] minimum angular velocity (XY plane) to maintain slash
    float slashAccThreshold;        // [rad/s^2] minimum angular acceleration to initiate slash
    float slashVelocityThreshold;   // [rad/s] alternate slash detection via angular velocity magnitude
    float slashAccDriftThreshold;   // [rad/s^2] max angular velocity axis drift before slash fails
    float slashTravelAngle;         // [rad] minimum rotation angle before slash activates hitbox

    // Tolerance
    float maxBadDuration;                             // [s] accumulated bad-sample time before attack cancels
    XrTime goodSampleGracePeriod;                  // [ns] grace period before resetting good-sample counter
    float minGoodSwingDuration;                     // [s] accumulated good-sample time before slash locks
    float minGoodStabDuration;                      // [s] accumulated good-sample time before stab locks
    float smoothingTimeConstant;                     // [s] EMA time constant (larger = more smoothing)
    float angularDriftMinVelocity;                  // [rad/s] minimum angular velocity for drift calculation

    // Auto-calibration
    float referenceArmLength;  // [m] baseline arm length for threshold scaling

    static WeaponProfile ForSensitivity(SwingSensitivity sensitivity);
};

struct WeaponDebugSnapshot {
    WeaponType weaponType = WeaponType::LargeSword;
    AttackType lockedAttackType = AttackType::None;
    XrTime sampleTime = 0;
    bool attackActive = false;
    bool hitboxEnabled = false;
    float swingPower = 0.0f;
    float badSampleAccumTime = 0.0f;
    float maxBadDuration = 0.0f;
    float calibratedArmLength = 0.0f;

    float forwardStabSpeed = 0.0f;
    float stabSpeedThreshold = 0.0f;
    float forwardStabAcceleration = 0.0f;
    float stabAccThreshold = 0.0f;
    float stabDirectionAbsZ = 0.0f;
    float stabLinearSteadinessThreshold = 0.0f;
    float stabAngularXY = 0.0f;
    float stabAngularSteadinessThreshold = 0.0f;
    float goodStabAccumTime = 0.0f;
    float minGoodStabDuration = 0.0f;

    float slashAngularSpeed = 0.0f;
    float slashSpeedThreshold = 0.0f;
    float slashAngularAcceleration = 0.0f;
    float slashAccThreshold = 0.0f;
    float slashVelocityThreshold = 0.0f;
    float slashAngularDrift = 0.0f;
    float slashAccDriftThreshold = 0.0f;
    float goodSwingAccumTime = 0.0f;
    float minGoodSwingDuration = 0.0f;

    bool stabSpeedOk = false;
    bool stabAccelOk = false;
    bool stabDirectionOk = false;
    bool stabAngularOk = false;
    bool stabDurationOk = false;
    bool slashSpeedOk = false;
    bool slashAccelOk = false;
    bool slashVelocityOk = false;
    bool slashDriftOk = false;
    bool slashDurationOk = false;
};

class WeaponMotionAnalyser {
public:
    static constexpr int MAX_SAMPLES = 90;
    static constexpr float HAND_VELOCITY_LENGTH_THRESHOLD = 2.0f;

    WeaponMotionAnalyser() = default;

    void Update(const XrSpaceLocation& handLocation, const XrSpaceVelocity& handVelocity, const glm::fmat4& headsetMtx, XrTime inputTime);

    bool IsAttacking() const { return m_attackActive; }
    bool IsHitboxEnabled() const { return m_isHitboxEnabled; }
    void SetHitboxEnabled(bool enabled) { m_isHitboxEnabled = enabled; }

    // 0-1, from the fastest point of the current attack
    float GetSwingPower() const { return m_swingPower; }

    float GetHandSpeed() const { return m_handSpeed; }

    void ResetSwing();
    void ResetStab();
    void Reset();
    void ResetIfWeaponTypeChanged(WeaponType weaponType);

    void ApplyProfile(SwingSensitivity sensitivity);

    WeaponDebugSnapshot GetDebugSnapshot() const;
    const std::array<MotionSample, MAX_SAMPLES>& GetSamples() const { return m_samples; }
    uint32_t GetLastSampleIndex() const { return m_lastSampleIdx; }

private:
    // ---- Signal processing ----
    void computeLocalSignals(const glm::fvec3& linearVelocity, const glm::fvec3& angularVelocity, const glm::fquat& rotation, float dt);
    void computeSmoothedSignals(float dt);
    float computeAngularDrift(const glm::fvec3& angularVelocity, float dt);
    void updateArmCalibration(float handToHeadDist);

    // ---- Detection pipeline ----
    void detectAttackType(const glm::fvec3& position, const glm::fquat& rotation, XrTime inputTime, float dt);
    void checkAttackSteadiness(XrTime inputTime, float dt);
    void updateAttackActivity(const glm::fquat& rotation, const glm::fvec3& position);
    void updateSwingPower();

    // ---- Profile ----
    WeaponProfile m_profile = WeaponProfile::ForSensitivity(SwingSensitivity::SWING_NORMAL);

    // ---- Sample buffer ----
    std::array<MotionSample, MAX_SAMPLES> m_samples = {};
    uint32_t m_sampleIdx = 0;
    uint32_t m_lastSampleIdx = 0;

    // ---- Previous-frame state for derivatives ----
    glm::fvec3 m_prevLocalLinVel = {};
    glm::fvec3 m_prevFlatAngVel = {};
    glm::fvec3 m_prevAngularVelocity = {};  // world-space, for drift calc
    XrTime m_prevSampleTime = 0;

    // ---- Smoothed signals (EMA-filtered) ----
    glm::fvec3 m_smoothedLinAccel = {};
    glm::fvec3 m_smoothedLinAccelSlow = {};
    float m_smoothedFlatAngAcc = 0.0f;
    float m_smoothedAngDrift = 0.0f;
    float m_smoothedFrameDt = 0.0f;

    // ---- Current-frame computed values (used across detection methods) ----
    glm::fvec3 m_localLinVel = {};
    glm::fvec3 m_localAngVel = {};
    glm::fvec3 m_localLinAccel = {};
    float m_flatAngAcc = 0.0f;
    float m_angularDrift = 0.0f;
    float m_handSpeed = 0.0f;

    // ---- Attack state machine ----
    AttackType m_lockedAttackType = AttackType::None;
    bool m_attackActive = false;
    bool m_isHitboxEnabled = false;

    float m_goodSwingAccumTime = 0.0f;
    float m_goodStabAccumTime = 0.0f;
    float m_badSampleAccumTime = 0.0f;

    XrTime m_lastGoodSwingTime = 0;
    XrTime m_lastGoodStabTime = 0;
    XrTime m_lockTime = 0;
    XrTime m_refractoryUntil = 0;

    glm::fvec3 m_lockedPosition = {};
    glm::fvec3 m_lockedAngle = {};

    // ---- Swing power tracking ----
    float m_swingPower = 0.0f;
    float m_peakAttackVelocity = 0.0f;

    // ---- Auto-calibration ----
    float m_calibratedArmLength = 0.55f;

    // ---- Weapon type ----
    WeaponType m_weaponType = WeaponType::LargeSword;

    // ---- Reference velocities for swing power normalization ----
    static constexpr float REFERENCE_SLASH_VELOCITY = 10.0f;  // rad/s for a strong slash
    static constexpr float REFERENCE_STAB_VELOCITY = 2.0f;    // m/s for a strong stab
};
