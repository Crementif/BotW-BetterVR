#pragma once


class RumbleManager {
public:
    RumbleManager(XrSession session, XrAction haptic_action, XrPath subaction_path = XR_NULL_PATH) : m_session(session), m_haptic_action(haptic_action), m_subaction_path(subaction_path) {
        m_update_thread = std::thread(&RumbleManager::update_thread, this);
    }

    ~RumbleManager() {
        m_shutdown.store(true);
        if (m_update_thread.joinable()) {
            m_update_thread.join();
        }
    }

    void initializeXrPathsAndStartTime(XrInstance instance)
    {
        xrStringToPath(instance, "/user/hand/left", &m_handSubactionPaths[0]);
        xrStringToPath(instance, "/user/hand/right", &m_handSubactionPaths[1]);
        m_startTime = std::chrono::steady_clock::now();
    }

    // pattern: uint8_t* rumble pattern
    // length: length in bits
    void controlMotor(uint8_t* pattern, uint8_t length) {
        length = std::min<uint8_t>(length, 120);

        if (length == 0) {
            stopMotor();
            return;
        }

        push_rumble(pattern, length);
    }

    void stopMotor() {
        std::scoped_lock lock(m_rumble_mutex);
        while (!m_rumble_queue.empty()) {
            m_rumble_queue.pop();
        }
        m_parser = 0;
        m_current_rumbling = false;
        stop_haptic();
    }

    void stopInputsRumble(int hand, RumbleType rumbleType) {
        std::scoped_lock lock(m_inputs_rumble_mutex);
        auto& state = m_hapticStates[hand];
        if (state.active && state.inputRumble && rumbleType == state.params.rumbleType) {
            state.endTime = std::chrono::steady_clock::now();
            state.active = false;
        }
    }

    void enqueueInputsRumbleCommand(const RumbleParameters& rumbleParameters) {
        std::scoped_lock lock(m_inputs_rumble_mutex);
        if (rumbleParameters.prioritizeThisRumble) {
            m_inputs_rumble_queue.push(rumbleParameters);
            return;
        }

        if (m_inputs_rumble_queue.empty())
            m_inputs_rumble_queue.push(rumbleParameters);
    }

    void updateHaptics() {
        std::scoped_lock lock(m_inputs_rumble_mutex);
        auto now = std::chrono::steady_clock::now();

        while (!m_inputs_rumble_queue.empty()) {
            RumbleParameters cmd = m_inputs_rumble_queue.front();
            auto& state = m_hapticStates[cmd.hand]; 

            // Priority Check: Don't overwrite if a high-priority rumble or the same rumble is still running
            bool sameRumble = state.active && state.params.rumbleType == cmd.rumbleType && now < state.endTime;
            bool highPriorityRumble = state.active && state.inputRumble && (!cmd.prioritizeThisRumble || now < state.endTime);
            if ( sameRumble || highPriorityRumble) {
                m_inputs_rumble_queue.pop();
                continue;
            }

            state.active = true;
            state.params = cmd;
            state.startTime = now;
            state.endTime = now + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::duration<double>(cmd.effectDuration)
                                  );
            state.inputRumble = cmd.prioritizeThisRumble;

            m_inputs_rumble_queue.pop();
        }

        for (int i = 0; i < 2; ++i) {
            auto& state = m_hapticStates[i];
            if (!state.active) continue;

            if (now >= state.endTime) {
                state.active = false;
                continue;
            }

            double currentAmplitude = state.params.amplitude;
            double currentFrequency = state.params.frequency;
            double elapsed = std::chrono::duration<double>(now - state.startTime).count();
            double progress = 0.0;
            double envelope = 1.0;
            double wave = 0.0;
            double omega = 0.0;

            switch (state.params.rumbleType) {
                case RumbleType::Raising:
                    progress = glm::clamp(elapsed / state.params.effectDuration, 0.0, 1.0);
                    envelope = progress * progress; //exponential optional. Need testing
                    currentAmplitude *= envelope;
                    currentFrequency *= envelope;

                    // if rumble is still running at 95%, extend duration until the input is released
                    // prevents the rumble from restarting from zero if the input is held longer than duration
                    if (state.params.keepRumblingOnEffectEnd && progress > 0.95f)
                    {
                        state.params.effectDuration += 0.05;
                        state.endTime += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.05));
                    }
                    break;
                case RumbleType::Falling:
                    progress = glm::clamp(elapsed / state.params.effectDuration, 0.0, 1.0);
                    envelope = (1.0 - progress) * (1.0 - progress); //exponential optional. Need testing
                    currentAmplitude *= envelope;
                    currentFrequency *= envelope;

                    if (state.params.keepRumblingOnEffectEnd && progress > 0.95f)
                    {
                        state.params.effectDuration += 0.05;
                        state.endTime += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.05));
                    }
                    break;
                case RumbleType::OscillationSmooth:
                    omega = 2.0 * glm::pi<double>() * state.params.oscillationFrequency;
                    wave = (std::sin(elapsed * omega) + 1.0) * 0.5;
                    currentAmplitude *= wave;
                    currentFrequency *= wave;
                    break;
                case RumbleType::OscillationFallingSawtoothWave:
                    progress = fmod(elapsed * state.params.oscillationFrequency, 1.0);
                    wave = (1.0 - progress) * (1.0 - progress); //exponential optional. Need testing
                    currentAmplitude *= wave; 
                    currentFrequency *= wave;
                    break;
                case RumbleType::OscillationRaisingSawtoothWave:
                    progress = fmod(elapsed * state.params.oscillationFrequency, 1.0);
                    wave = progress * progress;
                    currentAmplitude *= wave;
                    currentFrequency *= wave;
                    break;
            }

            XrHapticVibration vibration = { XR_TYPE_HAPTIC_VIBRATION };
            vibration.next = nullptr;
            // Pulse duration: Set slightly longer than the frame time
            // to ensure continuous feel without gaps.
            vibration.duration = (XrDuration)(0.03 * 1e9);
            vibration.frequency = (float)currentFrequency;
            vibration.amplitude = (float)currentAmplitude;

            XrHapticActionInfo haptic_info = { XR_TYPE_HAPTIC_ACTION_INFO };
            haptic_info.action = m_haptic_action;
            haptic_info.subactionPath = m_handSubactionPaths[state.params.hand];

            xrApplyHapticFeedback(m_session, &haptic_info, (const XrHapticBaseHeader*)&vibration);
        }
    }

private:
    bool push_rumble(uint8_t* pattern, uint8_t length) {
        if (pattern == nullptr || length == 0) {
            stopMotor();
            return true;
        }

        std::scoped_lock lock(m_rumble_mutex);
        if (m_rumble_queue.size() >= 5) {
            return false;
        }

        std::vector<bool> bitset;
        int len = length;
        int byte_idx = 0;
        while (len > 0) {
            uint8_t p = pattern[byte_idx];
            for (int j = 0; j < 8 && j < len; j += 2) {
                bool set = (p & (3 << j)) != 0;
                bitset.push_back(set);
            }
            ++byte_idx;
            len -= 8;
        }

        m_rumble_queue.emplace(std::move(bitset));
        return true;
    }

    void update_thread() {
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::milliseconds(1000 / 60);

        auto next_tick = clock::now();
        while (!m_shutdown.load(std::memory_order_relaxed)) {
            {
                std::unique_lock lock(m_rumble_mutex);
                if (m_rumble_queue.empty()) {
                    if (m_current_rumbling) {
                        stop_haptic();
                        m_current_rumbling = false;
                    }
                    m_parser = 0;
                }
                else {
                    const auto& current_pattern = m_rumble_queue.front();
                    bool should_rumble = current_pattern[m_parser];
                    if (should_rumble != m_current_rumbling) {
                        if (should_rumble) {
                            apply_haptic_infinite();
                        }
                        else {
                            stop_haptic();
                        }
                        m_current_rumbling = should_rumble;
                    }
                    ++m_parser;
                    if (m_parser >= current_pattern.size()) {
                        m_rumble_queue.pop();
                        m_parser = 0;
                    }
                }
            }
            next_tick += period;
            std::this_thread::sleep_until(next_tick);
        }
    }

void apply_haptic_infinite() {
        XrHapticVibration vibration = {};
        vibration.type = XR_TYPE_HAPTIC_VIBRATION;
        vibration.next = nullptr;
        vibration.duration = XR_INFINITE_DURATION;
        vibration.frequency = XR_FREQUENCY_UNSPECIFIED;
        vibration.amplitude = 1.0f;

        XrHapticActionInfo haptic_info = {};
        haptic_info.type = XR_TYPE_HAPTIC_ACTION_INFO;
        haptic_info.next = nullptr;
        haptic_info.action = m_haptic_action;
        haptic_info.subactionPath = m_subaction_path;

        checkXRResult(xrApplyHapticFeedback(m_session, &haptic_info, (const XrHapticBaseHeader*)&vibration), "Failed to start rumble");

        m_haptic_active = true;
    }

    void stop_haptic() {
        XrHapticActionInfo haptic_info = {};
        haptic_info.type = XR_TYPE_HAPTIC_ACTION_INFO;
        haptic_info.next = nullptr;
        haptic_info.action = m_haptic_action;
        haptic_info.subactionPath = m_subaction_path;

        checkXRResult(xrStopHapticFeedback(m_session, &haptic_info), "Failed to stop rumble");

        m_haptic_active = false;
    }

    XrInstance m_instance;
    XrSession m_session;
    XrAction m_haptic_action;
    XrPath m_subaction_path;
    XrPath m_handSubactionPaths[2];

    std::queue<std::vector<bool>> m_rumble_queue;
    std::mutex m_rumble_mutex;
    size_t m_parser = 0;
    bool m_current_rumbling = false;
    std::atomic<bool> m_shutdown{ false };
    std::thread m_update_thread;

    bool m_haptic_active = false;

    std::mutex m_inputs_rumble_mutex;
    std::queue<RumbleParameters> m_inputs_rumble_queue;
    struct ActiveHaptic {
        bool active = false;
        bool inputRumble = false;
        std::chrono::steady_clock::time_point startTime;
        std::chrono::steady_clock::time_point endTime;
        RumbleParameters params;
    };
    ActiveHaptic m_hapticStates[2]; // 0 = left, 1 = right
    std::chrono::steady_clock::time_point m_startTime;
};