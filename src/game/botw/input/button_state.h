#pragma once

#include <cstdint>

class ButtonState {
public:
    enum class Event { None, Pressed, ShortPress, LongPress };
    Event lastEvent = Event::None;

    bool IsDown() const { return m_phase == Phase::Pressed || m_phase == Phase::LongHeld; }
    bool IsLongHeld() const { return m_phase == Phase::LongHeld; }

    void Cancel() {
        m_phase = Phase::Blocked;
        lastEvent = Event::None;
    }

    void Update(bool active, bool down, int64_t sampleTime) {
        lastEvent = Event::None;
        if (!active || sampleTime <= 0 || sampleTime < m_sampleTime) {
            Cancel();
            m_sampleTime = sampleTime;
            return;
        }
        if (sampleTime == m_sampleTime)
            return;
        m_sampleTime = sampleTime;

        if (m_phase == Phase::Blocked) {
            if (!down)
                m_phase = Phase::Released;
            return;
        }
        if (!down) {
            if (m_phase == Phase::Pressed)
                lastEvent = Event::ShortPress;
            m_phase = Phase::Released;
        }
        else if (m_phase == Phase::Released) {
            lastEvent = Event::Pressed;
            m_pressTime = sampleTime;
            m_phase = Phase::Pressed;
        }
        else if (m_phase == Phase::Pressed && sampleTime - m_pressTime >= 250'000'000) {
            m_phase = Phase::LongHeld;
            lastEvent = Event::LongPress;
        }
    }

private:
    enum class Phase { Released, Pressed, LongHeld, Blocked };
    Phase m_phase = Phase::Released;
    int64_t m_pressTime = 0;
    int64_t m_sampleTime = -1;
};
