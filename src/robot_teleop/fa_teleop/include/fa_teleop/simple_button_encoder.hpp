#pragma once

#include <cstdint>

#include "fa_teleop/button_event_encoder.hpp"

namespace fa_teleop
{
    /// control_scheme:=simple: single keys mapped to the event codes VRInputHandler already knows
    /// (pair with vr_control_scheme:=simple). Key table: docs/VR遥操作.md section 5.1.
    /// Grips, triggers and the left stick produce no events; analog topics are unchanged.
    class SimpleButtonEncoder
    {
    public:
        static constexpr int64_t kLongPressNs = 800'000'000;  // 0.8 s

        /// @param timestamp_ns headset frame time (monotonic within a session)
        EncodedControls update(const TeleState& state, int64_t timestamp_ns);
        void reset() { *this = SimpleButtonEncoder{}; }
        bool recording() const { return recording_; }

    private:
        ButtonEventEncoder analog_;  ///< reused for the analog / thumbstick outputs only
        bool prev_a_ = false, prev_x_ = false, prev_y_ = false, prev_b_ = false, prev_rstick_ = false;
        bool y_chorded_ = false;  ///< Y used in a chord during this press (no long-press delete)
        bool b_chorded_ = false;  ///< B used in a chord during this press (no recording toggle)
        bool y_long_fired_ = false;
        int64_t y_down_ns_ = 0;
        bool recording_ = false;
    };
} // namespace fa_teleop
