#pragma once

#include <array>

#include "xr_teleop/xr_types.hpp"

namespace xr_teleop
{
    /// Values published per frame besides poses (the /teleop/* contract consumed by XrInputHandler).
    struct EncodedControls
    {
        int event = 0;                       ///< /teleop/controller_state
        std::array<double, 6> thumbstick{};  ///< /teleop/thumbstick_axes: lx, ly, lgrip, rx, ry, rgrip
        double left_trigger_pull = 0.0;      ///< /teleop/trigger_values linear.x
        double right_trigger_pull = 0.0;     ///< /teleop/trigger_values angular.x
        double left_grip_value = 0.0;        ///< /teleop/grip_values linear.x
        double right_grip_value = 0.0;       ///< /teleop/grip_values angular.x
    };

    /// Button event codes, published every frame on /teleop/controller_state (key table: docs/使用手册.md
    /// section 6.1). 0 is sent on every frame without a key press, so it can never be a key.
    namespace event
    {
        constexpr int kNone = 0;
        constexpr int kFsmForward = 1;   ///< right A: HOME -> HOLD -> OCS2
        constexpr int kFsmBack = 2;      ///< left X: OCS2 / MOVEJ -> HOLD -> HOME, HOME <-> REST
        constexpr int kToggleFollow = 3; ///< right stick click: UPDATE (arms follow) <-> STORAGE
        constexpr int kToggleChassisDrive = 4; ///< left stick click: the sticks drive the base (only OCS2, base locked)
    } // namespace event

    /// The front keys are commands, each on its own key and fired when pressed; grips, triggers and
    /// sticks are analog inputs only. No chords, so a held grip or trigger never changes a key.
    class ButtonEventEncoder
    {
    public:
        static constexpr double kThumbstickDeadzone = 0.1;

        EncodedControls update(const TeleState& state);
        void reset() { *this = ButtonEventEncoder{}; }

    private:
        static double deadzone(double value);

        bool prev_a_ = false;
        bool prev_x_ = false;
        bool prev_right_stick_ = false;
        bool prev_left_stick_ = false;
    };
} // namespace xr_teleop
