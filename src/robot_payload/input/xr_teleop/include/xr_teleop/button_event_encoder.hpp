#pragma once

#include <array>

#include "xr_teleop/xr_types.hpp"

namespace xr_teleop
{
    /// Values published per frame besides poses (the /xr/* contract consumed by VRInputHandler).
    struct EncodedControls
    {
        int event = 0;                       ///< /teleop/controller_state
        std::array<double, 6> thumbstick{};  ///< /teleop/thumbstick_axes: lx, ly, lgrip, rx, ry, rgrip
        double left_trigger_pull = 0.0;      ///< /teleop/trigger_values linear.x
        double right_trigger_pull = 0.0;     ///< /teleop/trigger_values angular.x
        double left_grip_value = 0.0;        ///< /teleop/grip_values linear.x
        double right_grip_value = 0.0;       ///< /teleop/grip_values angular.x
    };

    /// Button event codes (see docs/VR遥操作.md for the full table).
    namespace event
    {
        constexpr int kNone = 0;
        constexpr int kLeftGrip = 2;
        constexpr int kLeftY = 3;
        constexpr int kRightStick = 4;
        constexpr int kRightGrip = 5;
        constexpr int kRightB = 6;
        constexpr int kLeftStick = 7;
        constexpr int kLeftTrigger = 9;
        constexpr int kRightTrigger = 10;
        constexpr int kRightA = 11;
        constexpr int kLeftX = 12;
        constexpr int kLeftGripTrigger = 13;
        constexpr int kRightGripTrigger = 14;
        constexpr int kLeftYRightB = 15;
        constexpr int kBothGripsYB = 16;
        constexpr int kBothSticks = 20;
        constexpr int kLeftGripUp = 21;   ///< 22 down, 23 left, 24 right
        constexpr int kRightGripUp = 25;  ///< 26 down, 27 left, 28 right
        constexpr int kLeftGripLeftStick = 29;
        constexpr int kLeftGripRightA = 50;
        constexpr int kLeftGripRightB = 51;
        constexpr int kRightGripRightStick = 52;
        constexpr int kLeftGripLeftY = 53;
        constexpr int kLeftYRightA = 54;
    } // namespace event

    /// Legacy encoder, a port of fa-py-libraries xr_target_node.py @041159c (golden-trace tested).
    /// Chords fire on the rising edge of their last key; modifier keys (grip, stick, Y, B) fire
    /// on release only if no chord used them during that press.
    class ButtonEventEncoder
    {
    public:
        static constexpr double kThumbstickDeadzone = 0.1;
        static constexpr double kGripDirectionTrigger = 0.7;
        static constexpr double kGripDirectionReset = 0.3;

        EncodedControls update(const TeleState& state);
        void reset() { *this = ButtonEventEncoder{}; }

    private:
        struct Side
        {
            bool prev_grip = false;
            bool prev_stick = false;
            bool prev_trigger = false;
            bool prev_primary = false;    ///< X / A
            bool prev_secondary = false;  ///< Y / B
            bool grip_consumed = false;
            bool stick_consumed = false;
            bool secondary_consumed = false;  ///< Y / B consumed by a chord
            bool direction_latched = false;
        };

        static double deadzone(double value);
        /// Candidate grip+stick direction event; latching is applied only when it is published.
        static int gripDirectionEvent(Side& side, bool grip, double x, double y, int up_event);

        Side left_;
        Side right_;
    };
} // namespace xr_teleop
