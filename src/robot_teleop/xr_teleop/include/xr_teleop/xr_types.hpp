#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace xr_teleop
{
    /// Pose as delivered by XRoboToolkit: x, y, z, qx, qy, qz, qw (OpenXR frame).
    using Pose7 = std::array<double, 7>;

    /// Raw controller sample from the PC Service JSON (values in SDK conventions).
    struct ControllerSample
    {
        Pose7 pose{0, 0, 0, 0, 0, 0, 1};
        double trigger = 0.0;  ///< 0 = released, 1 = fully pulled
        double grip = 0.0;     ///< 0 = released, 1 = fully squeezed
        double axis_x = 0.0;   ///< right = positive
        double axis_y = 0.0;   ///< SDK: forward = positive (flipped before publishing)
        bool axis_click = false;
        bool primary_button = false;    ///< X (left) / A (right)
        bool secondary_button = false;  ///< Y (left) / B (right)
        bool menu_button = false;
    };

    /// One device-state frame from the headset.
    struct XrFrame
    {
        std::string device_id;
        int64_t timestamp_ns = 0;
        Pose7 head{0, 0, 0, 0, 0, 0, 1};
        ControllerSample left;
        ControllerSample right;
        bool has_head = false;
        bool has_left = false;
        bool has_right = false;
    };

    /// Controller state in the conventions the downstream /xr/* contract expects
    /// (same as the former Python XRTargetNode TeleStateData).
    struct TeleState
    {
        bool left_trigger = false;   ///< trigger > kTriggerPressThreshold
        bool right_trigger = false;
        bool left_grip = false;      ///< grip > kGripPressThreshold
        bool right_grip = false;
        double left_grip_value = 0.0;   ///< clamped to [0, 1]
        double right_grip_value = 0.0;
        double left_trigger_pull = 0.0;  ///< clamped to [0, 1]
        double right_trigger_pull = 0.0;
        bool left_stick_click = false;
        bool right_stick_click = false;
        double left_axis_x = 0.0;   ///< right = positive
        double left_axis_y = 0.0;   ///< WebXR convention: forward = negative
        double right_axis_x = 0.0;
        double right_axis_y = 0.0;
        bool left_x = false;   ///< left primary
        bool left_y = false;   ///< left secondary
        bool right_a = false;  ///< right primary
        bool right_b = false;  ///< right secondary
    };

    constexpr double kTriggerPressThreshold = 0.1;
    constexpr double kGripPressThreshold = 0.5;

    /// Map raw SDK samples to TeleState (thresholds, clamping, thumbstick Y flip).
    TeleState toTeleState(const ControllerSample& left, const ControllerSample& right);
} // namespace xr_teleop
