#include "xr_teleop/button_event_encoder.hpp"

#include <algorithm>
#include <cmath>

namespace xr_teleop
{
    namespace
    {
        double clamp01(double value) { return std::clamp(value, 0.0, 1.0); }
    } // namespace

    TeleState toTeleState(const ControllerSample& left, const ControllerSample& right)
    {
        TeleState s;
        s.left_trigger = left.trigger > kTriggerPressThreshold;
        s.right_trigger = right.trigger > kTriggerPressThreshold;
        s.left_grip = left.grip > kGripPressThreshold;
        s.right_grip = right.grip > kGripPressThreshold;
        s.left_grip_value = clamp01(left.grip);
        s.right_grip_value = clamp01(right.grip);
        // Former pipeline: SDK [0,1] -> TeleData [10=release, 0=full] -> pull [0,1].
        // Kept verbatim so values match the golden trace bit for bit.
        s.left_trigger_pull = clamp01(1.0 - (10.0 - clamp01(left.trigger) * 10.0) / 10.0);
        s.right_trigger_pull = clamp01(1.0 - (10.0 - clamp01(right.trigger) * 10.0) / 10.0);
        s.left_stick_click = left.axis_click;
        s.right_stick_click = right.axis_click;
        // App SDK: forward is positive; the /teleop/thumbstick_axes contract (WebXR) uses negative.
        s.left_axis_x = left.axis_x;
        s.left_axis_y = -left.axis_y;
        s.right_axis_x = right.axis_x;
        s.right_axis_y = -right.axis_y;
        s.left_x = left.primary_button;
        s.left_y = left.secondary_button;
        s.right_a = right.primary_button;
        s.right_b = right.secondary_button;
        return s;
    }

    double ButtonEventEncoder::deadzone(double value)
    {
        return std::abs(value) < kThumbstickDeadzone ? 0.0 : value;
    }

    EncodedControls ButtonEventEncoder::update(const TeleState& s)
    {
        const bool a_pressed = s.right_a && !prev_a_;
        const bool x_pressed = s.left_x && !prev_x_;
        const bool right_stick_pressed = s.right_stick_click && !prev_right_stick_;
        const bool left_stick_pressed = s.left_stick_click && !prev_left_stick_;
        prev_a_ = s.right_a;
        prev_x_ = s.left_x;
        prev_right_stick_ = s.right_stick_click;
        prev_left_stick_ = s.left_stick_click;

        EncodedControls out;
        // One Int32 per frame: keys pressed in the same frame are rare; the first one listed wins.
        if (a_pressed) { out.event = event::kFsmForward; }
        else if (x_pressed) { out.event = event::kFsmBack; }
        else if (right_stick_pressed) { out.event = event::kToggleFollow; }
        else if (left_stick_pressed) { out.event = event::kToggleChassisDrive; }

        out.thumbstick = {deadzone(s.left_axis_x), deadzone(s.left_axis_y), s.left_grip ? 1.0 : 0.0,
                          deadzone(s.right_axis_x), deadzone(s.right_axis_y), s.right_grip ? 1.0 : 0.0};
        out.left_trigger_pull = s.left_trigger_pull;
        out.right_trigger_pull = s.right_trigger_pull;
        out.left_grip_value = s.left_grip_value;
        out.right_grip_value = s.right_grip_value;
        return out;
    }
} // namespace xr_teleop
