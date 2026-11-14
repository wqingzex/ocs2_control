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

    int ButtonEventEncoder::gripDirectionEvent(Side& side, bool grip, double x, double y, int up_event)
    {
        const double magnitude = std::max(std::abs(x), std::abs(y));
        if (!grip || magnitude <= kGripDirectionReset)
        {
            side.direction_latched = false;
            return event::kNone;
        }
        if (side.direction_latched || magnitude < kGripDirectionTrigger)
        {
            return event::kNone;
        }
        // Larger axis wins, Y on ties. Pico: pushing the stick forward (up) is negative Y.
        if (std::abs(y) >= std::abs(x))
        {
            return y < 0.0 ? up_event : up_event + 1;  // up / down
        }
        return x > 0.0 ? up_event + 3 : up_event + 2;  // right / left
    }

    EncodedControls ButtonEventEncoder::update(const TeleState& s)
    {
        Side& L = left_;
        Side& R = right_;

        const double lx = deadzone(s.left_axis_x);
        const double ly = deadzone(s.left_axis_y);
        const double rx = deadzone(s.right_axis_x);
        const double ry = deadzone(s.right_axis_y);

        // Rising edges of stick clicks (prev_stick is updated at the very end of the frame).
        const bool left_stick_rise = s.left_stick_click && !L.prev_stick;
        const bool right_stick_rise = s.right_stick_click && !R.prev_stick;

        const int left_direction = gripDirectionEvent(L, s.left_grip, lx, ly, event::kLeftGripUp);
        const int right_direction = gripDirectionEvent(R, s.right_grip, rx, ry, event::kRightGripUp);

        // Grip edges; standalone = released without being consumed by a chord.
        const bool left_grip_was_pressed = L.prev_grip;
        const bool left_grip_released = !s.left_grip && L.prev_grip;
        const bool right_grip_released = !s.right_grip && R.prev_grip;
        const bool left_grip_standalone = left_grip_released && !L.grip_consumed;
        const bool right_grip_standalone = right_grip_released && !R.grip_consumed;
        L.prev_grip = s.left_grip;
        R.prev_grip = s.right_grip;

        const bool left_stick_released = !s.left_stick_click && L.prev_stick;
        const bool right_stick_released = !s.right_stick_click && R.prev_stick;
        const bool left_stick_standalone = left_stick_released && !L.stick_consumed;
        const bool right_stick_standalone = right_stick_released && !R.stick_consumed;

        const bool left_y_rise = s.left_y && !L.prev_secondary;
        const bool right_b_rise = s.right_b && !R.prev_secondary;
        const bool left_y_released = !s.left_y && L.prev_secondary;
        const bool right_b_released = !s.right_b && R.prev_secondary;
        const bool left_y_standalone = left_y_released && !L.secondary_consumed;
        const bool right_b_standalone = right_b_released && !R.secondary_consumed;

        const bool left_trigger_rise = s.left_trigger && !L.prev_trigger;
        const bool right_trigger_rise = s.right_trigger && !R.prev_trigger;
        const bool left_x_rise = s.left_x && !L.prev_primary;
        const bool right_a_rise = s.right_a && !R.prev_primary;

        L.prev_secondary = s.left_y;
        R.prev_secondary = s.right_b;
        L.prev_trigger = s.left_trigger;
        R.prev_trigger = s.right_trigger;
        L.prev_primary = s.left_x;
        R.prev_primary = s.right_a;

        // Event selection, highest priority first.
        const bool yb_chord = (left_y_rise && s.right_b) || (right_b_rise && s.left_y);
        const bool both_grips = s.left_grip && s.right_grip;
        int ev = event::kNone;
        if (s.left_y && right_a_rise)
        {
            ev = event::kLeftYRightA;
            L.secondary_consumed = true;
        }
        else if (both_grips && yb_chord)
        {
            ev = event::kBothGripsYB;
            L.grip_consumed = R.grip_consumed = true;
            L.secondary_consumed = R.secondary_consumed = true;
        }
        else if (yb_chord)
        {
            ev = event::kLeftYRightB;
            L.secondary_consumed = R.secondary_consumed = true;
        }
        else if (s.left_grip && right_a_rise)
        {
            ev = event::kLeftGripRightA;
            L.grip_consumed = true;
        }
        else if (s.left_grip && right_b_rise)
        {
            ev = event::kLeftGripRightB;
            L.grip_consumed = true;
            R.secondary_consumed = true;
        }
        else if (s.right_grip && right_stick_rise)
        {
            ev = event::kRightGripRightStick;
            R.grip_consumed = true;
            R.stick_consumed = true;
        }
        else if (s.left_grip && left_y_rise)
        {
            ev = event::kLeftGripLeftY;
            L.grip_consumed = true;
            L.secondary_consumed = true;
        }
        else if (s.left_grip && left_grip_was_pressed && left_stick_rise)
        {
            ev = event::kLeftGripLeftStick;
            L.grip_consumed = true;
            L.stick_consumed = true;
        }
        else if ((left_stick_rise && s.right_stick_click) || (right_stick_rise && s.left_stick_click))
        {
            ev = event::kBothSticks;
            L.stick_consumed = R.stick_consumed = true;
        }
        else if (s.left_grip && left_trigger_rise)
        {
            ev = event::kLeftGripTrigger;
            L.grip_consumed = true;
        }
        else if (s.right_grip && right_trigger_rise)
        {
            ev = event::kRightGripTrigger;
            R.grip_consumed = true;
        }
        else if (left_direction != event::kNone)
        {
            ev = left_direction;
            L.direction_latched = true;
            L.grip_consumed = true;
        }
        else if (right_direction != event::kNone)
        {
            ev = right_direction;
            R.direction_latched = true;
            R.grip_consumed = true;
        }
        else if (left_trigger_rise) { ev = event::kLeftTrigger; }
        else if (right_trigger_rise) { ev = event::kRightTrigger; }
        else if (right_a_rise) { ev = event::kRightA; }
        else if (left_x_rise) { ev = event::kLeftX; }
        else if (left_stick_standalone) { ev = event::kLeftStick; }
        else if (left_y_standalone) { ev = event::kLeftY; }
        else if (right_stick_standalone) { ev = event::kRightStick; }
        else if (right_b_standalone) { ev = event::kRightB; }
        else if (left_grip_standalone) { ev = event::kLeftGrip; }
        else if (right_grip_standalone) { ev = event::kRightGrip; }

        // Consumption flags are cleared on release, after this frame's decision.
        if (left_grip_released) { L.grip_consumed = false; }
        if (right_grip_released) { R.grip_consumed = false; }
        if (left_stick_released) { L.stick_consumed = false; }
        if (right_stick_released) { R.stick_consumed = false; }
        if (left_y_released) { L.secondary_consumed = false; }
        if (right_b_released) { R.secondary_consumed = false; }
        L.prev_stick = s.left_stick_click;
        R.prev_stick = s.right_stick_click;

        EncodedControls out;
        out.event = ev;
        out.thumbstick = {lx, ly, s.left_grip ? 1.0 : 0.0, rx, ry, s.right_grip ? 1.0 : 0.0};
        out.left_trigger_pull = s.left_trigger_pull;
        out.right_trigger_pull = s.right_trigger_pull;
        out.left_grip_value = s.left_grip_value;
        out.right_grip_value = s.right_grip_value;
        return out;
    }
} // namespace xr_teleop
