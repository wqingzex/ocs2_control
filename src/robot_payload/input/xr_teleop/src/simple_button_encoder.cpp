#include "xr_teleop/simple_button_encoder.hpp"

namespace xr_teleop
{
    EncodedControls SimpleButtonEncoder::update(const TeleState& s, int64_t timestamp_ns)
    {
        EncodedControls out = analog_.update(s);  // analog / thumbstick values are scheme independent
        out.event = event::kNone;

        const bool a_rise = s.right_a && !prev_a_;
        const bool x_rise = s.left_x && !prev_x_;
        const bool y_rise = s.left_y && !prev_y_;
        const bool b_rise = s.right_b && !prev_b_;
        const bool b_released = !s.right_b && prev_b_;
        const bool rstick_rise = s.right_stick_click && !prev_rstick_;

        if (y_rise)
        {
            y_down_ns_ = timestamp_ns;
            y_chorded_ = false;
            y_long_fired_ = false;
        }
        if (b_rise)
        {
            b_chorded_ = false;
        }

        if (a_rise && s.left_y)
        {
            out.event = event::kLeftYRightA;  // 54 enter MOVEJ
            y_chorded_ = true;
        }
        else if ((y_rise && s.right_b) || (b_rise && s.left_y))
        {
            out.event = event::kLeftYRightB;  // 15
            y_chorded_ = b_chorded_ = true;
        }
        else if (a_rise) { out.event = event::kRightA; }
        else if (x_rise) { out.event = event::kLeftX; }
        else if (rstick_rise) { out.event = event::kRightGripRightStick; }  // 52 intervention
        else if (b_released && !b_chorded_)
        {
            out.event = recording_ ? event::kLeftGripRightB : event::kLeftGripRightA;  // 51 stop / 50 start
            recording_ = !recording_;
        }
        else if (s.left_y && !y_chorded_ && !y_long_fired_ && timestamp_ns - y_down_ns_ >= kLongPressNs)
        {
            out.event = event::kLeftGripLeftY;  // 53 delete
            y_long_fired_ = true;
        }

        prev_a_ = s.right_a;
        prev_x_ = s.left_x;
        prev_y_ = s.left_y;
        prev_b_ = s.right_b;
        prev_rstick_ = s.right_stick_click;
        return out;
    }
} // namespace xr_teleop
