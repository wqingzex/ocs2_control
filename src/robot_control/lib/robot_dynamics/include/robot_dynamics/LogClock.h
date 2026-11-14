#pragma once

#include <rclcpp/clock.hpp>

namespace robot_dynamics
{
    /// The clock for RCLCPP_*_THROTTLE in code that has no node clock at hand (trajectory managers, planners).
    /// A steady clock: it keeps running when simulation time pauses and never jumps, so a throttled message
    /// is neither lost nor repeated because of the time source. One instance for the whole process.
    inline rclcpp::Clock& steadyLogClock()
    {
        static rclcpp::Clock clock(RCL_STEADY_TIME);
        return clock;
    }
} // namespace robot_dynamics
