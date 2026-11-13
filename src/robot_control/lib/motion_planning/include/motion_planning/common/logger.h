#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace planning
{
    /// The ROS logger of this library: RCLCPP_ERROR_STREAM(planning::logger(), "joint " << i << " has no limit");
    inline const rclcpp::Logger& logger()
    {
        static const rclcpp::Logger instance = rclcpp::get_logger("motion_planning");
        return instance;
    }
} // namespace planning
