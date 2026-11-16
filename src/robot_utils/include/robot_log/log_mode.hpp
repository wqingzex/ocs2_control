#pragma once

// The two logging modes of the robot software: normal (INFO and above) and debug (the robot software at DEBUG, the noisy ROS internals kept at INFO).
//
// The mode is fixed for a run. It comes from log.mode of the config; the supervisor hands it to every process it
// starts in the environment variable ROBOT_LOG_MODE ("normal" or "debug"); each component applies it once while it
// initialises. Nothing changes it while the system runs: to change the mode, change the config and restart.

#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <rcutils/error_handling.h>
#include <rcutils/logging.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace robot_log
{
    inline constexpr const char* kLogModeEnvironment = "ROBOT_LOG_MODE";

    /// Is this run in debug mode? ROBOT_LOG_MODE=debug. Not set, or anything but normal / debug (reported): normal.
    inline bool debugMode()
    {
        const char* text = std::getenv(kLogModeEnvironment);
        if (text == nullptr || *text == '\0' || std::strcmp(text, "normal") == 0)
        {
            return false;
        }
        if (std::strcmp(text, "debug") == 0)
        {
            return true;
        }
        RCLCPP_WARN(rclcpp::get_logger("robot_log"), "%s='%s' is not a log mode (normal | debug); using normal",
                    kLogModeEnvironment, text);
        return false;
    }

    /// ROS internals that log at DEBUG on every message or cycle: in a debug run they stay at INFO, or they bury
    /// everything else (measured in the simulation: with the default level at DEBUG, controller_manager and rcl wrote
    /// 140 000 lines and 20 MB in the first minute, the robot software itself a few dozen). Children inherit from their
    /// parent by the dot in the name (controller_manager.pal_statistics follows controller_manager); a name with :: is not a child.
    inline const std::vector<std::string>& noisyRosLoggers()
    {
        static const std::vector<std::string> names{"controller_manager", "rcl", "rcl_action", "rclcpp", "rclcpp_action",
                                                    "rmw_fastrtps_cpp", "rmw_fastrtps_shared_cpp", "pluginlib",
                                                    "pluginlib.ClassLoader", "resource_manager",
                                                    "ControllerManager::utils"};
        return names;
    }

    /// Applies the mode of this run. In debug the default level of the process goes to DEBUG, so every logger that
    /// has no level of its own (the robot software's: node loggers and the ones libraries create by name) follows it,
    /// with no list of ours to keep. The noisy ROS internals above are first pinned at INFO, once per process.
    /// rcutils' per-logger table is not thread safe: that batch is the only thing written to it, at the first call,
    /// and the default level is a single integer. In normal nothing is touched, so a level given with
    /// --ros-args --log-level still counts. Call while the component initialises. Returns whether this run is debug.
    inline bool applyLogMode()
    {
        const bool debug = debugMode();
        if (!debug)
        {
            return false;
        }
        static std::once_flag pinned;
        std::call_once(pinned, []() {
            for (const auto& name : noisyRosLoggers())
            {
                if (rcutils_logging_set_logger_level(name.c_str(), RCUTILS_LOG_SEVERITY_INFO) != RCUTILS_RET_OK)
                {
                    rcutils_reset_error();
                    RCLCPP_WARN(rclcpp::get_logger("robot_log"), "could not keep '%s' at INFO in debug mode", name.c_str());
                }
            }
        });
        rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
        return true;
    }
} // namespace robot_log
