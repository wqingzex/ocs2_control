#pragma once

// The record types of the flight recorder. Keep in step with config/flight_types.yaml (a test compares them): that file
// says what each value of a record is.

#include <cstdint>

namespace robot_log::flight
{
    // 1xx: XR bridge
    inline constexpr std::uint16_t kXrFrameIn = 101;      // an accepted headset frame
    inline constexpr std::uint16_t kXrPublish = 102;      // that frame published
    // 2xx: teleop_command
    inline constexpr std::uint16_t kTelVrIn = 201;        // a VR pose received
    inline constexpr std::uint16_t kTelTargetOut = 202;   // a target pose published to a controller
    // 3xx: controllers
    inline constexpr std::uint16_t kCtlTargetIn = 301;    // a joint target received
    inline constexpr std::uint16_t kCtlState = 302;       // the measured joint positions of a cycle
    inline constexpr std::uint16_t kCtlCommand = 303;     // the joint positions commanded in a cycle
    inline constexpr std::uint16_t kCtlFsm = 304;         // a state change of the state machine
    // 4xx: hardware (the plugin that talks to the motors)
    inline constexpr std::uint16_t kHwRead = 401;         // the joint positions read from the hardware in a cycle
    inline constexpr std::uint16_t kHwWrite = 402;        // the joint positions written to the hardware in a cycle
} // namespace robot_log::flight
