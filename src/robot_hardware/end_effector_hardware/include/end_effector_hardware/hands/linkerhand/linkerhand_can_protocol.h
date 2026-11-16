#pragma once

// LinkerHand CAN protocol (classic CAN, 1 Mbit/s): frames are built and parsed by plain functions on
// robot_comm::CanFrame, so the protocol is tested without ROS and without a bus (test_linkerhand_can_protocol.cpp).

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "robot_comm/can.h"

namespace end_effector_hardware::linkerhand
{
    constexpr std::size_t kMaxJointCount = 7;
    constexpr std::size_t kFingerCount = 5;
    constexpr std::size_t kMaxTactileRows = 12;
    constexpr std::size_t kMaxTactileColumns = 6;

    /// One byte per joint in protocol order (positions, torque or velocity limits).
    using RawJoints = std::array<uint8_t, kMaxJointCount>;

    constexpr uint32_t kRightHandCanId = 0x27;
    constexpr uint32_t kLeftHandCanId = 0x28;
    constexpr uint32_t kCanBitrate = 1'000'000;

    constexpr uint8_t kPositionCommand = 0x01;    ///< command frame and position request / feedback
    constexpr uint8_t kMaxTorqueCommand = 0x02;   ///< per-joint torque limit, 0..255
    constexpr uint8_t kMaxVelocityCommand = 0x05; ///< per-joint velocity limit, 0..255
    constexpr uint8_t kTactileRequestLen = 2;

    enum class Finger : uint8_t
    {
        Thumb = 0,
        Index,
        Middle,
        Ring,
        Pinky,
    };

    constexpr std::array<uint8_t, kFingerCount> kTactileCommands{0xB1, 0xB2, 0xB3, 0xB4, 0xB5};

    /// What differs between LinkerHand models on the same protocol.
    struct ModelConfig
    {
        const char* hand_type;  ///< robot.end_effector key, e.g. "linkerhand_l6"
        const char* model_name; ///< short name used in topics and logs, e.g. "l6"
        std::size_t joint_count;
        std::array<std::size_t, kMaxJointCount> protocol_to_ros; ///< protocol slot i -> ros2_control joint index
        uint8_t tactile_layout;
        std::size_t tactile_frame_count;
        std::size_t tactile_values_per_frame;
        std::array<double, kMaxJointCount> upper_limits; ///< rad; the lower limit is 0 for every joint
    };

    /// The model for a hand_type ("linkerhand_l6"), or nullptr if the CAN driver does not support it.
    const ModelConfig* findModel(const std::string& hand_type);

    /// The CAN ID of a hand side ("left" / "right").
    uint32_t canIdForSide(const std::string& side);

    const char* fingerName(Finger finger);
    bool fingerFromTactileCommand(uint8_t command, Finger& finger);

    /// Request the current joint positions (the hand answers with a position frame).
    robot_comm::CanFrame makePositionRequest(uint32_t can_id);

    /// A frame of `command` followed by one byte per joint, in protocol order: position, torque or velocity.
    robot_comm::CanFrame makeJointFrame(uint32_t can_id, uint8_t command, const RawJoints& values,
                                        std::size_t joint_count);

    /// A position feedback frame from `can_id`: fills `raw` (protocol order) and returns true.
    bool parsePositionFeedback(const robot_comm::CanFrame& frame, uint32_t can_id, std::size_t joint_count,
                               RawJoints& raw);

    robot_comm::CanFrame makeTactileRequest(uint32_t can_id, Finger finger, uint8_t layout);

    /// One chunk of a finger's tactile matrix: `values_per_frame` bytes at (row, column).
    struct TactileChunk
    {
        Finger finger = Finger::Thumb;
        std::size_t row = 0;
        std::size_t column = 0;
        const uint8_t* values = nullptr;
    };

    /// A tactile frame from `can_id` for `model`: fills `chunk` (pointing into `frame`) and returns true.
    bool parseTactileFrame(const robot_comm::CanFrame& frame, uint32_t can_id, const ModelConfig& model,
                           TactileChunk& chunk);

    /// Joint angle <-> protocol byte: 255 is the lower limit (open), 0 the upper limit.
    uint8_t radiansToRaw(double radians, double lower, double upper);
    double rawToRadians(uint8_t raw, double lower, double upper);

    /// A 0..1 torque / velocity scale as a protocol byte.
    uint8_t scaleToRaw(double scale);
} // namespace end_effector_hardware::linkerhand
