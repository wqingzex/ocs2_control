#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "robot_comm/can.h"

/// The Juxie joint-module protocol ("关节模组使用协议简易说明书 V1.0"), as pure functions on robot_comm::CanFrame.
/// No ROS and no socket: the driver (jx_hardware.cpp) wires it to a CanBus, and the tests run it without hardware.
namespace juxie_hardware::protocol
{
    constexpr uint32_t kSyncFrameId = 0x80;       ///< sync frame (also the base of the emergency frames 0x80 + id)
    constexpr uint32_t kEmergencyBase = 0x80;
    constexpr uint32_t kMultiControlId = 0x200;   ///< one frame that controls up to 8 motors
    constexpr uint32_t kFeedbackBase = 0x300;     ///< motor state feedback: 0x300 + id
    constexpr uint32_t kEnableBase = 0x600;       ///< SDO write to enable a motor: 0x600 + id
    constexpr std::size_t kMaxMotors = 8;
    constexpr std::size_t kMultiControlLen = 64;  ///< CAN FD frame, 7 bytes per motor, ids at byte 56 + i

    constexpr double kAngleMin = -180.0;          ///< degrees
    constexpr double kAngleMax = 180.0;
    constexpr int16_t kRawMin = -32668;           ///< CSP position range, corresponds to -180 degrees
    constexpr int16_t kRawMax = 32668;

    constexpr uint8_t kCtrlEnable = 0x80;         ///< bit 7: enable
    constexpr uint8_t kCtrlBrakeRelease = 0x40;   ///< bit 6: release the brake
    constexpr uint8_t kCtrlClearError = 0x20;     ///< bit 5: reset errors
    constexpr uint8_t kCtrlModeCsp = 0x03 << 1;   ///< bits 4:1: cyclic synchronous position mode
    constexpr uint8_t kCtrlWordCsp = kCtrlEnable | kCtrlBrakeRelease | kCtrlModeCsp;
    constexpr uint8_t kCtrlWordCspClear = kCtrlWordCsp | kCtrlClearError;

    /// Position count <-> degrees. rawToAngle rounds to 0.001 degree like the driver always did.
    int16_t angleToRaw(double angle_deg);
    double rawToAngle(int16_t raw);

    robot_comm::CanFrame makeSyncFrame();
    robot_comm::CanFrame makeEnableFrame(uint8_t motor_id);

    /// The control byte of the multi-motor frame: depends on the feedback length the motors answer with
    /// (7 bytes: 0xD8 for the first frame, 0xD0 after; 12 bytes: CSP control word, with "clear error" on the first).
    uint8_t controlByte(uint8_t feedback_len, bool first_command);

    /// Multi-motor control frame (id 0x200, 64 bytes). `raw` and `ids` have one entry per motor, at most 8.
    robot_comm::CanFrame makeMultiMotorFrame(uint8_t control_byte, const std::vector<int16_t>& raw,
                                             const std::vector<uint8_t>& ids);

    // ---- rate limiting -------------------------------------------------------------------------------------------

    struct StepLimits
    {
        int32_t max_step = 25;  ///< largest position increment per frame (nct)
        int32_t max_accel = 5;  ///< largest change of the increment between two frames; 0 = no acceleration limit
    };

    struct StepResult
    {
        int16_t sent = 0;       ///< the position to send
        int32_t delta = 0;      ///< sent - last
    };

    /// Move `last` towards `target`: at most max_step per frame, the increment changing by at most max_accel,
    /// never past the target, always inside [kRawMin, kRawMax].
    StepResult limitStep(int32_t target, int32_t last, int32_t last_delta, const StepLimits& limits);

    // ---- receiving -------------------------------------------------------------------------------------------------

    enum class FrameKind
    {
        Feedback,  ///< state of one of our motors (0x300 + id)
        Emergency, ///< emergency frame of one of our motors (0x80 + id)
        Other      ///< not ours, or not a plain data frame
    };

    struct Classified
    {
        FrameKind kind = FrameKind::Other;
        uint8_t motor_id = 0;
    };

    Classified classify(const robot_comm::CanFrame& frame, const std::vector<uint8_t>& motor_ids);

    struct MotorFeedback
    {
        double position_rad = 0.0;
        double velocity_rad_s = 0.0;
        double effort = 0.0;      ///< Iq current in mA (12-byte protocol); 0 for the 7-byte protocol
        bool enabled = false;
        bool fault = false;
        uint16_t error_code = 0;  ///< 12-byte protocol only
        uint8_t status = 0;
        uint8_t mode = 0;         ///< 12-byte protocol only: mode feedback (0x03 = CSP)
    };

    enum class DecodeResult
    {
        Ok,
        TooShort,       ///< the frame has fewer bytes than the protocol needs
        UnsupportedLen  ///< protocol_len is neither 7 nor 12
    };

    /// `protocol_len` is the feedback length the motors use: 12 (current protocol) or 7 (older firmware).
    DecodeResult decodeFeedback(const robot_comm::CanFrame& frame, uint8_t protocol_len, MotorFeedback& out);

    // ---- error codes ---------------------------------------------------------------------------------------------

    struct FaultInfo
    {
        uint16_t fault_code;
        const char* message;
    };

    /// 12-byte protocol: the 16-bit error word of a feedback or emergency frame.
    std::vector<FaultInfo> decodeErrorWord(uint16_t error_code_raw);

    /// 7-byte protocol: the two error bytes of an emergency frame.
    std::vector<FaultInfo> decodeEmergencyBytes(uint8_t byte0, uint8_t byte1);

    /// The value published on /Base_HardwareError: 0x3 << 24 | motor_id << 16 | fault_code.
    inline int64_t hardwareErrorCode(uint8_t motor_id, uint16_t fault_code)
    {
        return static_cast<int64_t>((0x3u << 24) | (static_cast<uint32_t>(motor_id) << 16) | fault_code);
    }

    constexpr uint16_t kUnknownFault = 0x0100;
    constexpr int64_t kFeedbackTimeoutError = 0x300200; ///< published when not every motor answered at start-up
} // namespace juxie_hardware::protocol
