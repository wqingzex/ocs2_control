#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "end_effector_hardware/bus/bus_device.h"

namespace end_effector_hardware
{
    /** The XHand1 RS485 wire format (custom, 3 Mbit/s): [55 AA][src][dst][cmd][len u16 LE][data][CRC16-CCITT LE]. */
    namespace xhand1
    {
        constexpr std::size_t kJoints = 12;
        constexpr std::size_t kHeader = 7;
        constexpr std::size_t kCrc = 2;
        constexpr std::size_t kCommandBytesPerJoint = 24;
        constexpr std::size_t kStateBytesPerJoint = 24;
        constexpr std::size_t kTactileBytes = 384 * 5;
        constexpr std::size_t kMaxReply = kHeader + kJoints * kStateBytesPerJoint + kTactileBytes + kCrc;
        constexpr uint8_t kRealtime = 0x02;
        constexpr uint16_t kModeDisabled = 0;  ///< no torque: what initialization sends to read the hand
        constexpr uint16_t kModePosition = 3;

        struct Gains
        {
            int16_t kp = 100;
            int16_t ki = 0;
            int16_t kd = 0;
        };

        /** CRC-16/CCITT with initial value 0 (as the hand computes it). */
        uint16_t crc16(const uint8_t* data, std::size_t size);
        /** One realtime frame: per joint its target position [rad], torque limit and the control mode. */
        std::vector<uint8_t> realtimeFrame(uint8_t host_id, uint8_t hand_id, const Gains& gains,
                                           const std::array<double, kJoints>& positions,
                                           const std::array<uint16_t, kJoints>& torque_limits, uint16_t mode);
        /** Total length of the reply once its header is in; 0 while it is not; the bytes so far if it is no frame. */
        std::size_t replyLength(const std::vector<uint8_t>& received);

        struct State
        {
            std::array<double, kJoints> position{};  ///< rad
            std::array<double, kJoints> effort{};    ///< the hand's raw torque reading
        };
        /** The realtime reply of hand `hand_id` to host `host_id` (CRC checked); nullopt if it is not one. */
        std::optional<State> parseReply(const std::vector<uint8_t>& frame, uint8_t host_id, uint8_t hand_id);
    }  // namespace xhand1

    /**
     * XHand1 dexterous hand (12 joints) on a robot_comm bus, as the earlier XHand1Rs485Hardware:
     *
     *  - joints matched by name: thumb_joint1..3, index_joint1..3, middle_joint1..2, ring_joint1..2, pinky_joint1..2
     *    (hand joint ids 0..11 in that order); positions clamped to the hand's limits;
     *  - initialization sends zero-torque frames (mode 0) until the hand answers, then holds where it is;
     *  - running, every exchange sends the commands (mode 3, torque limit = torque_limit * torque) and reads the joint
     *    state back (the reply also carries the tactile data, about 2.2 kB); with velocity > 0 the target moves at most
     *    30 rad/s * velocity from the measured position per exchange;
     *  - without a good reply for 200 ms (offline) it goes back to the zero-torque frames and, once the hand answers,
     *    holds the position it reports.
     *
     * Parameters: hand_id (0..127, default 0), host_id (default 0xFE), kp / ki / kd (default 100 / 0 / 0),
     * torque_limit (0..300, default 300), torque (default 0.8), velocity (default 0: no speed limit),
     * reply_timeout_ms (default 50).
     */
    class XHand1 final : public BusDevice
    {
    public:
        explicit XHand1(rclcpp::Logger logger);
        bool configure(const std::vector<std::string>& joint_names, const Params& params, std::string& error) override;
        void setDynamics(double torque, double velocity) override;

    protected:
        std::optional<Request> next() override;
        void handle(uint32_t kind, const robot_comm::Completion& completion) override;
        void reset() override;

    private:
        enum Kind : uint32_t { kProbe = 1, kCommand = 2 };
        enum class Phase { Probe, Running };
        Request request(const std::array<double, xhand1::kJoints>& positions,
                        const std::array<uint16_t, xhand1::kJoints>& torque_limits, uint16_t mode, Kind kind) const;
        std::array<double, xhand1::kJoints> limitedTargets(double elapsed_s) const;
        void takeState(const xhand1::State& state);

        uint8_t host_id_ = 0xFE;
        uint8_t hand_id_ = 0;
        xhand1::Gains gains_;
        uint16_t torque_limit_ = 300;
        double torque_ = 0.8;
        double velocity_ = 0.0;
        std::chrono::milliseconds reply_timeout_{50};
        std::array<std::size_t, xhand1::kJoints> id_of_{};  ///< joints_ index -> hand joint id
        std::array<double, xhand1::kJoints> lower_{};      ///< by joints_ index
        std::array<double, xhand1::kJoints> upper_{};

        Phase phase_ = Phase::Probe;
        bool initialized_ = false;  ///< the first probe succeeded (later probes, after going offline, never fail)
        int attempts_ = 0;
        bool has_state_ = false;
        std::chrono::steady_clock::time_point state_time_{};
        std::chrono::steady_clock::time_point last_command_time_{};
    };
}  // namespace end_effector_hardware
