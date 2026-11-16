#pragma once

#include <array>

#include "end_effector_hardware/bus/bus_device.h"

namespace end_effector_hardware
{
    /**
     * LinkerHand L6 dexterous hand, Modbus RTU (left 0x28, right 0x27). Six joints, matched by name: thumb_joint1,
     * thumb_joint2, index_joint, middle_joint, ring_joint, pinky_joint; radians within the L6 limits. Registers
     * (0-255, 255 = open): position 0-5 (FC04 read, FC16 write), torque 6-11, speed 12-17.
     *
     * As the earlier driver: initialization reads the position (5 attempts), then writes it back together with
     * torque / speed (5 attempts), so the hand does not move. Commands are written when a joint changes by more than
     * 0.001 rad; torque / speed only after setDynamics(). Writes wait for the acknowledgement.
     *
     * Parameters: side (left / right, required), slave_id (overrides side), torque, velocity (0..1, default 1),
     * reply_timeout_ms (default 100).
     */
    class LinkerHandL6 final : public BusDevice
    {
    public:
        static constexpr std::size_t kJoints = 6;

        explicit LinkerHandL6(rclcpp::Logger logger) : BusDevice("linkerhand_l6", std::move(logger)) {}

        bool configure(const std::vector<std::string>& joint_names, const Params& params, std::string& error) override;
        void setDynamics(double torque, double velocity) override;

        /** Register value (0-255) of joint `j` at `radians`, and back. */
        uint16_t toRaw(std::size_t j, double radians) const;
        double toRadians(std::size_t j, uint16_t raw) const;

    protected:
        std::optional<Request> next() override;
        void handle(uint32_t kind, const robot_comm::Completion& completion) override;
        void reset() override;

    private:
        enum Kind : uint32_t { kRead = 1, kWritePositions = 2, kWriteAll = 3 };
        enum class Phase { ReadInitial, WriteInitial, Running };

        Request writeRequest(bool with_dynamics);

        uint8_t slave_ = 0x27;
        double torque_ = 1.0;
        double velocity_ = 1.0;
        std::chrono::milliseconds reply_timeout_{100};
        std::array<int, kJoints> register_of_{};  ///< joint index -> register index
        std::array<double, kJoints> lower_{};
        std::array<double, kJoints> upper_{};

        Phase phase_ = Phase::ReadInitial;
        int attempts_ = 0;
        bool dynamics_pending_ = true;
        std::array<double, kJoints> last_sent_{};
        std::array<double, kJoints> in_flight_{};
    };
}  // namespace end_effector_hardware
