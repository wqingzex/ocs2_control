#pragma once

#include "end_effector_hardware/bus/bus_device.h"

namespace end_effector_hardware
{
    /**
     * Jodell RG75 parallel gripper, Modbus RTU (slave 0x09 by default). One joint whose name contains "gripper";
     * position is the opening in metres (0 .. 0.038372). As the earlier drivers: an activation write and the motion
     * commands are sent without waiting for an acknowledgement (the gripper may not send one); the link is confirmed by
     * the first status read (FC04 @0x07D0), retried up to 3 times. A command is sent when it changes.
     *
     * Parameters: slave_id (default 9), torque, velocity (0..1, default 1), reply_timeout_ms (default 100).
     */
    class JodellRg75 final : public BusDevice
    {
    public:
        explicit JodellRg75(rclcpp::Logger logger) : BusDevice("jodell_rg75", std::move(logger)) {}

        bool configure(const std::vector<std::string>& joint_names, const Params& params, std::string& error) override;
        void setDynamics(double torque, double velocity) override;

    protected:
        std::optional<Request> next() override;
        void handle(uint32_t kind, const robot_comm::Completion& completion) override;
        void reset() override;

    private:
        enum Kind : uint32_t { kActivate = 1, kStatus = 2, kMove = 3 };
        enum class Phase { Activate, FirstStatus, Running };

        uint8_t slave_ = 0x09;
        double torque_ = 1.0;
        double velocity_ = 1.0;
        std::chrono::milliseconds reply_timeout_{100};
        Phase phase_ = Phase::Activate;
        int status_attempts_ = 0;
        bool initialized_ = false;  ///< the first status was read (after going offline it never fails, it retries)
        double last_sent_ = 0.0;   ///< last command sent
        double in_flight_ = 0.0;   ///< command of the move request in flight
    };
}  // namespace end_effector_hardware
