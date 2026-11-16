#include "end_effector_hardware/bus/jodell_rg75.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/logging.hpp>

#include "end_effector_hardware/bus/params.h"
#include "end_effector_hardware/gripper_protocol/JodellGripper.h"
#include "robot_comm_modbus/rtu_frame.h"

namespace end_effector_hardware
{
    namespace rtu = robot_comm::modbus_rtu;
    using gripper_protocol::CommandChangeDetector;
    using gripper_protocol::JodellCommandBuilder;
    using gripper_protocol::PositionConverter;
    using gripper_protocol::TorqueConverter;
    using gripper_protocol::VelocityConverter;
    namespace Jodell = gripper_protocol::ModbusConfig::Jodell;

    namespace
    {
        constexpr int kStatusAttempts = 3;

        robot_comm::ReplySpec noReply()
        {
            robot_comm::ReplySpec spec;
            spec.expect_reply = false;
            return spec;
        }
    }  // namespace

    bool JodellRg75::configure(const std::vector<std::string>& joint_names, const Params& p, std::string& error)
    {
        joints_.clear();
        for (const auto& name : joint_names)
        {
            std::string lower = name;
            std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
            if (lower.find("gripper") != std::string::npos) joints_.push_back(DeviceJoint{name});
        }
        if (joints_.size() != 1)
        {
            error = "jodell_rg75 drives exactly one joint whose name contains 'gripper', found " +
                    std::to_string(joints_.size());
            return false;
        }
        long slave = Jodell::SLAVE_ID;
        long timeout = reply_timeout_.count();
        if (!params::integer(p, "slave_id", 1, 247, slave, error) || !params::ratio(p, "torque", torque_, error) ||
            !params::ratio(p, "velocity", velocity_, error) ||
            !params::integer(p, "reply_timeout_ms", 1, 5000, timeout, error) || !configureOffline(p, error))
        {
            return false;
        }
        slave_ = static_cast<uint8_t>(slave);
        reply_timeout_ = std::chrono::milliseconds(timeout);
        return true;
    }

    void JodellRg75::setDynamics(double torque, double velocity)
    {
        torque_ = std::clamp(torque, 0.0, 1.0);
        velocity_ = std::clamp(velocity, 0.0, 1.0);
        last_sent_ = std::numeric_limits<double>::quiet_NaN();  // resend the position with the new force / speed
    }

    void JodellRg75::reset()
    {
        phase_ = Phase::Activate;
        status_attempts_ = 0;
        initialized_ = false;
    }

    std::optional<BusDevice::Request> JodellRg75::next()
    {
        if (phase_ == Phase::Running && !online())
        {
            // Power cut or unplugged: once it is back it is not activated and ignores moves.
            RCLCPP_WARN(logger_, "jodell_rg75: offline; activating again until it answers, then holding");
            phase_ = Phase::Activate;
        }
        if (phase_ == Phase::Activate)
        {
            const std::vector<uint16_t> value{Jodell::INIT_VALUE};
            return Request{rtu::writeRegisters(slave_, Jodell::INIT_REG_ADDR, value), noReply(), kActivate};
        }
        if (phase_ == Phase::Running)
        {
            const double command = joints_[0].command;
            if (std::isfinite(command) &&
                (!std::isfinite(last_sent_) || CommandChangeDetector::hasChanged(command, last_sent_)))
            {
                const auto values = JodellCommandBuilder::buildCommand(
                    PositionConverter::Jodell::normalizedToJodell(command),
                    TorqueConverter::Jodell::normalizedToJodell(torque_),
                    VelocityConverter::Jodell::normalizedToLowByte(velocity_));
                in_flight_ = command;
                return Request{rtu::writeRegisters(slave_, Jodell::POSITION_REG_ADDR, values), noReply(), kMove};
            }
        }
        return Request{rtu::readRegisters(slave_, rtu::kReadInputRegisters, Jodell::STATUS_REG_ADDR,
                                          Jodell::STATUS_REG_COUNT),
                       rtu::replySpec(reply_timeout_), kStatus};
    }

    void JodellRg75::handle(uint32_t kind, const robot_comm::Completion& completion)
    {
        if (kind == kActivate)
        {
            if (completion.status != robot_comm::CompletionStatus::Ok)
            {
                RCLCPP_WARN(logger_, "jodell_rg75: activation write not sent: %s", completion.error.c_str());
            }
            phase_ = Phase::FirstStatus;
            return;
        }
        if (kind == kMove)
        {
            if (completion.status == robot_comm::CompletionStatus::Ok) last_sent_ = in_flight_;
            return;
        }
        // Status: register 1 high byte = position, register 2 = force (high byte) / speed (low byte).
        rtu::Result result;
        if (completion.status == robot_comm::CompletionStatus::Ok)
        {
            result = rtu::parseReadReply(completion.reply, slave_, rtu::kReadInputRegisters, Jodell::STATUS_REG_COUNT);
        }
        else
        {
            result.error = completion.error;
        }
        noteReply(result.ok);
        if (!result.ok)
        {
            if (phase_ == Phase::FirstStatus && initialized_)
            {
                phase_ = Phase::Activate;  // still away: activate again before the next status read
                return;
            }
            if (phase_ == Phase::FirstStatus && ++status_attempts_ >= kStatusAttempts)
            {
                markFailed("not responding (slave " + std::to_string(slave_) + "): " + result.error +
                           "; check wiring, baud rate, slave_id");
            }
            return;
        }
        DeviceJoint& joint = joints_[0];
        joint.position = PositionConverter::Jodell::jodellToNormalized(result.registers[1] >> 8);
        joint.velocity = (result.registers[2] & 0xFF) / 255.0;
        joint.effort = (result.registers[2] >> 8) / 255.0;
        if (phase_ == Phase::FirstStatus)
        {
            joint.command = joint.position;  // hold where it is
            last_sent_ = joint.position;
            phase_ = Phase::Running;
            initialized_ = true;
            markReady();
        }
    }
}  // namespace end_effector_hardware
