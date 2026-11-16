#include "end_effector_hardware/bus/linkerhand_l6.h"

#include <algorithm>
#include <cmath>

#include "end_effector_hardware/bus/params.h"
#include "end_effector_hardware/gripper_protocol/utils/ModbusConfig.h"
#include "robot_comm_modbus/rtu_frame.h"

namespace end_effector_hardware
{
    namespace rtu = robot_comm::modbus_rtu;
    using L6 = gripper_protocol::ModbusConfig::DexterousHand::L6;
    namespace LinkerHand = gripper_protocol::ModbusConfig::LinkerHand;

    namespace
    {
        constexpr int kAttempts = 5;
        constexpr double kChangeThreshold = 0.001;  // rad
        constexpr uint16_t kTorqueStart = 6;
        constexpr uint16_t kSpeedStart = 12;

        struct JointSpec
        {
            const char* key;
            int reg;
            double lower;
            double upper;
        };
        // Register order of the L6 protocol, and its limits (must match the URDF).
        const JointSpec kSpecs[] = {
            {"thumb_joint1", 0, L6::THUMB_JOINT1_LOWER, L6::THUMB_JOINT1_UPPER},
            {"thumb_joint2", 1, L6::THUMB_JOINT2_LOWER, L6::THUMB_JOINT2_UPPER},
            {"index_joint", 2, L6::INDEX_JOINT_LOWER, L6::INDEX_JOINT_UPPER},
            {"middle_joint", 3, L6::MIDDLE_JOINT_LOWER, L6::MIDDLE_JOINT_UPPER},
            {"ring_joint", 4, L6::RING_JOINT_LOWER, L6::RING_JOINT_UPPER},
            {"pinky_joint", 5, L6::PINKY_JOINT_LOWER, L6::PINKY_JOINT_UPPER},
        };
    }  // namespace

    bool LinkerHandL6::configure(const std::vector<std::string>& joint_names, const Params& p, std::string& error)
    {
        joints_.clear();
        std::array<bool, kJoints> taken{};
        for (const auto& name : joint_names)
        {
            std::string lower = name;
            std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
            for (const auto& spec : kSpecs)
            {
                if (lower.find(spec.key) == std::string::npos) continue;
                if (taken[spec.reg])
                {
                    error = "two joints map to " + std::string(spec.key) + ": " + name;
                    return false;
                }
                taken[spec.reg] = true;  // at most one joint per register, so at most kJoints joints
                register_of_[joints_.size()] = spec.reg;
                lower_[joints_.size()] = spec.lower;
                upper_[joints_.size()] = spec.upper;
                joints_.push_back(DeviceJoint{name});
                break;
            }
        }
        if (joints_.size() != kJoints)
        {
            error = "linkerhand_l6 needs the 6 joints thumb_joint1, thumb_joint2, index_joint, middle_joint, ring_joint, "
                    "pinky_joint; found " + std::to_string(joints_.size());
            return false;
        }
        const auto side = p.find("side");
        if (side == p.end() || (side->second != "left" && side->second != "right"))
        {
            error = "linkerhand_l6 needs side = left or right";
            return false;
        }
        long slave = side->second == "left" ? LinkerHand::LEFT_HAND_SLAVE_ID : LinkerHand::RIGHT_HAND_SLAVE_ID;
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

    void LinkerHandL6::setDynamics(double torque, double velocity)
    {
        torque_ = std::clamp(torque, 0.0, 1.0);
        velocity_ = std::clamp(velocity, 0.0, 1.0);
        dynamics_pending_ = true;
    }

    uint16_t LinkerHandL6::toRaw(std::size_t j, double radians) const
    {
        const double range = upper_[j] - lower_[j];
        if (range <= 0.0) return 128;
        const double normalized = std::clamp((radians - lower_[j]) / range, 0.0, 1.0);
        return static_cast<uint16_t>(255 - std::round(normalized * 255.0));
    }

    double LinkerHandL6::toRadians(std::size_t j, uint16_t raw) const
    {
        const double range = upper_[j] - lower_[j];
        if (range <= 0.0) return (lower_[j] + upper_[j]) / 2.0;
        return lower_[j] + (255 - (raw & 0xFF)) / 255.0 * range;
    }

    void LinkerHandL6::reset()
    {
        phase_ = Phase::ReadInitial;
        attempts_ = 0;
        dynamics_pending_ = true;
    }

    BusDevice::Request LinkerHandL6::writeRequest(bool with_dynamics)
    {
        std::vector<uint16_t> values(with_dynamics ? kJoints * 3 : kJoints, 0);
        for (std::size_t j = 0; j < kJoints; ++j)
        {
            in_flight_[j] = joints_[j].command;
            values[register_of_[j]] = toRaw(j, joints_[j].command);
        }
        if (with_dynamics)
        {
            const auto torque = static_cast<uint16_t>(std::lround(torque_ * 255.0));
            const auto speed = static_cast<uint16_t>(std::lround(velocity_ * 255.0));
            std::fill_n(values.begin() + kTorqueStart, kJoints, torque);
            std::fill_n(values.begin() + kSpeedStart, kJoints, speed);
        }
        return Request{rtu::writeRegisters(slave_, 0, values), rtu::replySpec(reply_timeout_),
                       with_dynamics ? kWriteAll : kWritePositions};
    }

    std::optional<BusDevice::Request> LinkerHandL6::next()
    {
        const auto read = [this] {
            return Request{rtu::readRegisters(slave_, rtu::kReadInputRegisters, 0, kJoints),
                           rtu::replySpec(reply_timeout_), kRead};
        };
        if (phase_ == Phase::ReadInitial) return read();
        if (phase_ == Phase::WriteInitial) return writeRequest(true);
        const bool finite = std::all_of(joints_.begin(), joints_.end(), [](const DeviceJoint& j) {
            return std::isfinite(j.command);
        });
        if (finite && dynamics_pending_) return writeRequest(true);
        if (finite)
        {
            for (std::size_t j = 0; j < kJoints; ++j)
            {
                if (std::abs(joints_[j].command - last_sent_[j]) > kChangeThreshold) return writeRequest(false);
            }
        }
        return read();
    }

    void LinkerHandL6::handle(uint32_t kind, const robot_comm::Completion& completion)
    {
        const bool delivered = completion.status == robot_comm::CompletionStatus::Ok;
        if (kind == kRead)
        {
            rtu::Result result;
            if (delivered)
            {
                result = rtu::parseReadReply(completion.reply, slave_, rtu::kReadInputRegisters, kJoints);
            }
            else
            {
                result.error = completion.error;
            }
            noteReply(result.ok);
            if (!result.ok)
            {
                if (phase_ == Phase::ReadInitial && ++attempts_ >= kAttempts)
                {
                    markFailed("initial position not read (slave " + std::to_string(slave_) + "): " + result.error);
                }
                return;
            }
            for (std::size_t j = 0; j < kJoints; ++j)
            {
                joints_[j].position = toRadians(j, result.registers[register_of_[j]]);
            }
            if (phase_ == Phase::ReadInitial)
            {
                for (auto& joint : joints_) joint.command = joint.position;  // hold where it is
                phase_ = Phase::WriteInitial;
                attempts_ = 0;
            }
            return;
        }
        const uint16_t count = kind == kWriteAll ? kJoints * 3 : kJoints;
        rtu::Result result;
        if (delivered)
        {
            result = rtu::parseWriteReply(completion.reply, slave_, rtu::kWriteMultipleRegisters, 0, count);
        }
        else
        {
            result.error = completion.error;
        }
        noteReply(result.ok);
        if (!result.ok)
        {
            if (phase_ == Phase::WriteInitial && ++attempts_ >= kAttempts)
            {
                markFailed("initial command not written (slave " + std::to_string(slave_) + "): " + result.error);
            }
            return;
        }
        last_sent_ = in_flight_;
        if (kind == kWriteAll) dynamics_pending_ = false;
        if (phase_ == Phase::WriteInitial)
        {
            phase_ = Phase::Running;
            markReady();
        }
    }
}  // namespace end_effector_hardware
