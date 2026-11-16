#include "end_effector_hardware/bus/xhand1.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <rclcpp/logging.hpp>

#include "end_effector_hardware/bus/params.h"

namespace end_effector_hardware
{
    namespace xhand1
    {
        namespace
        {
            constexpr uint8_t kHead0 = 0x55;
            constexpr uint8_t kHead1 = 0xAA;

            void putU16(std::vector<uint8_t>& out, uint16_t v)
            {
                out.push_back(static_cast<uint8_t>(v & 0xFF));
                out.push_back(static_cast<uint8_t>(v >> 8));
            }

            void putFloat(std::vector<uint8_t>& out, float v)
            {
                uint32_t raw = 0;
                std::memcpy(&raw, &v, sizeof(raw));
                for (int shift = 0; shift < 32; shift += 8) out.push_back(static_cast<uint8_t>(raw >> shift));
            }

            uint16_t getU16(const std::vector<uint8_t>& in, std::size_t at)
            {
                return static_cast<uint16_t>(in[at] | (in[at + 1] << 8));
            }

            float getFloat(const std::vector<uint8_t>& in, std::size_t at)
            {
                const uint32_t raw = static_cast<uint32_t>(in[at]) | (static_cast<uint32_t>(in[at + 1]) << 8) |
                                     (static_cast<uint32_t>(in[at + 2]) << 16) |
                                     (static_cast<uint32_t>(in[at + 3]) << 24);
                float v = 0.0F;
                std::memcpy(&v, &raw, sizeof(v));
                return v;
            }

            uint8_t board(uint8_t hand_id) { return static_cast<uint8_t>(hand_id | 0x80); }
        }  // namespace

        uint16_t crc16(const uint8_t* data, std::size_t size)
        {
            uint16_t crc = 0;
            for (std::size_t i = 0; i < size; ++i)
            {
                crc ^= static_cast<uint16_t>(data[i] << 8);
                for (int bit = 0; bit < 8; ++bit)
                {
                    crc = static_cast<uint16_t>((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
                }
            }
            return crc;
        }

        std::vector<uint8_t> realtimeFrame(uint8_t host_id, uint8_t hand_id, const Gains& gains,
                                           const std::array<double, kJoints>& positions,
                                           const std::array<uint16_t, kJoints>& torque_limits, uint16_t mode)
        {
            std::vector<uint8_t> frame{kHead0, kHead1, host_id, board(hand_id), kRealtime};
            frame.reserve(kHeader + kJoints * kCommandBytesPerJoint + kCrc);
            putU16(frame, static_cast<uint16_t>(kJoints * kCommandBytesPerJoint));
            for (std::size_t id = 0; id < kJoints; ++id)
            {
                putU16(frame, static_cast<uint16_t>(id));
                putU16(frame, static_cast<uint16_t>(gains.kp));
                putU16(frame, static_cast<uint16_t>(gains.ki));
                putU16(frame, static_cast<uint16_t>(gains.kd));
                putFloat(frame, static_cast<float>(positions[id]));
                putU16(frame, torque_limits[id]);
                putU16(frame, mode);
                for (int reserved = 0; reserved < 4; ++reserved) putU16(frame, 0);
            }
            putU16(frame, crc16(frame.data(), frame.size()));
            return frame;
        }

        std::size_t replyLength(const std::vector<uint8_t>& received)
        {
            if (received.size() >= 1 && received[0] != kHead0) return received.size();  // not a frame: end it
            if (received.size() >= 2 && received[1] != kHead1) return received.size();
            if (received.size() < kHeader) return 0;
            const std::size_t length = kHeader + getU16(received, 5) + kCrc;
            return length > kMaxReply ? received.size() : length;
        }

        std::optional<State> parseReply(const std::vector<uint8_t>& frame, uint8_t host_id, uint8_t hand_id)
        {
            constexpr std::size_t kMin = kHeader + kJoints * kStateBytesPerJoint + kCrc;
            if (frame.size() < kMin || frame[0] != kHead0 || frame[1] != kHead1 || frame[2] != board(hand_id) ||
                frame[3] != host_id || frame[4] != kRealtime)
            {
                return std::nullopt;
            }
            const std::size_t data = getU16(frame, 5);
            if (data < kJoints * kStateBytesPerJoint || frame.size() != kHeader + data + kCrc) return std::nullopt;
            if (getU16(frame, frame.size() - kCrc) != crc16(frame.data(), frame.size() - kCrc)) return std::nullopt;
            State state;
            for (std::size_t record = 0; record < kJoints; ++record)
            {
                const std::size_t at = kHeader + record * kStateBytesPerJoint;
                const std::size_t id = frame[at];
                if (id >= kJoints) continue;
                state.position[id] = getFloat(frame, at + 2);
                state.effort[id] = getU16(frame, at + 6);
            }
            return state;
        }
    }  // namespace xhand1

    namespace
    {
        constexpr double kMaxVelocity = 30.0;  ///< rad/s at velocity 1
        constexpr int kProbeAttempts = 20;
        constexpr auto kOffline = std::chrono::milliseconds(200);

        struct JointSpec
        {
            const char* suffix;
            double lower;
            double upper;
        };
        // By hand joint id.
        constexpr std::array<JointSpec, xhand1::kJoints> kSpecs{{
            {"thumb_joint1", 0.0, 1.832},  {"thumb_joint2", -0.698, 1.57}, {"thumb_joint3", 0.0, 1.57},
            {"index_joint1", -0.174, 0.174}, {"index_joint2", 0.0, 1.919}, {"index_joint3", 0.0, 1.919},
            {"middle_joint1", 0.0, 1.919}, {"middle_joint2", 0.0, 1.919}, {"ring_joint1", 0.0, 1.919},
            {"ring_joint2", 0.0, 1.919},   {"pinky_joint1", 0.0, 1.919},  {"pinky_joint2", 0.0, 1.919},
        }};

        bool endsWith(const std::string& s, const std::string& suffix)
        {
            return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
        }

        bool int16Param(const Params& p, const char* key, int16_t& out, std::string& error)
        {
            long v = out;
            if (!params::integer(p, key, std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::max(), v,
                                 error))
            {
                return false;
            }
            out = static_cast<int16_t>(v);
            return true;
        }
    }  // namespace

    XHand1::XHand1(rclcpp::Logger logger) : BusDevice("xhand1", std::move(logger)) { offline_after_ = kOffline; }

    bool XHand1::configure(const std::vector<std::string>& joint_names, const Params& p, std::string& error)
    {
        joints_.clear();
        std::array<bool, xhand1::kJoints> seen{};
        for (const auto& name : joint_names)
        {
            for (std::size_t id = 0; id < xhand1::kJoints && joints_.size() < xhand1::kJoints; ++id)
            {
                if (seen[id] || !endsWith(name, kSpecs[id].suffix)) continue;
                seen[id] = true;
                id_of_[joints_.size()] = id;
                lower_[joints_.size()] = kSpecs[id].lower;
                upper_[joints_.size()] = kSpecs[id].upper;
                joints_.push_back(DeviceJoint{name});
                break;
            }
        }
        if (!std::all_of(seen.begin(), seen.end(), [](bool b) { return b; }))
        {
            error = "xhand1 needs the 12 joints thumb_joint1..3, index_joint1..3, middle_joint1..2, ring_joint1..2, "
                    "pinky_joint1..2; found " + std::to_string(joints_.size());
            return false;
        }
        long hand_id = hand_id_;
        long host_id = host_id_;
        long torque_limit = torque_limit_;
        long timeout = reply_timeout_.count();
        if (!params::integer(p, "hand_id", 0, 127, hand_id, error) ||
            !params::integer(p, "host_id", 0, 255, host_id, error) || !int16Param(p, "kp", gains_.kp, error) ||
            !int16Param(p, "ki", gains_.ki, error) || !int16Param(p, "kd", gains_.kd, error) ||
            !params::integer(p, "torque_limit", 0, 300, torque_limit, error) ||
            !params::ratio(p, "torque", torque_, error) || !params::ratio(p, "velocity", velocity_, error) ||
            !params::integer(p, "reply_timeout_ms", 1, 1000, timeout, error) || !configureOffline(p, error))
        {
            return false;
        }
        hand_id_ = static_cast<uint8_t>(hand_id);
        host_id_ = static_cast<uint8_t>(host_id);
        torque_limit_ = static_cast<uint16_t>(torque_limit);
        reply_timeout_ = std::chrono::milliseconds(timeout);
        return true;
    }

    void XHand1::setDynamics(double torque, double velocity)
    {
        torque_ = std::clamp(torque, 0.0, 1.0);
        velocity_ = std::clamp(velocity, 0.0, 1.0);
    }

    void XHand1::reset()
    {
        phase_ = Phase::Probe;
        initialized_ = false;
        attempts_ = 0;
        has_state_ = false;
    }

    BusDevice::Request XHand1::request(const std::array<double, xhand1::kJoints>& positions,
                                       const std::array<uint16_t, xhand1::kJoints>& torque_limits, uint16_t mode,
                                       Kind kind) const
    {
        // Joint order on the wire is the hand's id order.
        std::array<double, xhand1::kJoints> by_id{};
        for (std::size_t j = 0; j < xhand1::kJoints; ++j) by_id[id_of_[j]] = positions[j];
        robot_comm::ReplySpec reply;
        reply.frame_length = xhand1::replyLength;
        reply.max_size = xhand1::kMaxReply;
        reply.timeout = reply_timeout_;
        return Request{xhand1::realtimeFrame(host_id_, hand_id_, gains_, by_id, torque_limits, mode), std::move(reply),
                       kind};
    }

    std::array<double, xhand1::kJoints> XHand1::limitedTargets(double elapsed_s) const
    {
        std::array<double, xhand1::kJoints> out{};
        const double max_step = kMaxVelocity * velocity_ * std::max(0.0, elapsed_s);
        for (std::size_t j = 0; j < xhand1::kJoints; ++j)
        {
            const double target = std::clamp(joints_[j].command, lower_[j], upper_[j]);
            const double current = joints_[j].position;
            double value = target;
            if (velocity_ > 0.0 && std::abs(target - current) > max_step)
            {
                value = current + std::copysign(max_step, target - current);
            }
            out[j] = std::clamp(value, lower_[j], upper_[j]);
        }
        return out;
    }

    std::optional<BusDevice::Request> XHand1::next()
    {
        const auto now = std::chrono::steady_clock::now();
        if (phase_ == Phase::Running && !online())
        {
            RCLCPP_WARN(logger_, "xhand1: no reply for %lld ms; zero torque until it answers, then holding",
                        static_cast<long long>(offline_after_.count()));
            phase_ = Phase::Probe;
        }
        if (phase_ == Phase::Probe)
        {
            std::array<double, xhand1::kJoints> hold{};
            for (std::size_t j = 0; j < xhand1::kJoints; ++j) hold[j] = joints_[j].position;
            return request(hold, {}, xhand1::kModeDisabled, kProbe);
        }
        const bool finite = std::all_of(joints_.begin(), joints_.end(),
                                        [](const DeviceJoint& j) { return std::isfinite(j.command); });
        if (!finite)
        {
            for (auto& joint : joints_) joint.command = joint.position;
        }
        const double elapsed = std::chrono::duration<double>(now - last_command_time_).count();
        last_command_time_ = now;
        std::array<uint16_t, xhand1::kJoints> limits{};
        limits.fill(static_cast<uint16_t>(std::lround(torque_limit_ * torque_)));
        return request(limitedTargets(elapsed), limits, xhand1::kModePosition, kCommand);
    }

    void XHand1::takeState(const xhand1::State& state)
    {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - state_time_).count();
        for (std::size_t j = 0; j < xhand1::kJoints; ++j)
        {
            const double position = state.position[id_of_[j]];
            joints_[j].velocity = has_state_ && dt > 1e-6 ? (position - joints_[j].position) / dt : 0.0;
            joints_[j].position = position;
            joints_[j].effort = state.effort[id_of_[j]];
        }
        state_time_ = now;
        has_state_ = true;
    }

    void XHand1::handle(uint32_t kind, const robot_comm::Completion& completion)
    {
        std::optional<xhand1::State> state;
        if (completion.status == robot_comm::CompletionStatus::Ok)
        {
            state = xhand1::parseReply(completion.reply, host_id_, hand_id_);
        }
        noteReply(state.has_value());
        if (!state)
        {
            if (kind == kProbe && !initialized_ && ++attempts_ >= kProbeAttempts)
            {
                markFailed("no realtime reply from hand id " + std::to_string(hand_id_) + " after " +
                           std::to_string(kProbeAttempts) + " zero-torque frames" +
                           (completion.error.empty() ? "" : ": " + completion.error));
            }
            return;
        }
        takeState(*state);
        if (kind == kProbe)
        {
            for (auto& joint : joints_) joint.command = joint.position;  // hold where it is
            phase_ = Phase::Running;
            initialized_ = true;
            attempts_ = 0;
            last_command_time_ = std::chrono::steady_clock::now();
            markReady();
        }
    }
}  // namespace end_effector_hardware
